// M6: RotorDegradationSystem. See RotorDegradationSystem.cc for the full
// design rationale (relay architecture, topic naming, thread-safety notes).
#ifndef AERO_ROTOR_DEGRADATION_SYSTEM_HH_
#define AERO_ROTOR_DEGRADATION_SYSTEM_HH_

#include <atomic>
#include <cstdint>
#include <string>

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/param.pb.h>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace aero
{

/// A gz-sim System that sits between PX4's bridge and Gazebo's stock
/// MulticopterMotorModel plugins, relaying commanded rotor velocities
/// unchanged except for one, optionally-faulted rotor. See the module
/// comment in RotorDegradationSystem.cc.
class RotorDegradationSystem
  : public gz::sim::System,
    public gz::sim::ISystemConfigure
{
  public: void Configure(
              const gz::sim::Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              gz::sim::EntityComponentManager &_ecm,
              gz::sim::EventManager &_eventMgr) override;

  /// Relays every Actuators command from the real PX4-published topic to
  /// the faulted topic MulticopterMotorModel actually listens to, scaling
  /// the currently-faulted rotor's commanded velocity (if any). Also drives
  /// the periodic status heartbeat (throttled -- see .cc).
  private: void OnCommand(const gz::msgs::Actuators &_msg);

  /// Sets/clears which rotor is faulted and at what severity, from a
  /// {rotor_index: int, severity: double} gz.msgs.Param command. Publishes
  /// an immediate status echo on every change -- this, plus OnCommand's
  /// periodic heartbeat, is what lets a Python caller confirm a fire-and-
  /// forget publish actually took effect rather than assuming it did.
  private: void OnFaultCmd(const gz::msgs::Param &_msg);

  private: void PublishStatus();

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher relayPub;
  private: gz::transport::Node::Publisher statusPub;

  private: std::string modelName;

  /// -1 == no rotor currently faulted. std::atomic, not a mutex: the two
  /// fields are read together in OnCommand's hot path (every relayed
  /// Actuators message) and written rarely (only on an OnFaultCmd call).
  /// A full mutex would serialize the hot path against a write that
  /// happens a few times per episode at most; the accepted trade is a
  /// microsecond-scale window, on a fault *change*, where OnCommand could
  /// read a still-old rotor index paired with an already-new severity (or
  /// vice versa) -- physically indistinguishable from the fault command
  /// arriving one relay tick earlier or later, which is already inherent
  /// to an async, fire-and-forget control channel. Documented here rather
  /// than silently assumed safe.
  private: std::atomic<int32_t> faultedRotor{-1};
  private: std::atomic<double> faultSeverity{0.0};

  /// Throttle counter for OnCommand's piggybacked heartbeat -- see .cc.
  private: std::atomic<uint64_t> commandCount{0};
};

}  // namespace aero

#endif
