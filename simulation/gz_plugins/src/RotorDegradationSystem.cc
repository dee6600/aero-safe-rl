// M6: RotorDegradationSystem -- the Gazebo-side half of the rotor-
// degradation fault model (CLAUDE.md §1.4/§1.6, D2). PX4 never learns
// about this: it only feels the aircraft behaving oddly, exactly as D2
// requires.
//
// Architecture: a RELAY, not a physics reimplementation. Gazebo's stock
// gz-sim-multicopter-motor-model-system plugin fixes its thrust constant
// (motorConstant) at model load, so it cannot express a live-settable
// per-rotor efficiency on its own (see milestones.md M6's design note).
// This system does not replace it -- the x500_aero model (simulation/
// models/x500_aero/model.sdf) keeps the same 4 stock MulticopterMotorModel
// instances as PX4's own x500, so all real thrust/torque physics stays
// Gazebo-validated. This system instead sits between PX4's bridge (which
// publishes commanded rotor velocities as gz.msgs.Actuators on
// /<model_name>/command/motor_speed -- confirmed from PX4 v1.17.0's
// GZMixingInterfaceESC.cpp, not guessed) and the stock motor-model plugins
// (repointed, in x500_aero's SDF, to a relayed topic instead of the real
// one): every rotor's commanded velocity passes through unchanged except
// the currently-faulted one, scaled by sqrt(1 - severity). Since stock
// thrust is proportional to velocity^2 (and reaction torque to thrust, via
// momentConstant), this scales both thrust and reaction torque by exactly
// efficiency = 1 - severity, using Gazebo's own already-validated physics
// rather than re-deriving a thrust model.
//
// Control/status channel: gz.msgs.Param (a generic string-keyed variant
// map), not a new custom .proto message -- its C++ headers ship with
// libgz-msgs10-dev (already a transitive dependency of libgz-sim8-dev), and
// its Python bindings are already used twice in this repo
// (simulation/sim_clock.py, aero_bridge/reset.py) for exactly this kind of
// ad-hoc runtime control channel.
//
// "Confirm, don't assume": a gz-transport publish is fire-and-forget, so a
// Python caller (simulation/rotor_fault.py) cannot know a fault command was
// actually applied just because Publish() returned true. This system
// echoes the ACTUALLY-applied (rotor, severity) back on a status topic on
// every change, plus a periodic heartbeat piggybacked on the relay's own
// (already-continuous) callback rate -- see OnCommand.
#include "RotorDegradationSystem.hh"

#include <algorithm>
#include <cmath>
#include <iostream>

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>

using namespace aero;

void RotorDegradationSystem::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> & /*_sdf*/,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
  gz::sim::Model model(_entity);
  this->modelName = model.Name(_ecm);

  // Absolute topic strings, built to match PX4's own bridge exactly
  // (GZMixingInterfaceESC.cpp: "/" + model_name + "/command/motor_speed",
  // no "/model/" prefix) rather than relying on gz-sim's relative-topic-
  // in-plugin scoping rules, which this system does not independently
  // re-derive.
  const std::string base = "/" + this->modelName;
  const std::string realCommandTopic = base + "/command/motor_speed";
  const std::string relayedCommandTopic = base + "/command/motor_speed_faulted";
  const std::string faultCmdTopic = base + "/rotor_fault/cmd";
  const std::string faultStatusTopic = base + "/rotor_fault/status";

  this->relayPub = this->node.Advertise<gz::msgs::Actuators>(relayedCommandTopic);
  this->statusPub = this->node.Advertise<gz::msgs::Param>(faultStatusTopic);

  this->node.Subscribe(realCommandTopic, &RotorDegradationSystem::OnCommand, this);
  this->node.Subscribe(faultCmdTopic, &RotorDegradationSystem::OnFaultCmd, this);

  std::cerr << "[RotorDegradationSystem] configured for model '" << this->modelName
            << "': relaying " << realCommandTopic << " -> " << relayedCommandTopic
            << ", fault control on " << faultCmdTopic << std::endl;

  // One immediate heartbeat at startup, before the first real Actuators
  // message ever arrives -- Python's wait_for_heartbeat() should not have
  // to wait for PX4 to start publishing motor commands just to learn the
  // plugin itself loaded.
  this->PublishStatus();
}

void RotorDegradationSystem::OnCommand(const gz::msgs::Actuators &_msg)
{
  gz::msgs::Actuators relayed = _msg;

  const int32_t rotor = this->faultedRotor.load(std::memory_order_relaxed);
  if (rotor >= 0 && rotor < relayed.velocity_size())
  {
    const double severity = this->faultSeverity.load(std::memory_order_relaxed);
    const double efficiency = std::clamp(1.0 - severity, 0.0, 1.0);
    const double scale = std::sqrt(efficiency);
    relayed.set_velocity(rotor, relayed.velocity(rotor) * scale);
  }

  this->relayPub.Publish(relayed);

  // Piggyback the heartbeat on the relay's own callback rate rather than
  // running a dedicated timer thread: PX4's bridge publishes motor
  // commands continuously once it is up, even before arming, so this alone
  // is enough to give wait_for_heartbeat() a liveness signal within a
  // fraction of a second. Throttled so a heartbeat-only status message
  // isn't published on literally every relayed command.
  if (this->commandCount.fetch_add(1, std::memory_order_relaxed) % 50 == 0)
  {
    this->PublishStatus();
  }
}

void RotorDegradationSystem::OnFaultCmd(const gz::msgs::Param &_msg)
{
  int32_t rotor = -1;
  double severity = 0.0;

  const auto &params = _msg.params();
  auto rotorIt = params.find("rotor_index");
  if (rotorIt != params.end())
  {
    rotor = rotorIt->second.int_value();
  }
  auto severityIt = params.find("severity");
  if (severityIt != params.end())
  {
    severity = severityIt->second.double_value();
  }

  this->faultedRotor.store(rotor, std::memory_order_relaxed);
  this->faultSeverity.store(severity, std::memory_order_relaxed);

  // Immediate echo on every change -- do not wait for the next throttled
  // heartbeat, since a caller applying a fault mid-episode needs to
  // confirm it took effect promptly, not up to ~50 relay ticks later.
  this->PublishStatus();
}

void RotorDegradationSystem::PublishStatus()
{
  gz::msgs::Param status;
  auto &params = *status.mutable_params();

  const int32_t rotor = this->faultedRotor.load(std::memory_order_relaxed);
  const double severity = this->faultSeverity.load(std::memory_order_relaxed);

  auto &rotorEntry = params["rotor_index"];
  rotorEntry.set_type(gz::msgs::Any::INT32);
  rotorEntry.set_int_value(rotor);

  auto &severityEntry = params["severity"];
  severityEntry.set_type(gz::msgs::Any::DOUBLE);
  severityEntry.set_double_value(severity);

  auto &appliedEntry = params["applied"];
  appliedEntry.set_type(gz::msgs::Any::BOOLEAN);
  appliedEntry.set_bool_value(rotor >= 0);

  this->statusPub.Publish(status);
}

GZ_ADD_PLUGIN(
    aero::RotorDegradationSystem,
    gz::sim::System,
    gz::sim::ISystemConfigure)
