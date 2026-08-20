#!/usr/bin/env python3
"""Arm, take off to 5m, hover, and land -- prints a pass/fail checklist.

Connects as a MAVLink GCS (same approach used to validate M1), so it works
standalone without the ROS 2 bridge (that's M2). Meant to be run against an
already-running instance started by sim_start.sh / sim_watch.sh.
"""
import argparse
import statistics
import sys
import threading
import time

from pymavlink import mavutil

PX4_BIN_DIR = "/home/dee/projects/PX4-Autopilot/build/px4_sitl_default/bin"


class Heartbeater(threading.Thread):
    """Sends our own HEARTBEAT so PX4 considers a GCS connected."""

    def __init__(self, conn):
        super().__init__(daemon=True)
        self.conn = conn
        self._stop = threading.Event()

    def run(self):
        while not self._stop.is_set():
            self.conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self._stop.wait(1.0)

    def stop(self):
        self._stop.set()


def commander(instance, *args):
    import subprocess
    cmd = [f"{PX4_BIN_DIR}/px4-commander", *args]
    if instance:
        cmd += ["--instance", str(instance)]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
    return r


def param_set(instance, name, value):
    import subprocess
    cmd = [f"{PX4_BIN_DIR}/px4-param", "set", name, str(value)]
    if instance:
        cmd += ["--instance", str(instance)]
    subprocess.run(cmd, capture_output=True, text=True, timeout=20)


def check(label, ok, detail=""):
    mark = "PASS" if ok else "FAIL"
    print(f"  [{mark}] {label}" + (f" -- {detail}" if detail else ""))
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--instance", type=int, default=0)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--hover-alt", type=float, default=5.0)
    args = ap.parse_args()

    gcs_port = 14550 + args.instance if args.instance > 9 else 14550
    # GCS remote port is fixed at 14550 for instance 0; only offsets past
    # instance 9 (see ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink).
    if args.instance <= 9:
        gcs_port = 14550

    all_ok = True

    print(f"Connecting as GCS on udp:localhost:{gcs_port} (instance {args.instance})...")
    conn = mavutil.mavlink_connection(f'udpin:localhost:{gcs_port}')
    hb = conn.wait_heartbeat(timeout=30)
    all_ok &= check("MAVLink heartbeat received from PX4", hb is not None)
    if hb is None:
        print("Cannot continue without a heartbeat.")
        sys.exit(1)

    hbeater = Heartbeater(conn)
    hbeater.start()
    time.sleep(2)  # let PX4 register the GCS link as connected

    try:
        param_set(args.instance, "MIS_TAKEOFF_ALT", args.hover_alt)
        time.sleep(0.5)

        commander(args.instance, "arm", "-f")
        time.sleep(1)
        status = commander(args.instance, "status")
        all_ok &= check("Armed", "Armed" in status.stdout)

        commander(args.instance, "takeoff")

        deadline = time.time() + max(20, 10 * args.speed) / args.speed + 15
        altitudes = []
        max_alt = 0.0
        failsafe_seen = False
        while time.time() < deadline:
            msg = conn.recv_match(type=["LOCAL_POSITION_NED", "STATUSTEXT"], blocking=True, timeout=2)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                if "failsafe" in msg.text.lower() or "rtl" in msg.text.lower():
                    failsafe_seen = True
                continue
            alt = -msg.z
            max_alt = max(max_alt, alt)
            if alt > args.hover_alt * 0.85:
                altitudes.append(alt)
                if len(altitudes) > 20:
                    break

        all_ok &= check(f"Reached takeoff altitude (target {args.hover_alt:.1f}m)",
                         max_alt >= args.hover_alt * 0.85, f"max_alt={max_alt:.2f}m")
        if altitudes:
            hover_std = statistics.pstdev(altitudes)
            all_ok &= check("Stable hover (altitude stdev < 0.5m)", hover_std < 0.5,
                             f"stdev={hover_std:.3f}m")
        all_ok &= check("No EKF/failsafe warnings during flight", not failsafe_seen)

        commander(args.instance, "land")
        land_deadline = time.time() + max(20, 10 * args.speed) / args.speed + 15
        disarmed = False
        while time.time() < land_deadline:
            st = commander(args.instance, "status")
            if "Disarmed" in st.stdout:
                disarmed = True
                break
            time.sleep(1)
        all_ok &= check("Landed and disarmed", disarmed)

    finally:
        hbeater.stop()
        conn.close()

    print()
    if all_ok:
        print("All checks passed.")
    else:
        print("Some checks failed -- see [FAIL] lines above.")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
