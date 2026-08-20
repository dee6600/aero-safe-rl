# PX4 patches

PX4 stays stock (`planning.md` principle #3) — no in-place edits to
`~/projects/PX4-Autopilot`. When a patch is genuinely unavoidable, the diff
lives here, tracked in this repo, and is also committed onto the
`aero-safe-rl` branch inside the PX4-Autopilot checkout itself (so
`git status` there stays clean, satisfying M5's "zero uncommitted
modifications" check). This directory is the reproducibility record: apply
these on top of a fresh `v1.17.0` checkout to reproduce the exact build this
project uses.

## Patches

### `0001-expose-actuator-motors-outputs-over-dds.patch`

**Why:** PX4 v1.17.0's default `dds_topics.yaml` does not publish
`ActuatorMotors` or `ActuatorOutputs` over the ROS 2 bridge at all —
`actuator_motors` only exists as an *input* topic (`/fmu/in/actuator_motors`,
for external actuator override), and `actuator_outputs` isn't listed in
either direction. Found while verifying M2's required topic list against the
actual pinned build, per the standing project rule to verify real values,
not just topic presence.

This matters beyond M2: M4's most important feature (the thrust-vs-achieved-
acceleration residual) and "each motor's normalised output" both need this
data. Fixing it now avoids rediscovering the same gap during M4.

**What it does:** adds two `publications` entries to
`src/modules/uxrce_dds_client/dds_topics.yaml`, exposing
`/fmu/out/actuator_motors` and `/fmu/out/actuator_outputs` at 50 Hz.

**To apply:**
```bash
cd ~/projects/PX4-Autopilot
git apply ~/projects/aero-safe-rl/simulation/patches/0001-expose-actuator-motors-outputs-over-dds.patch
make px4_sitl   # regenerates the DDS bridge code and relinks
```
