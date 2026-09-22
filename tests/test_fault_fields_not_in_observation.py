"""M6 task 2: the test that exists specifically for CLAUDE.md §1.7 /
anti-pattern 12 -- ground-truth fault state must never be a policy input,
reward shaping only. Written before anything else in M6 touched the schema
(milestones.md M6), the same way M5 wrote its shared-feature test first.

No simulator needed -- this is a pure config/schema consistency check.
"""
import yaml

from experiments.episode_schema import load_schema

OBSERVATION_V1_PATH = "configs/rl/observation_v1.yaml"
FEATURES_PATH = "configs/features.yaml"

# Every fault-related field schema v4 adds (configs/schema/episode_record.yaml).
# Hand-listed rather than derived from the schema itself, so a future schema
# change can't accidentally narrow what this test checks -- if a new
# fault_* field is added and this list isn't updated, the "every fault_*
# episode field" test below will fail loudly and point straight here.
_FAULT_EPISODE_FIELDS = (
    "fault_config_digest",
    "fault_applied",
    "fault_type",
    "fault_rotor_index",
    "fault_severity_commanded",
    "fault_onset_time_s_requested",
    "fault_onset_time_s_observed",
    "fault_profile",
    "fault_ramp_duration_s",
    "fault_confirmed_applied",
    "fault_confirmed_severity_final",
    "px4_failure_detector_silent",
)
_FAULT_STEP_FIELDS = ("px4_failure_detector_status",)


def test_every_fault_episode_field_is_accounted_for():
    """Pins _FAULT_EPISODE_FIELDS above equal to the schema's actual set of
    fault_*/px4_failure_detector_* episode fields, so this test file cannot
    silently go stale as the schema evolves."""
    schema = load_schema()
    actual = {
        f for f in schema["episode_fields"]["required"]
        if f.startswith("fault_") or f.startswith("px4_failure_detector_")
    }
    assert actual == set(_FAULT_EPISODE_FIELDS)


def test_no_fault_field_appears_in_observation_v1():
    obs = yaml.safe_load(open(OBSERVATION_V1_PATH))
    obs_features = set(obs["features"])
    leaked = obs_features & (set(_FAULT_EPISODE_FIELDS) | set(_FAULT_STEP_FIELDS))
    assert not leaked, (
        f"configs/rl/observation_v1.yaml references ground-truth fault field(s) "
        f"{leaked} -- CLAUDE.md §1.7/anti-pattern 12: fault state may shape the "
        f"training reward only, never the policy observation")


def test_no_fault_field_is_marked_shared_in_features_yaml():
    cfg = yaml.safe_load(open(FEATURES_PATH))
    shared_names = {f["name"] for f in cfg["features"] if f["side"] == "shared"}
    leaked = shared_names & (set(_FAULT_EPISODE_FIELDS) | set(_FAULT_STEP_FIELDS))
    assert not leaked, (
        f"configs/features.yaml marks ground-truth fault field(s) {leaked} as "
        f"side: shared -- a shared feature can end up in the policy observation "
        f"(configs/rl/observation_v1.yaml), which fault state must never do")
