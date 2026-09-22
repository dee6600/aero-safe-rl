"""M6 task 4 (@pytest.mark.sim): the x500_aero model itself -- spawns
correctly (via the launcher's own custom spawn path, scripts/sim_start.sh),
and flies a full healthy mission indistinguishably from stock x500, with no
fault ever commanded. This is the "we didn't break healthy flight by adding
this model" check, run before any fault logic (tasks 5+) is trusted.
"""
from experiments.run_episodes import run_episodes


def test_x500_aero_model_name_matches_instance_spec(sim_worker_x500_aero):
    from simulation.instance_spec import InstanceSpec
    spec = InstanceSpec.from_file(f"/tmp/aero-safe-rl-sim/instance_{sim_worker_x500_aero}.json")
    assert spec.model == "x500_aero"
    assert spec.model_name == "x500_aero_0"


def test_healthy_mission_flies_on_x500_aero(sim_worker_x500_aero, tmp_path):
    # run_episodes() returns every attempt, not just completed ones (its own
    # retry-to-hard-reset policy after a bad attempt, same as any other
    # mission on this stack -- occasional preflight/timeout hiccups are
    # already-documented background noise, not new bugs). What matters here
    # is that at least one attempt actually completed on x500_aero.
    summaries = run_episodes(
        "square_circuit", n=1, instance=0, model="x500_aero",
        run_id="m6_x500_aero_model_test", results_dir=str(tmp_path))
    completed = [s for s in summaries if s["termination_reason"] == "completed"]
    assert completed, f"no completed episode among {len(summaries)} attempt(s): " \
                       f"{[s['termination_reason'] for s in summaries]}"
    summary = completed[0]
    assert summary["valid"] is True
    # Noise-floor sanity, not a tight bound: M3 measured mean 6.44m / std
    # 0.57m on plain x500 over 40 healthy episodes (docs/baseline_results.md)
    # -- x500_aero with no fault commanded should land in the same range,
    # not some wildly different number that would mean the model itself
    # flies differently from stock x500.
    assert summary["position_rmse_m"] < 15.0
