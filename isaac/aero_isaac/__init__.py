"""The Isaac Lab side of aero-safe-rl (M8b): the environment the recovery
policy trains in. Runs only in the `isaacsim` conda environment
(scripts/activate_isaac.sh).

Nothing here imports the PX4 side (rclpy, aero_bridge, rl/, experiments/,
ai/, simulation/) -- CLAUDE.md §0.1. The two sides share contract files under
configs/ and are held together by tests/fixtures/isaac_contract_v1.json.
isaac/tests/test_boundary.py enforces the rule.
"""
