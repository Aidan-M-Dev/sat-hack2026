"""Unit tests exercising demo_logic.

These tests can be run with pytest (`pip install pytest`).  They
initially fail against the original, yaw‑biased implementation; the
expectations here reflect the behaviour after the fix, so running them
will guide a red→green transition.
"""

import pytest
from demo_logic import State, step, SET_DISTANCE, MIN_SPEED, MAX_SPEED


def test_forward_speed_equality():
    # With no obstacle repeatedly applied, motors should get equal speeds
    state = State()
    for _ in range(50):
        cmds = step(SET_DISTANCE + 20, state)
        assert len(cmds) == 1
        assert cmds[0][0] == "forward"
        speed = cmds[0][1]
        # previously the sketch added +20 to one motor; we expect equality
        assert speed == state.speed_adjustment


def test_avoidance_sequence():
    state = State()
    cmds = step(SET_DISTANCE - 1, state)
    assert cmds == ["stop", "reverse", "turn_left"]
    assert state.speed_adjustment == MIN_SPEED
    assert state.stop_bit == 1


def test_speed_ramping():
    state = State()
    start = state.speed_adjustment
    # call with safe distance and see it increase but not exceed max
    for _ in range(500):
        step(SET_DISTANCE + 5, state)
    assert state.speed_adjustment <= MAX_SPEED
    assert state.speed_adjustment > start


if __name__ == "__main__":
    pytest.main([__file__])
