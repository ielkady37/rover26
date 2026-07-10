import math

from control.services.value_safety import sanitize_float, sanitize_quaternion


def test_sanitize_float_replaces_non_finite_values() -> None:
    assert sanitize_float(float("nan"), default=1.0) == 1.0
    assert sanitize_float(float("inf"), default=-2.0) == -2.0
    assert sanitize_float(3.5) == 3.5


def test_sanitize_quaternion_replaces_invalid_or_non_unit_values() -> None:
    q = sanitize_quaternion(float("nan"), 0.0, 0.0, 1.0)
    assert q == (0.0, 0.0, 0.0, 1.0)

    q = sanitize_quaternion(0.0, 0.0, 0.0, 0.0)
    assert q == (0.0, 0.0, 0.0, 1.0)

    q = sanitize_quaternion(0.0, 0.0, 0.5, math.sqrt(0.75))
    assert q[0] == 0.0
    assert q[1] == 0.0
    assert q[2] == 0.5
    assert q[3] == math.sqrt(0.75)
