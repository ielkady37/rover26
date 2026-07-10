import math
from typing import Tuple


def sanitize_float(value: float, default: float = 0.0) -> float:
    """Return a finite float, falling back to a safe default when needed."""
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return float(default)

    if not math.isfinite(numeric):
        return float(default)

    return numeric


def sanitize_quaternion(x: float, y: float, z: float, w: float) -> Tuple[float, float, float, float]:
    """Return a valid quaternion with a safe fallback when data is malformed."""
    x = sanitize_float(x, 0.0)
    y = sanitize_float(y, 0.0)
    z = sanitize_float(z, 0.0)
    w = sanitize_float(w, 1.0)

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 0.0:
        return 0.0, 0.0, 0.0, 1.0

    if abs(norm - 1.0) > 1e-6:
        x /= norm
        y /= norm
        z /= norm
        w /= norm

    # Normalize again to guard against tiny rounding issues.
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 0.0:
        return 0.0, 0.0, 0.0, 1.0

    return x / norm, y / norm, z / norm, w / norm
