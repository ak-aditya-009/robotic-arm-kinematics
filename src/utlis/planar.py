import numpy as np


def two_link_ik(x, y, l1, l2, elbow_up=True):
    """Solve a planar 2-link arm for a point (x, y).

    Returns (t1, t2) in radians. Raises ValueError when the point
    can't be reached. "Elbow up" means the elbow sits above the line
    from the base to the target, which ends up as a negative t2.
    """
    c2 = (x * x + y * y - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)

    # small tolerance so points right on the boundary don't blow up
    if abs(c2) > 1 + 1e-9:
        raise ValueError(f"point ({x:.2f}, {y:.2f}) is outside the reachable area")
    c2 = np.clip(c2, -1.0, 1.0)

    s2 = np.sqrt(1 - c2 ** 2)
    if elbow_up:
        s2 = -s2

    t2 = np.arctan2(s2, c2)
    t1 = np.arctan2(y, x) - np.arctan2(l2 * s2, l1 + l2 * c2)
    return t1, t2
