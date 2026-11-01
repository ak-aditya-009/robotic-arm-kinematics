import numpy as np


def within_limits(q, limits):
    """True if every joint angle is inside its (min, max) range."""
    q = np.asarray(q)
    return all(lo <= v <= hi for v, (lo, hi) in zip(q, limits))


def wrap(angle):
    """Wrap an angle to (-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi
