"""Closed-form inverse kinematics for the planar 3-DOF arm."""
import numpy as np
from src.config import LINKS_3DOF
from src.utils.planar import two_link_ik


def inverse_kinematics(x, y, phi, links=LINKS_3DOF, elbow_up=True):
    """Joint angles [t1, t2, t3] that put the tip at (x, y) pointing along phi.

    Idea: step back from the target along the last link to find where the
    wrist has to be. After that it's just a 2-link problem for joints 1 and 2,
    and joint 3 takes up whatever orientation is left.
    """
    l1, l2, l3 = links

    wx = x - l3 * np.cos(phi)
    wy = y - l3 * np.sin(phi)

    t1, t2 = two_link_ik(wx, wy, l1, l2, elbow_up=elbow_up)
    t3 = phi - t1 - t2
    return np.array([t1, t2, t3])


if __name__ == "__main__":
    q = inverse_kinematics(25.0, 15.0, np.radians(40))
    print("joint angles (deg):", np.degrees(q))
