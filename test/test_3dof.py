import numpy as np
import pytest

from src.config import LIMITS_3DOF
from src.arm_3dof.forward_kinematics import forward_kinematics, joint_positions
from src.arm_3dof.inverse_kinematics import inverse_kinematics


def test_fk_straight_arm():
    # everything lined up along x, so the tip is at the sum of the links
    pose = forward_kinematics([0, 0, 0], links=(2, 3, 4))
    assert np.allclose(pose, [9, 0, 0])


def test_fk_folded_back():
    pose = forward_kinematics([0, np.pi, 0], links=(2, 2, 1))
    assert np.allclose(pose[:2], [-1, 0], atol=1e-9)


def test_joint_positions_end_matches_fk():
    q = np.radians([40, -30, 20])
    assert np.allclose(joint_positions(q)[-1], forward_kinematics(q)[:2])


@pytest.mark.parametrize("seed", range(20))
def test_round_trip(seed):
    rng = np.random.default_rng(seed)
    q = np.array([rng.uniform(lo, hi) for lo, hi in LIMITS_3DOF])

    pose = forward_kinematics(q)
    q_back = inverse_kinematics(*pose, elbow_up=(q[1] < 0))

    # compare poses instead of angles, avoids trouble with 2*pi wrapping
    assert np.allclose(forward_kinematics(q_back), pose, atol=1e-8)


def test_both_elbow_solutions_reach_target():
    target = (22.0, 12.0, np.radians(30))
    for up in (True, False):
        q = inverse_kinematics(*target, elbow_up=up)
        assert np.allclose(forward_kinematics(q), target, atol=1e-8)


def test_unreachable_target_raises():
    with pytest.raises(ValueError):
        inverse_kinematics(100.0, 0.0, 0.0)
