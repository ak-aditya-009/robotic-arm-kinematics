import numpy as np
import pytest

from src.config import LIMITS_4DOF, LINKS_4DOF, D1_4DOF
from src.arm_4dof.forward_kinematics import forward_kinematics, joint_positions, transform
from src.arm_4dof.inverse_kinematics import inverse_kinematics


def test_fk_arm_straight_up():
    # all pitch joints at 90 deg: arm points straight up
    q = [0, np.pi / 2, 0, 0]
    pose = forward_kinematics(q)
    assert np.allclose(pose[:3], [0, 0, D1_4DOF + sum(LINKS_4DOF)], atol=1e-9)


def test_fk_arm_flat_along_x():
    pose = forward_kinematics([0, 0, 0, 0])
    assert np.allclose(pose[:3], [sum(LINKS_4DOF), 0, D1_4DOF])


def test_base_rotation_turns_the_arm():
    a = forward_kinematics([0, 0.3, -0.5, 0.2])
    b = forward_kinematics([np.pi / 2, 0.3, -0.5, 0.2])
    # same radius and height, just rotated 90 degrees about z
    assert np.isclose(np.hypot(a[0], a[1]), np.hypot(b[0], b[1]))
    assert np.isclose(a[2], b[2])
    assert np.isclose(b[0], 0, atol=1e-9)


def test_dh_matches_closed_form_positions():
    q = np.radians([25, 60, -70, -20])
    assert np.allclose(joint_positions(q)[-1], forward_kinematics(q)[:3])


def test_rotation_part_is_valid():
    R = transform(np.radians([10, 20, 30, 40]))[:3, :3]
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
    assert np.isclose(np.linalg.det(R), 1.0)


@pytest.mark.parametrize("seed", range(20))
def test_round_trip(seed):
    rng = np.random.default_rng(seed)

    # IK assumes the tip is on the front side of the base axis (r >= 0),
    # so skip random configs where the arm has folded over backwards
    while True:
        q = np.array([rng.uniform(lo, hi) for lo, hi in LIMITS_4DOF])
        pose = forward_kinematics(q)
        if joint_positions(q)[-1][:2] @ [np.cos(q[0]), np.sin(q[0])] >= 0:
            break

    q_back = inverse_kinematics(*pose, elbow_up=(q[2] < 0))
    assert np.allclose(forward_kinematics(q_back), pose, atol=1e-8)


def test_target_above_base_axis_does_not_crash():
    q = inverse_kinematics(0.0, 0.0, 30.0, np.pi / 2)
    assert np.all(np.isfinite(q))


def test_unreachable_target_raises():
    with pytest.raises(ValueError):
        inverse_kinematics(80.0, 0.0, 10.0, 0.0)
