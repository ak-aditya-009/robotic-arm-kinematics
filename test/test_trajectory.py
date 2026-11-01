import numpy as np

from src.trajectory.planner import (
    time_scaling, joint_trajectory, line_path, path_to_joints, waypoint_path,
)
from src.arm_4dof.forward_kinematics import forward_kinematics
from src.arm_4dof.inverse_kinematics import inverse_kinematics


def test_scaling_starts_at_zero_ends_at_one():
    for prof in ("linear", "cubic", "quintic"):
        s = time_scaling(30, prof)
        assert np.isclose(s[0], 0) and np.isclose(s[-1], 1)


def test_scaling_is_monotonic():
    s = time_scaling(100, "quintic")
    assert np.all(np.diff(s) >= 0)


def test_joint_trajectory_endpoints():
    traj = joint_trajectory([0, 0, 0], [1, -2, 0.5], n=25)
    assert np.allclose(traj[0], [0, 0, 0])
    assert np.allclose(traj[-1], [1, -2, 0.5])
    assert traj.shape == (25, 3)


def test_quintic_has_zero_velocity_at_ends():
    traj = joint_trajectory([0], [1], n=200)
    v = np.diff(traj[:, 0])
    assert v[0] < 1e-4 and v[-1] < 1e-4


def test_line_path_midpoint():
    path = line_path([0, 0, 0], [10, 4, 2], n=51, profile="linear")
    assert np.allclose(path[25], [5, 2, 1])


def test_waypoint_path_has_no_duplicate_points():
    wps = [[0, 0], [1, 0], [1, 1]]
    path = waypoint_path(wps, n_per_segment=10)
    assert len(path) == 19


def test_cartesian_path_followed_by_arm():
    path = line_path([20, 10, 12, -np.pi / 2], [20, 10, 2, -np.pi / 2], n=30)
    joints = path_to_joints(path, inverse_kinematics)
    for q, p in zip(joints, path):
        assert np.allclose(forward_kinematics(q), p, atol=1e-8)
