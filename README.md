<<<<<<< HEAD
# Robotic Arm Design & Kinematics

Kinematics code for two small robotic arms: a planar 3-DOF arm and a 4-DOF
arm (base rotation plus three pitch joints). It covers forward and inverse
kinematics, trajectory planning for pick-and-place, and some plots of the
workspace.

![4-DOF pick and place](results/4dof_pick_and_place.png)

## What's in here

- Forward kinematics for both arms (the 4-DOF one uses DH matrices)
- Closed-form inverse kinematics, with elbow-up / elbow-down choice
- Trajectory planning: cubic and quintic joint moves, straight-line Cartesian
  paths, multi-waypoint paths turned into joint angles through IK
- Workspace plots and a pick-and-place demo
- Unit tests, including FK -> IK round trips on random joint angles

## Project layout

```
robotic-arm-kinematics/
├── cad/                  structural design files (add yours here)
├── docs/                 DH tables and derivations
├── examples/             demo scripts that produce the plots
├── results/              generated figures
├── src/
│   ├── config.py         link lengths and joint limits
│   ├── arm_3dof/         FK and IK for the planar arm
│   ├── arm_4dof/         FK and IK for the 4-DOF arm
│   ├── trajectory/       trajectory helpers
│   └── utils/            DH matrices, 2-link solver, plotting, limits
└── tests/
```

## Setup

```bash
git clone https://github.com/<your-username>/robotic-arm-kinematics.git
cd robotic-arm-kinematics
python -m venv .venv
source .venv/bin/activate        # on Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

## Usage

Forward kinematics of the 3-DOF arm:

```python
import numpy as np
from src.arm_3dof.forward_kinematics import forward_kinematics

pose = forward_kinematics(np.radians([30, 45, 15]))
print(pose)   # [x, y, phi]
```

Inverse kinematics of the 4-DOF arm:

```python
import numpy as np
from src.arm_4dof.inverse_kinematics import inverse_kinematics

# tip at (20, 10, 2) cm, gripper pointing straight down
q = inverse_kinematics(20, 10, 2, -np.pi / 2)
print(np.degrees(q))
```

Run the demos (from the repo root):

```bash
python examples/demo_3dof.py
python examples/demo_4dof.py
```

Run the tests:

```bash
python -m pytest
```

## Results

| 3-DOF: two IK solutions | 3-DOF workspace |
|---|---|
| ![ik](results/3dof_ik_solutions.png) | ![ws](results/3dof_workspace.png) |

| 4-DOF workspace (side view) |
|---|
| ![ws4](results/4dof_workspace.png) |

## Arm parameters

Defaults are in `src/config.py` (cm):

| Arm | Links | Notes |
|-----|-------|-------|
| 3-DOF | 20, 15, 10 | planar |
| 4-DOF | d1 = 10, links 20, 15, 10 | base yaw + 3 pitch joints |

See `docs/dh_parameters.md` for the DH tables and `docs/derivations.md` for
the math.

## Known limitations

- The 4-DOF IK only returns solutions where the arm faces the target. Poses
  with the arm folded backwards over the base are not generated.
- IK does not enforce joint limits itself. Use `utils.limits.within_limits`
  to check the result.
- Trajectories are straight lines in Cartesian space with no collision
  checking, and no velocity or torque limits.
- The link lengths are example values. Update `src/config.py` to match the
  actual CAD model before using the numbers on hardware.

## Ideas for later

- Numerical (Jacobian-based) IK for arms with no closed form
- Singularity checks using the Jacobian determinant
- Animation of the arm following the trajectory
- Serial output to servo drivers

## License

MIT, see `LICENSE`. Remember to put your name in it.
=======
# robotics-arm-kinematics
>>>>>>> c63c1b9759c4c22bafd02fda32b6b20103bf2714
