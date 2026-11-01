# DH parameters

Standard Denavit-Hartenberg convention. Lengths in cm, `alpha` in radians.
Link lengths come from `src/config.py`.

## 3-DOF planar arm

All three joints rotate about parallel axes, so every `alpha` and `d` is zero.

| i | theta_i | d_i | a_i | alpha_i |
|---|---------|-----|-----|---------|
| 1 | t1      | 0   | l1  | 0       |
| 2 | t2      | 0   | l2  | 0       |
| 3 | t3      | 0   | l3  | 0       |

Default links: l1 = 20, l2 = 15, l3 = 10.

For this arm the closed-form FK in `forward_kinematics.py` is short enough
that it doesn't go through the DH matrices.

## 4-DOF arm (base yaw + 3 pitch joints)

| i | theta_i | d_i | a_i | alpha_i |
|---|---------|-----|-----|---------|
| 1 | q1      | d1  | 0   | pi/2    |
| 2 | q2      | 0   | l2  | 0       |
| 3 | q3      | 0   | l3  | 0       |
| 4 | q4      | 0   | l4  | 0       |

Default values: d1 = 10, l2 = 20, l3 = 15, l4 = 10.

The `pi/2` twist on joint 1 turns the z axis of frame 1 so it lies flat. That
makes q2, q3 and q4 bend in a vertical plane, and q2 is measured up from the
horizontal.

## Joint limits

| Arm   | Joint | Range (deg) |
|-------|-------|-------------|
| 3-DOF | t1    | 0 to 180    |
| 3-DOF | t2    | -150 to 150 |
| 3-DOF | t3    | -120 to 120 |
| 4-DOF | q1    | -180 to 180 |
| 4-DOF | q2    | 0 to 180    |
| 4-DOF | q3    | -150 to 150 |
| 4-DOF | q4    | -120 to 120 |

These are just reasonable values for a hobby-servo style arm. Change them in
`src/config.py` to match the real hardware.
