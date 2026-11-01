# Kinematics derivations

## 3-DOF planar arm

### Forward kinematics

With absolute link angles a1 = t1, a2 = t1 + t2, a3 = t1 + t2 + t3:

```
x   = l1 cos(a1) + l2 cos(a2) + l3 cos(a3)
y   = l1 sin(a1) + l2 sin(a2) + l3 sin(a3)
phi = t1 + t2 + t3
```

### Inverse kinematics

Given (x, y, phi) there are three unknowns and three equations, so the
problem is solvable in closed form.

1. Find the wrist position by stepping back along the last link:

   ```
   wx = x - l3 cos(phi)
   wy = y - l3 sin(phi)
   ```

2. Joints 1 and 2 now form a regular 2-link arm reaching (wx, wy). By the law
   of cosines:

   ```
   cos(t2) = (wx^2 + wy^2 - l1^2 - l2^2) / (2 l1 l2)
   ```

   If |cos(t2)| > 1 the wrist is out of reach and the code raises a
   `ValueError`.

3. Two solutions exist, one for each sign of sin(t2):

   ```
   sin(t2) = +/- sqrt(1 - cos(t2)^2)
   t2      = atan2(sin(t2), cos(t2))
   t1      = atan2(wy, wx) - atan2(l2 sin(t2), l1 + l2 cos(t2))
   ```

   "Elbow up" uses the negative sine.

4. The last joint fills in the orientation: `t3 = phi - t1 - t2`.

## 4-DOF arm

### Forward kinematics

Multiply the four DH matrices from `dh_parameters.md`. Because joints 2 to 4
all move in one vertical plane the result simplifies to:

```
r   = l2 cos(q2) + l3 cos(q2+q3) + l4 cos(q2+q3+q4)
z   = d1 + l2 sin(q2) + l3 sin(q2+q3) + l4 sin(q2+q3+q4)
x   = r cos(q1)
y   = r sin(q1)
phi = q2 + q3 + q4
```

The test suite checks that the DH product and these formulas agree.

### Inverse kinematics

1. Base angle from the top view: `q1 = atan2(y, x)` and `r = sqrt(x^2 + y^2)`.
2. Wrist in the vertical plane:

   ```
   wr = r - l4 cos(phi)
   wz = (z - d1) - l4 sin(phi)
   ```

3. Solve the 2-link problem (l2, l3) for (wr, wz) to get q2 and q3, same as
   step 2-3 of the planar case.
4. `q4 = phi - q2 - q3`.

### Singular / special cases

- Target on the base axis (x = y = 0): q1 is undefined, the code returns 0.
- Arm fully stretched or fully folded: the two elbow solutions merge and
  small position errors can turn into big joint changes.
- Only solutions with the tip in front of the base axis (r >= 0) are
  produced.

## Trajectory planning

Joint-space moves use a time-scaling function s(tau), tau from 0 to 1:

```
cubic:   s = 3 tau^2 - 2 tau^3
quintic: s = 10 tau^3 - 15 tau^4 + 6 tau^5
```

and then `q(tau) = q_start + s (q_end - q_start)`. Both start and end with
zero velocity, quintic also has zero acceleration there.

For Cartesian moves the same scaling is applied along a straight line between
two poses, and IK is solved at every sample (`path_to_joints`).
