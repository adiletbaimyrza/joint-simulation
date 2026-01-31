# Physics Simulation — Technical Documentation

This document describes the physics engine in detail: principles, architecture, joint types, constraints, and important code excerpts.

---

## 1. Overview

The project implements a **3D rigid-body physics simulator** with **position-based constraint solving**. Bodies are connected by joints that enforce geometric and kinematic constraints (distance, angles, axes). The solver uses **XPBD-style** (Extended Position-Based Dynamics) ideas: position corrections with **compliance** (soft constraints) and **sub-stepping** for stability.

### Main components

| Component | Role |
|-----------|------|
| **RigidBody** | Position, orientation, velocity, angular velocity, mass, inertia; integration and correction application |
| **Joint** | Connects two bodies (or one body to world); holds constraint parameters; implements `solve_position` and `solve_orientation` |
| **RigidBodySimulator** | Holds bodies and joints; runs sub-stepped loop: integrate → solve joints → update velocities → damping |
| **SceneImporter** | Parses JSON and creates rigid bodies and joints |
| **JointSimulationApp** | Pygame/OpenGL app: input, camera, raycast picking, drag joint, debug overlay |

---

## 2. Principles Used

### 2.1 Position-based dynamics (PBD / XPBD)

- **Positions** (and orientations) are the primary state; constraints are expressed as functions of positions and solved by applying **position corrections**.
- Each constraint has a **violation** (e.g. distance minus rest distance). A correction vector is computed and applied to the bodies so that the violation is reduced.
- Corrections are **weighted by inverse mass** (and inverse inertia at the constraint point) so that heavy bodies move less than light ones.

### 2.2 Compliance (soft constraints)

- **Compliance** \( \alpha \) (in the code, stored as `distance_compliance`, `target_angle_compliance`, etc.) makes the constraint soft: the solver uses \( \lambda = -C / (w + \alpha/\Delta t^2) \) instead of \( \lambda = -C/w \), where \( C \) is the constraint violation and \( w \) is the effective inverse mass.
- **Zero compliance** = hard constraint (no give). **Positive compliance** = spring-like behavior (reduces jitter and allows some penetration).

### 2.3 Sub-stepping

- The simulator divides each frame into **sub-steps** (`num_sub_steps = 10`). Per sub-step:
  1. **Integrate** all bodies (gravity, velocity → position, angular velocity → orientation).
  2. **Solve** all joint constraints (position then orientation).
  3. **Update velocities** from the new positions/orientations (position difference / dt).
  4. **Apply damping** (linear and angular) on each joint.

This improves stability and reduces constraint drift.

### 2.4 Correction application (mass splitting)

- Corrections are applied so that **both bodies** move in a way that satisfies the constraint, proportional to their **inverse mass** (and inverse inertia at the constraint point). The central routine is `RigidBody.apply_correction(compliance, corr, pos, other_body, other_pos, velocity_level)`.

---

## 3. Architecture and Important Code

### 3.1 Simulation loop (`physics_sim/simulator.py`)

Each frame, `simulate()` runs:

```python
def simulate(self):
    self.update_control()

    sdt = self.dt / self.num_sub_steps

    for sub_step in range(self.num_sub_steps):
        for body in self.rigid_bodies:
            body.integrate(sdt, self.gravity)

        for joint in self.joints:
            joint.solve(sdt)

        if self.drag_joint:
            self.drag_joint.solve(sdt)

        for body in self.rigid_bodies:
            body.update_velocities()

        for joint in self.joints:
            joint.apply_linear_damping(sdt)
            joint.apply_angular_damping(sdt)
```

- **Integrate**: semi-implicit Euler (velocity + gravity → position; angular velocity → quaternion).
- **Solve**: each joint runs `solve_position(dt)` then `solve_orientation(dt)` (except distance-only joints).
- **Update velocities**: velocities are derived from position/orientation change over the sub-step (to keep them consistent with the corrected positions).
- **Damping**: linear (along joint axis) and angular damping per joint.

### 3.2 Constraint correction (`physics_sim/rigid_body.py`)

The core of constraint response is **apply_correction**:

```python
def apply_correction(
    self,
    compliance,
    corr,
    pos,
    other_body=None,
    other_pos=None,
    velocity_level=False,
):
    # ...
    C = corr.length()
    normal = corr.normalized()

    w = self.get_inverse_mass(normal, pos)
    if other_body:
        w += other_body.get_inverse_mass(normal, other_pos)

    if not velocity_level:
        alpha = compliance / self.dt / self.dt
        lambda_val = -C / (w + alpha)
    else:
        lambda_val = -C / w

    correction = normal * (-lambda_val)
    self._apply_correction(correction, pos, velocity_level)
    if other_body:
        correction_other = normal * lambda_val
        other_body._apply_correction(correction_other, other_pos, velocity_level)
```

- **corr**: direction and magnitude of the position (or velocity) correction.
- **pos**: application point on this body (for torque); can be `None` for pure angular constraints.
- **velocity_level=True**: applies the correction to velocities only (used for damping).
- **get_inverse_mass(normal, pos)**: returns the effective inverse mass for a correction along `normal` at `pos`, including rotational inertia (\( r \times n \) and inv_inertia).

### 3.3 Joint frame representation (`physics_sim/joint.py`)

Each joint has:

- **global_pos0, global_rot0**: attachment point and frame on body0 (world space).
- **global_pos1, global_rot1**: attachment point and frame on body1 (world space).
- **local_pos0, local_rot0** / **local_pos1, local_rot1**: same in each body’s local space.

The joint’s **axis** (e.g. hinge axis) is the X-axis of the joint frame (`axis0 = Vector3(1,0,0)` transformed by `global_rot0` / `global_rot1`). Constraints are expressed in terms of these global positions and axes.

---

## 4. Joint Types and Constraints

Below, for each joint type we state:

1. **What it does** (physical meaning).
2. **Position constraints** (where `solve_position` applies).
3. **Orientation constraints** (where `solve_orientation` applies).
4. **Parameters** and where they are used.
5. **Relevant code excerpts**.

---

### 4.1 Distance joint

**What it does:** Keeps the distance between the two attachment points at a **rest distance**. Like a rigid rod or a soft spring depending on compliance.

**Position constraints:**

- **1 constraint:** \( \|p_1 - p_0\| = d_{\text{rest}} \).  
  Violation: \( C = \|p_1 - p_0\| - d_{\text{rest}} \). Correction direction: along \( (p_1 - p_0) \) (or opposite), magnitude proportional to \( C \), with compliance.

**Orientation constraints:** None. Rotation is free.

**Parameters:**

- `rest_distance` → `target_distance`
- `compliance` → `distance_compliance`
- `damping` → `linear_damping_coeff`

**Code (solve_position, distance part):**

```python
# joint.py — only position; no orientation solve for DISTANCE
if self.type != JointType.CYLINDER and self.has_target_distance:
    self.update_global_frames()
    corr = self.global_pos1 - self.global_pos0
    distance = corr.length()
    if distance == 0.0:
        corr = Vector3(0, 0, 1)
        corr = self.global_rot0.apply_to_vector(corr)
    else:
        corr.normalize()

    corr = corr * (self.target_distance - distance) * -1.0
    self.body0.apply_correction(
        self.distance_compliance,
        corr,
        self.global_pos0,
        self.body1,
        self.global_pos1,
    )
```

**Summary:** One distance constraint; no angular constraints. Linear damping along the line between the two points.

---

### 4.2 Hinge joint

**What it does:** Allows rotation only around a **common axis** (like a door hinge). Optionally supports swing limits and a target angle with compliance.

**Position constraints:**

- **1 constraint:** Coincidence of the two attachment points (same as zero distance). Implemented by `has_target_distance = True` and `target_distance = 0.0`, so the distance constraint keeps the two points together.

**Orientation constraints:**

- **1 constraint:** Align the **hinge axes** of the two bodies (body0’s X-axis of joint frame = body1’s X-axis). The solver uses the cross product of the two axes as the correction direction (axis alignment).
- **Optional:** Target angle between the two frames (e.g. secondary axis) — enforced via `limit_angle` with `min_angle = max_angle = target_angle` and `target_angle_compliance`.
- **Optional:** Swing limits: angle between the secondary axes clamped to `[swing_min, swing_max]` via `limit_angle`.

**Parameters:**

- `swingMin`, `swingMax` → `swing_min`, `swing_max`
- `targetAngle` (optional) → `has_target_angle`, `target_angle`
- `targetAngleCompliance` → `target_angle_compliance`
- `damping` → `angular_damping_coeff`

**Code (orientation — axis alignment):**

```python
# joint.py — HINGE / SERVO / MOTOR: align axis0 (X-axis of joint frame)
a0 = self.global_rot0.apply_to_vector(axis0)   # axis0 = (1,0,0)
a1 = self.global_rot1.apply_to_vector(axis0)
corr = a0.cross(a1)
# Soft when nearly aligned to avoid snapping
if corr_mag < 0.1 and corr_mag > 0:
    soft_compliance = 0.02
    self.body0.apply_correction(soft_compliance, corr, None, self.body1, None)
else:
    self.body0.apply_correction(hard_compliance, corr, None, self.body1, None)
```

**Code (target angle and swing limits):**

```python
if self.has_target_angle:
    # ... limit_angle(n, a0, a1, target_angle, target_angle, target_angle_compliance)
if self.swing_min > -inf or self.swing_max < inf:
    # ... limit_angle(n, a0, a1, swing_min, swing_max, swing_compliance)
```

**Summary:** 1 position (point coincidence); 1 orientation (axis alignment); optional target angle and swing limits with compliance. Angular damping around the hinge axis.

---

### 4.3 Servo joint

**What it does:** Same as a hinge (one axis of rotation), but with a **target angle** that can be set from outside (e.g. by `control_vector` in `update_control`). Used for controlled rotation (e.g. robot joints).

**Position constraints:** Same as hinge (point coincidence via distance 0).

**Orientation constraints:** Same as hinge: axis alignment, plus **target angle** (with compliance). The target angle is driven externally (`joint.target_angle = self.control_vector.x * math.pi / 4` in the simulator).

**Parameters:**

- `swingMin`, `swingMax` → `swing_min`, `swing_max`
- Target angle is set in code; no rest angle in JSON.

**Summary:** Same constraints as hinge; target angle is controllable. No extra constraints beyond hinge.

---

### 4.4 Motor joint

**What it does:** Hinge that **rotates at a given angular velocity**. Each step, `target_angle` is incremented by `velocity * dt`, so the constraint tries to hold that evolving angle.

**Position constraints:** Same as hinge (point coincidence).

**Orientation constraints:** Same as hinge (axis alignment + target angle). The difference is that **target_angle** is updated every step:

```python
# joint.py — solve_orientation
if self.type == JointType.MOTOR:
    a_angle = max(-1.0, min(self.velocity * dt, 1.0))
    self.target_angle += a_angle
```

**Parameters:**

- `velocity` → `velocity` (and used in `update_control` from `control_vector.y`).

**Summary:** Same as hinge + servo, but target angle is integrated from a constant angular velocity.

---

### 4.5 Ball joint (spherical joint)

**What it does:** Keeps the two attachment **points** coincident but allows **full 3D rotation** (ball-and-socket). Optionally limits **swing** (cone angle) and **twist** (rotation around the axis between the two bodies).

**Position constraints:**

- **1 constraint:** Point coincidence (distance 0), same as hinge.

**Orientation constraints:**

- **Swing limit:** The “cone” of allowed directions of one axis relative to the other is limited by `swing_min` and `swing_max`. Implemented by taking the two X-axes of the joint frames, computing the cross product for the swing plane normal, and calling `limit_angle(..., swing_min, swing_max, ...)`.
- **Twist limit:** Rotation around the (average) axis between the two bodies is limited to `[twist_min, twist_max]`. The code builds the twist axis as \( n = (a_0 + a_1)/\|a_0 + a_1\| \), projects the secondary axes onto the plane perpendicular to \( n \), and limits the angle between them.

**Parameters:**

- `swingMax` → `swing_max` (swing_min = 0)
- `twistMin`, `twistMax` → `twist_min`, `twist_max`
- `damping` → `angular_damping_coeff`

**Code (orientation — swing and twist):**

```python
# joint.py — BALL / PRISMATIC / CYLINDER: swing then twist
a0 = self.global_rot0.apply_to_vector(axis0)
a1 = self.global_rot1.apply_to_vector(axis0)
n = a0.cross(a1)
n.normalize()
self.limit_angle(n, a0, a1, self.swing_min, self.swing_max, swing_compliance)

# Twist: axis n = normalized(a0 + a1), then limit angle between secondary axes
n = a0 + a1
n.normalize()
a0 = self.global_rot0.apply_to_vector(axis1)  # axis1 = (0,1,0)
a1 = self.global_rot1.apply_to_vector(axis1)
a0.add_scaled(n, -n.dot(a0))
a0.normalize()
# ... same for a1
self.limit_angle(n, a0, a1, self.twist_min, self.twist_max, twist_compliance)
```

**Summary:** 1 position (point coincidence); optional swing and twist angle limits with compliance. Angular damping.

---

### 4.6 Prismatic joint

**What it does:** Allows **translation only along one axis** (sliding). Rotation around that axis (twist) can be free or limited; no swing (axes stay aligned).

**Position constraints:**

- **1 constraint:** The **distance along the joint’s X-axis** between the two attachment points is clamped to `[distance_min, distance_max]`. The solver projects the current separation onto the axis, clamps it, and applies a position correction along that axis.

**Orientation constraints:**

- **Swing:** Same as ball — keep the two X-axes aligned (swing_min = swing_max = 0 in practice).
- **Twist:** Optional limits via `twist_min`, `twist_max` (same construction as ball joint twist).

**Parameters:**

- `distanceMin`, `distanceMax` → `distance_min`, `distance_max`
- `twistMin`, `twistMax` → `twist_min`, `twist_max`
- `distanceTarget`, `posTarget`, `compliance` → optional target distance with compliance
- `damping` → `linear_damping_coeff`

**Code (position — prismatic slide):**

```python
# joint.py — PRISMATIC (and CYLINDER)
self.target_distance = max(self.distance_min, min(self.target_distance, self.distance_max))
self.update_global_frames()
corr = self.global_pos1 - self.global_pos0
corr_local = self.global_rot0.conjugate().apply_to_vector(corr)
if self.type == JointType.CYLINDER:
    corr_local.x -= self.target_distance
elif corr_local.x > self.distance_max:
    corr_local.x -= self.distance_max
elif corr_local.x < self.distance_min:
    corr_local.x -= self.distance_min
else:
    corr_local.x = 0.0
corr = self.global_rot0.apply_to_vector(corr_local)
self.body0.apply_correction(hard_compliance, corr, self.global_pos0, self.body1, self.global_pos1)
```

**Summary:** 1 position (slide along one axis, clamped to [min, max]); orientation: axis alignment (swing) + optional twist limits. Linear damping along the slide axis.

---

### 4.7 Cylinder joint

**What it does:** Combines **slide** along one axis (like prismatic) with **free twist** around that axis. So: one translational DOF and one rotational DOF.

**Position constraints:**

- **1 constraint:** Distance along the joint X-axis is clamped to `[distance_min, distance_max]`. Same as prismatic; `target_distance` is updated from current separation for cylinder.

**Orientation constraints:**

- **Swing:** Same as ball/prismatic — align the X-axes (swing limits 0).
- **Twist:** Free (no twist limit in the current cylinder init; twist_min/max can be set but cylinder init uses 0.0 for angular damping).

**Parameters:**

- `distanceMin`, `distanceMax` → `distance_min`, `distance_max`
- `twistMin`, `twistMax` → `twist_min`, `twist_max`
- In code, `has_target_distance = True`, `distance_compliance = 0.0`.

**Summary:** 1 position (slide along axis, clamped); orientation: axis alignment only (twist free). Linear/angular damping.

---

### 4.8 Drag joint (internal)

**What it does:** Used for **mouse picking**: one body is attached to a “ghost” point that follows the mouse ray. Implemented as a **distance joint** with body1 = None; the “world” point is updated each frame to the current mouse ray position.

**Position constraints:** Same as distance joint: maintain distance between body attachment and `local_pos1` (the drag target). When body1 is None, only body0 is corrected.

**Orientation constraints:** None.

**Parameters:** Soft compliance and damping so the dragged body follows the cursor smoothly.

---

## 5. Angle limiting (limit_angle)

Used by hinge, servo, motor, ball, prismatic, and cylinder for swing/twist or target angle. Core idea:

- Compute current angle \( \phi \) between two directions in the plane with normal **n** (`get_angle`).
- Clamp \( \phi \) to `[min_angle, max_angle]` → `phi_clamped`.
- Angle error: `angle_error = phi - phi_clamped`.
- Correction direction: proportional to cross product of the two directions (or **n** if cross is degenerate), scaled by a clamped `angle_error` to avoid huge corrections.
- Apply as orientation correction with optional **soft compliance**.
- If over the limit, apply **angular damping** (velocity-level correction) to reduce jitter.

Relevant snippet:

```python
def limit_angle(self, n, a, b, min_angle, max_angle, compliance):
    phi = self.get_angle(n, a, b)
    if min_angle <= phi <= max_angle:
        return
    phi_clamped = max(min_angle, min(phi, max_angle))
    angle_error = phi - phi_clamped
    soft_compliance = compliance if compliance > 0.0 else 0.01
    ra = a.copy()
    corr = ra.cross(b)
    # ... scale corr by clamped angle_error, then:
    self.body0.apply_correction(soft_compliance, corr, None, self.body1, None)
    # Plus velocity-level damping when over limit
```

---

## 6. Constraint Summary Table

| Joint        | Position constraints                    | Orientation constraints                          |
|-------------|------------------------------------------|--------------------------------------------------|
| **Distance** | 1: distance = rest_distance              | None                                             |
| **Hinge**    | 1: point coincidence (distance 0)        | 1: axis alignment; optional target angle; optional swing limits |
| **Servo**   | 1: point coincidence                     | Same as hinge; target angle set externally        |
| **Motor**   | 1: point coincidence                     | Same as hinge; target angle += velocity*dt        |
| **Ball**    | 1: point coincidence                     | Optional swing limits; optional twist limits     |
| **Prismatic** | 1: slide along axis in [min,max]       | Axis alignment (swing); optional twist limits    |
| **Cylinder** | 1: slide along axis in [min,max]       | Axis alignment (swing); twist free               |
| **Drag**    | 1: distance to cursor (body1 = None)     | None                                             |

---

## 7. Rigid body integration and inertia

**Integration** (`rigid_body.py` — `integrate(dt, gravity)`):

- Apply global damping to `vel` and `omega`.
- Clamp linear and angular speed (max 10 and 15) for stability.
- Semi-implicit Euler:  
  `vel += gravity * dt`  
  `pos += vel * dt`  
  Quaternion update: \( \dot q = \frac{1}{2} \omega q \), then normalize.
- Store previous position/rotation for `update_velocities()`.

**Inverse mass and inertia:**

- Box: \( m = \rho \, V \), \( I_{xx} = \frac{1}{12} m (b^2 + c^2) \) (and cyclic). Stored as `inv_mass` and `inv_inertia` (per-axis inverse inertia).
- Sphere: \( m = \frac{4}{3}\pi r^3 \rho \), \( I = \frac{2}{5} m r^2 \). Same storage.
- `get_inverse_mass(normal, pos)` returns \( 1/m + (r\times n)^T I^{-1} (r\times n) \) when `pos` is given, so corrections at that point produce the correct linear and angular response.

---

## 8. Scene JSON format (reference)

- **Rigid bodies:** `simType`: `RigidBox` or `RigidSphere`; `density` (0 = kinematic). Bounding box/sphere from `vertices`.
- **Joints:** `simType`: `DistanceJoint`, `BallJoint`, `HingeJoint`, `ServoJoint`, `MotorJoint`, `PrismaticJoint`, `CylinderJoint`; `parent1`, `parent2` = body names; `transform.position` and `transform.rotation` = joint frame in world.
- **Joint-specific props:** e.g. `restDistance`, `compliance`, `damping` (Distance); `swingMax`, `twistMin`, `twistMax`, `damping` (Ball); `swingMin`, `swingMax`, `targetAngle`, `targetAngleCompliance`, `damping` (Hinge); etc. See `scene_importer.py` for the exact mapping.

---

## 9. File-by-file roles

| File | Role |
|------|------|
| **main.py** | Entry; creates `JointSimulationApp` and runs it. |
| **physics_sim/config.py** | FPS, WINDOW_WIDTH, WINDOW_HEIGHT. |
| **physics_sim/math_utils.py** | Vector3 (dot, cross, normalize, length, add_scaled); Quaternion (multiply, conjugate, apply_to_vector, set_from_euler, to_matrix). |
| **physics_sim/rigid_body.py** | RigidBody: mass/inertia, integrate, update_velocities, get_inverse_mass, apply_correction, _apply_correction; rendering (box, sphere, visual meshes). |
| **physics_sim/joint.py** | Joint: frames, init_* for each type, get_angle, limit_angle, solve_position, solve_orientation, apply_linear_damping, apply_angular_damping, render. |
| **physics_sim/joint_type.py** | JointType enum (NONE, DISTANCE, HINGE, SERVO, MOTOR, BALL, PRISMATIC, CYLINDER, FIXED). |
| **physics_sim/simulator.py** | RigidBodySimulator: gravity, sub-steps, integrate → solve → update_velocities → damping; drag joint (start_drag, drag, end_drag). |
| **physics_sim/scene_importer.py** | Load JSON: create RigidBox/RigidSphere from vertices; create joints from props; attach Visual meshes to bodies. |
| **physics_sim/app.py** | JointSimulationApp: OpenGL/Pygame init, load_scene_from_file, create_test_scene, handle_events, mouse raycast, update, render, debug overlay. |

This completes the technical documentation of the physics project: principles, constraints per joint type, and the main code paths.
