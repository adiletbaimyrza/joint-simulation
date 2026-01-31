# 3D Rigid Body & Joint Simulation

A real-time 3D physics simulation with rigid bodies and multiple joint types. Built with Python, Pygame, and OpenGL. Supports scene loading from JSON and mouse-drag interaction.

## Features

- **Rigid bodies**: Box and sphere shapes with mass, inertia, gravity, and damping
- **Joint types**: Distance, Hinge, Servo, Motor, Ball, Prismatic, Cylinder
- **Position-based dynamics**: XPBD-style constraint solving with compliance and sub-stepping
- **Scene import**: Load scenes from JSON (rigid bodies, joints, visual meshes)
- **Interaction**: Click and drag bodies; pause/resume (Space), reload scene (R)
- **Debug overlay**: FPS, camera, mouse, bodies, and joints info on screen

## Requirements

- Python 3.8+
- pygame ≥ 2.5.0
- PyOpenGL ≥ 3.1.6
- PyOpenGL-accelerate ≥ 3.1.6
- numpy ≥ 1.24.0

## Installation

```bash
pip install -r requirements.txt
```

## Run

```bash
python main.py
```

The app loads `basicJoints.json` from the project root. If the file is missing, a simple test scene (ground + box + distance joint) is created.

## Controls

| Key / Action      | Effect                    |
|-------------------|---------------------------|
| **Space**         | Pause / resume simulation  |
| **R**             | Reload scene from JSON    |
| **Left click + drag** | Grab and move a body  |
| **Close window**  | Quit                      |

## Project Structure

```
physics-project/
├── main.py                 # Entry point
├── basicJoints.json        # Scene definition (optional)
├── requirements.txt
├── README.md
├── DOCUMENTATION.md        # Full technical documentation
└── physics_sim/
    ├── __init__.py         # Package exports
    ├── config.py           # FPS, window size
    ├── math_utils.py       # Vector3, Quaternion
    ├── rigid_body.py       # Rigid body dynamics & rendering
    ├── joint.py            # Joint constraints & types
    ├── joint_type.py       # JointType enum
    ├── simulator.py        # Simulation loop, drag joint
    ├── scene_importer.py  # JSON → rigid bodies & joints
    └── app.py              # Pygame/OpenGL app, input, debug UI
```

## Scene Format (JSON)

Scenes are defined in a JSON file with a `meshes` array. Each mesh has:

- **name**: Unique identifier
- **vertices**, **triangles**, **normals**: Geometry (used for bounding box/sphere or visuals)
- **transform**: `position` [x,y,z], `rotation` [x,y,z,w] quaternion
- **properties.simType**: One of:
  - `RigidBox`, `RigidSphere` — rigid bodies (use `density`, `parent` for visuals)
  - `DistanceJoint`, `BallJoint`, `HingeJoint`, `ServoJoint`, `MotorJoint`, `PrismaticJoint`, `CylinderJoint` — joints (use `parent1`, `parent2`, and type-specific props)
  - `Visual` — visual mesh attached to a rigid body (`parent`, `color`)

See `basicJoints.json` for examples and **DOCUMENTATION.md** for full specification and constraint details.

## Documentation

For a detailed explanation of the physics model, each joint type, constraints, and code excerpts, see **[DOCUMENTATION.md](DOCUMENTATION.md)**.
