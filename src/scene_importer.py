import json
import math
from .math_utils import Vector3, Quaternion
from .rigid_body import RigidBody
from .joint import Joint


class SceneImporter:

    def __init__(self, simulator):
        self.simulator = simulator
        self.rigid_bodies = {}

    def load_scene(self, json_data):
        if isinstance(json_data, str):
            data = json.loads(json_data)
        else:
            data = json_data

        if not data or "meshes" not in data or len(data["meshes"]) == 0:
            print("Empty scene data")
            return

        self.simulator.clear()
        self.rigid_bodies = {}

        for mesh in data["meshes"]:
            if self._is_rigid_body(mesh):
                self._create_rigid_body(mesh)

        for mesh in data["meshes"]:
            if self._is_joint(mesh):
                self._create_joint(mesh)

        for mesh in data["meshes"]:
            if self._is_visual(mesh):
                self._create_visual_mesh(mesh)

    def _is_rigid_body(self, mesh):
        sim_type = mesh.get("properties", {}).get("simType", "")
        return sim_type.startswith("Rigid")

    def _is_joint(self, mesh):
        sim_type = mesh.get("properties", {}).get("simType", "")
        return sim_type.endswith("Joint")

    def _is_visual(self, mesh):
        sim_type = mesh.get("properties", {}).get("simType", "")
        return sim_type == "Visual"

    def _create_rigid_body(self, mesh):
        props = mesh.get("properties", {})
        sim_type = props.get("simType", "")
        density = props.get("density", 0.0)

        transform = mesh.get("transform", {})
        pos = Vector3(*transform.get("position", [0, 0, 0]))
        rot_quat = transform.get("rotation", [0, 0, 0, 1])
        quat = Quaternion(rot_quat[0], rot_quat[1], rot_quat[2], rot_quat[3])
        quat.normalize()

        w, x, y, z = rot_quat[3], rot_quat[0], rot_quat[1], rot_quat[2]
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        angles = Vector3(roll, pitch, yaw)

        if sim_type == "RigidBox":
            vertices = mesh.get("vertices", [])
            size = self._calculate_bounding_box(vertices)
            rigid_body = RigidBody("box", size, density, pos, angles)
        elif sim_type == "RigidSphere":
            vertices = mesh.get("vertices", [])
            radius = self._calculate_bounding_sphere(vertices)
            size = Vector3(radius, radius, radius)
            rigid_body = RigidBody("sphere", size, density, pos, angles)
        else:
            return

        rigid_body.rot = quat.copy()
        rigid_body.inv_rot = rigid_body.rot.copy()
        rigid_body.inv_rot.invert()

        self.rigid_bodies[mesh.get("name", "")] = rigid_body
        self.simulator.add_rigid_body(rigid_body)

    def _calculate_bounding_box(self, vertices):
        if len(vertices) < 3:
            return Vector3(1, 1, 1)

        min_vals = [float("inf")] * 3
        max_vals = [float("-inf")] * 3

        for i in range(0, len(vertices), 3):
            for j in range(3):
                if i + j < len(vertices):
                    min_vals[j] = min(min_vals[j], vertices[i + j])
                    max_vals[j] = max(max_vals[j], vertices[i + j])

        return Vector3(
            max_vals[0] - min_vals[0],
            max_vals[1] - min_vals[1],
            max_vals[2] - min_vals[2],
        )

    def _calculate_bounding_sphere(self, vertices):
        if len(vertices) < 3:
            return 0.5

        center = [0, 0, 0]
        count = len(vertices) // 3

        for i in range(0, len(vertices), 3):
            for j in range(3):
                if i + j < len(vertices):
                    center[j] += vertices[i + j]

        for j in range(3):
            center[j] /= count

        max_radius = 0
        for i in range(0, len(vertices), 3):
            dx = vertices[i] - center[0] if i < len(vertices) else 0
            dy = vertices[i + 1] - center[1] if i + 1 < len(vertices) else 0
            dz = vertices[i + 2] - center[2] if i + 2 < len(vertices) else 0
            radius = math.sqrt(dx * dx + dy * dy + dz * dz)
            max_radius = max(max_radius, radius)

        return max_radius

    def _create_joint(self, mesh):
        props = mesh.get("properties", {})
        sim_type = props.get("simType", "")

        body0 = self.rigid_bodies.get(props.get("parent1", ""))
        body1 = self.rigid_bodies.get(props.get("parent2", ""))

        if not body0 or not body1:
            return

        transform = mesh.get("transform", {})
        joint_pos = Vector3(*transform.get("position", [0, 0, 0]))
        rot_quat = transform.get("rotation", [0, 0, 0, 1])
        joint_rot = Quaternion(*rot_quat)

        joint = Joint(body0, body1, joint_pos, joint_rot)

        if sim_type == "BallJoint":
            swing_max = props.get("swingMax", float("inf"))
            twist_min = props.get("twistMin", float("-inf"))
            twist_max = props.get("twistMax", float("inf"))
            damping = props.get("damping", 0.0)
            joint.init_ball_joint(swing_max, twist_min, twist_max, damping)
        elif sim_type == "HingeJoint":
            swing_min = props.get("swingMin", float("-inf"))
            swing_max = props.get("swingMax", float("inf"))
            has_target = props.get("targetAngle") is not None
            target_angle = props.get("targetAngle", 0.0)
            compliance = props.get("targetAngleCompliance", 0.0)
            damping = props.get("damping", 0.0)
            joint.init_hinge_joint(
                swing_min, swing_max, has_target, target_angle, compliance, damping
            )
        elif sim_type == "ServoJoint":
            swing_min = props.get("swingMin", float("-inf"))
            swing_max = props.get("swingMax", float("inf"))
            joint.init_servo(swing_min, swing_max)
        elif sim_type == "MotorJoint":
            velocity = props.get("velocity", 3.0)
            joint.init_motor(velocity)
        elif sim_type == "DistanceJoint":
            rest_distance = props.get("restDistance", 0.0)
            compliance = props.get("compliance", 0.0)
            damping = props.get("damping", 0.0)
            joint.init_distance_joint(rest_distance, compliance, damping)
        elif sim_type == "PrismaticJoint":
            distance_min = props.get("distanceMin", float("-inf"))
            distance_max = props.get("distanceMax", float("inf"))
            twist_min = props.get("twistMin", float("-inf"))
            twist_max = props.get("twistMax", float("inf"))
            has_target = props.get("distanceTarget") is not None
            target_distance = props.get("posTarget", 0.0)
            compliance = props.get("compliance", 0.0)
            damping = props.get("damping", 0.0)
            joint.init_prismatic_joint(
                distance_min,
                distance_max,
                twist_min,
                twist_max,
                has_target,
                target_distance,
                compliance,
                damping,
            )
        elif sim_type == "CylinderJoint":
            distance_min = props.get("distanceMin", float("-inf"))
            distance_max = props.get("distanceMax", float("inf"))
            twist_min = props.get("twistMin", float("-inf"))
            twist_max = props.get("twistMax", float("inf"))
            joint.init_cylinder_joint(
                distance_min, distance_max, twist_min, twist_max, True, 0.0, 0.0, 0.0
            )

        self.simulator.add_joint(joint)

    def _create_visual_mesh(self, mesh):
        props = mesh.get("properties", {})
        parent_name = props.get("parent", "")

        sim_type = props.get("simType", "")
        if sim_type.endswith("Joint"):
            return

        parent_body = self.rigid_bodies.get(parent_name)
        if not parent_body:
            print(f"Warning: Parent body '{parent_name}' not found for visual mesh")
            return

        transform = mesh.get("transform", {})
        visual_pos = Vector3(*transform.get("position", [0, 0, 0]))
        rot_quat = transform.get("rotation", [0, 0, 0, 1])
        visual_rot = Quaternion(rot_quat[0], rot_quat[1], rot_quat[2], rot_quat[3])
        visual_rot.normalize()

        vertices = mesh.get("vertices", [])
        triangles = mesh.get("triangles", [])
        color = props.get("color", [1.0, 1.0, 1.0])

        p_rel_world = visual_pos - parent_body.pos
        p_rel_local = parent_body.inv_rot.apply_to_vector(p_rel_world)
        q_rel = Quaternion().multiply_quaternions(parent_body.inv_rot, visual_rot)

        transformed_vertices = []
        for i in range(0, len(vertices), 3):
            if i + 2 < len(vertices):
                vertex_visual = Vector3(vertices[i], vertices[i + 1], vertices[i + 2])
                vertex_world = visual_rot.apply_to_vector(vertex_visual) + visual_pos
                vertex_parent = vertex_world - parent_body.pos
                vertex_parent = parent_body.inv_rot.apply_to_vector(vertex_parent)
                transformed_vertices.extend(
                    [vertex_parent.x, vertex_parent.y, vertex_parent.z]
                )

        visual_mesh_data = {
            "vertices": transformed_vertices,
            "triangles": triangles,
            "color": color,
        }

        parent_body.visual_meshes.append(visual_mesh_data)
        print(
            f"Added visual mesh to body '{parent_name}' with {len(transformed_vertices)//3} vertices"
        )
