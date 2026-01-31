import math
from .math_utils import Vector3
from .joint import Joint
from .joint_type import JointType


class RigidBodySimulator:

    def __init__(self, gravity):
        self.gravity = gravity.copy() * 0.3
        self.dt = 1.0 / 30.0
        self.num_sub_steps = 10
        self.num_iterations = 1
        self.rigid_bodies = []
        self.joints = []
        self.drag_joint = None
        self.simulation_view = True
        self.control_vector = Vector3(0, 0, 0)
        self.is_dragging = False

        self.add_drag_joint()

    def add_drag_joint(self):
        drag_compliance = 0.05
        drag_damping = 0.5
        self.drag_joint = Joint(None, None, Vector3(0, 0, 0))
        self.drag_joint.init_distance_joint(0.0, drag_compliance, drag_damping)
        self.drag_joint.disabled = True

    def clear(self):
        self.rigid_bodies = []
        self.joints = []
        self.add_drag_joint()

    def add_rigid_body(self, rigid_body):
        self.rigid_bodies.append(rigid_body)

    def add_joint(self, joint):
        self.joints.append(joint)

    def update_control(self):
        for joint in self.joints:
            if joint.type == JointType.MOTOR:
                joint.velocity = self.control_vector.y * 5.0
            elif joint.type == JointType.SERVO:
                joint.target_angle = self.control_vector.x * math.pi / 4
            elif joint.type == JointType.CYLINDER:
                joint.target_distance = -self.control_vector.y * 0.1

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

    def start_drag(self, body, pos):
        self.drag_joint.body0 = body
        self.drag_joint.set_frames(pos)
        self.drag_joint.disabled = False

        if body.inv_mass > 0.0:
            body.omega = body.omega * 0.1

    def drag(self, pos):
        self.drag_joint.local_pos1 = pos

        if self.drag_joint.body0 and self.drag_joint.body0.inv_mass > 0.0:
            max_angular_velocity_drag = 5.0
            omega_mag = self.drag_joint.body0.omega.length()
            if omega_mag > max_angular_velocity_drag:
                self.drag_joint.body0.omega = self.drag_joint.body0.omega * (max_angular_velocity_drag / omega_mag)

            self.drag_joint.body0.omega = self.drag_joint.body0.omega * 0.7

    def end_drag(self):
        self.drag_joint.disabled = True
