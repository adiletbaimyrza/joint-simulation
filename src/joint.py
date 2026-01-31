import math
from OpenGL.GL import *

from .math_utils import Vector3, Quaternion
from .joint_type import JointType


class Joint:

    def __init__(self, body0, body1, global_frame_pos, global_frame_rot=None):
        self.type = JointType.NONE
        self.body0 = body0
        self.body1 = body1
        self.disabled = False

        if global_frame_rot is None:
            global_frame_rot = Quaternion(0, 0, 0, 1)

        self.has_target_distance = False
        self.target_distance = 0.0
        self.distance_compliance = 0.0
        self.distance_min = float("-inf")
        self.distance_max = float("inf")
        self.linear_damping_coeff = 0.0

        self.swing_min = float("-inf")
        self.swing_max = float("inf")
        self.twist_min = float("-inf")
        self.twist_max = float("inf")
        self.target_angle = 0.0
        self.has_target_angle = False
        self.target_angle_compliance = 0.0
        self.angular_damping_coeff = 0.0

        self.velocity = 0.0

        self.global_pos0 = global_frame_pos.copy()
        self.global_rot0 = global_frame_rot.copy()
        self.global_pos1 = global_frame_pos.copy()
        self.global_rot1 = global_frame_rot.copy()

        self.local_pos0 = global_frame_pos.copy()
        self.local_rot0 = global_frame_rot.copy()
        self.local_pos1 = global_frame_pos.copy()
        self.local_rot1 = global_frame_rot.copy()

        self.set_frames(global_frame_pos, global_frame_rot)

    def set_frames(self, global_frame_pos, global_frame_rot=None):
        if global_frame_rot is None:
            global_frame_rot = Quaternion(0, 0, 0, 1)

        if self.body0:
            self.local_pos0 = self.body0.world_to_local(global_frame_pos)
            self.local_rot0 = Quaternion().multiply_quaternions(
                self.body0.inv_rot, global_frame_rot
            )
        else:
            self.local_pos0 = global_frame_pos.copy()
            self.local_rot0 = global_frame_rot.copy()

        if self.body1:
            self.local_pos1 = self.body1.world_to_local(global_frame_pos)
            self.local_rot1 = Quaternion().multiply_quaternions(
                self.body1.inv_rot, global_frame_rot
            )
        else:
            self.local_pos1 = global_frame_pos.copy()
            self.local_rot1 = global_frame_rot.copy()

    def update_global_frames(self):
        if self.body0:
            self.global_pos0 = self.body0.local_to_world(self.local_pos0)
            self.global_rot0 = Quaternion().multiply_quaternions(
                self.body0.rot, self.local_rot0
            )
            self.global_rot0.normalize()
        else:
            self.global_pos0 = self.local_pos0.copy()
            self.global_rot0 = self.local_rot0.copy()

        if self.body1:
            self.global_pos1 = self.body1.local_to_world(self.local_pos1)
            self.global_rot1 = Quaternion().multiply_quaternions(
                self.body1.rot, self.local_rot1
            )
            self.global_rot1.normalize()
        else:
            self.global_pos1 = self.local_pos1.copy()
            self.global_rot1 = self.local_rot1.copy()

    def init_hinge_joint(
        self, swing_min, swing_max, has_target_angle, target_angle, compliance, damping
    ):
        self.type = JointType.HINGE
        self.has_target_distance = True
        self.target_distance = 0.0
        self.swing_min = swing_min
        self.swing_max = swing_max
        self.has_target_angle = has_target_angle
        self.target_angle = target_angle
        self.target_angle_compliance = compliance
        self.angular_damping_coeff = damping

    def init_servo(self, swing_min, swing_max):
        self.type = JointType.SERVO
        self.has_target_distance = True
        self.target_distance = 0.0
        self.swing_min = swing_min
        self.swing_max = swing_max
        self.has_target_angle = True
        self.target_angle = 0.0
        self.target_angle_compliance = 0.0

    def init_motor(self, velocity):
        self.type = JointType.MOTOR
        self.has_target_distance = True
        self.target_distance = 0.0
        self.velocity = velocity
        self.has_target_angle = True
        self.target_angle = 0.0
        self.target_angle_compliance = 0.0

    def init_ball_joint(self, swing_max, twist_min, twist_max, damping):
        self.type = JointType.BALL
        self.has_target_distance = True
        self.target_distance = 0.0
        self.swing_min = 0.0
        self.swing_max = swing_max
        self.twist_min = twist_min
        self.twist_max = twist_max
        self.angular_damping_coeff = damping

    def init_distance_joint(self, rest_distance, compliance, damping):
        self.type = JointType.DISTANCE
        self.has_target_distance = True
        self.target_distance = rest_distance
        self.distance_compliance = compliance
        self.linear_damping_coeff = damping

    def init_prismatic_joint(
        self,
        distance_min,
        distance_max,
        twist_min,
        twist_max,
        has_target,
        target_distance,
        target_compliance,
        damping,
    ):
        self.type = JointType.PRISMATIC
        self.distance_min = distance_min
        self.distance_max = distance_max
        self.swing_min = 0.0
        self.swing_max = 0.0
        self.twist_min = twist_min
        self.twist_max = twist_max
        self.has_target_distance = has_target
        self.target_distance = target_distance
        self.distance_compliance = target_compliance
        self.linear_damping_coeff = damping

    def init_cylinder_joint(
        self,
        distance_min,
        distance_max,
        twist_min,
        twist_max,
        has_target_distance,
        rest_distance,
        compliance,
        damping,
    ):
        self.type = JointType.CYLINDER
        self.distance_min = distance_min
        self.distance_max = distance_max
        self.swing_min = 0.0
        self.swing_max = 0.0
        self.twist_min = twist_min
        self.twist_max = twist_max
        self.has_target_distance = True
        self.distance_compliance = 0.0

    def get_angle(self, n, a, b):
        c = a.cross(b)
        phi = math.asin(c.dot(n))
        if a.dot(b) < 0.0:
            phi = math.pi - phi
        if phi > math.pi:
            phi -= 2.0 * math.pi
        if phi < -math.pi:
            phi += 2.0 * math.pi
        return phi

    def limit_angle(self, n, a, b, min_angle, max_angle, compliance):
        phi = self.get_angle(n, a, b)
        if min_angle <= phi <= max_angle:
            return

        phi_clamped = max(min_angle, min(phi, max_angle))
        angle_error = phi - phi_clamped

        soft_compliance = compliance if compliance > 0.0 else 0.01

        ra = a.copy()
        corr = ra.cross(b)

        corr_mag = corr.length()
        if corr_mag > 1e-6:
            max_correction = 0.05
            scaled_error = max(-max_correction, min(angle_error, max_correction))
            corr = corr * (scaled_error / corr_mag)
        else:
            max_correction = 0.05
            scaled_error = max(-max_correction, min(angle_error, max_correction))
            corr = n * scaled_error

        self.body0.apply_correction(soft_compliance, corr, None, self.body1, None)

        if abs(angle_error) > 0.001:
            d_omega = self.body0.omega.copy()
            if self.body1:
                d_omega = d_omega - self.body1.omega

            omega_along_n = n * d_omega.dot(n)

            if (angle_error > 0 and omega_along_n.dot(n) > 0) or (
                angle_error < 0 and omega_along_n.dot(n) < 0
            ):
                damping_factor = 0.3
                omega_damped = omega_along_n * (-damping_factor)
                self.body0.apply_correction(
                    0.0, omega_damped, None, self.body1, None, True
                )

    def solve_position(self, dt):
        if self.disabled or self.type == JointType.NONE:
            return

        hard_compliance = 0.0

        if self.type == JointType.PRISMATIC or self.type == JointType.CYLINDER:
            self.target_distance = max(
                self.distance_min, min(self.target_distance, self.distance_max)
            )
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
            self.body0.apply_correction(
                hard_compliance, corr, self.global_pos0, self.body1, self.global_pos1
            )

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

    def solve_orientation(self, dt):
        if (
            self.disabled
            or self.type == JointType.NONE
            or self.type == JointType.DISTANCE
        ):
            return

        if self.type == JointType.MOTOR:
            a_angle = max(-1.0, min(self.velocity * dt, 1.0))
            self.target_angle += a_angle

        hard_compliance = 0.0
        axis0 = Vector3(1.0, 0.0, 0.0)
        axis1 = Vector3(0.0, 1.0, 0.0)

        if (
            self.type == JointType.HINGE
            or self.type == JointType.SERVO
            or self.type == JointType.MOTOR
        ):
            self.update_global_frames()
            a0 = self.global_rot0.apply_to_vector(axis0)
            a1 = self.global_rot1.apply_to_vector(axis0)
            corr = a0.cross(a1)

            corr_mag = corr.length()
            if corr_mag < 0.1 and corr_mag > 0:
                soft_compliance = 0.02
                self.body0.apply_correction(
                    soft_compliance, corr, None, self.body1, None
                )
            else:
                self.body0.apply_correction(
                    hard_compliance, corr, None, self.body1, None
                )

            if self.has_target_angle:
                self.update_global_frames()
                n = self.global_rot0.apply_to_vector(axis0)
                a0 = self.global_rot0.apply_to_vector(axis1)
                a1 = self.global_rot1.apply_to_vector(axis1)
                self.limit_angle(
                    n,
                    a0,
                    a1,
                    self.target_angle,
                    self.target_angle,
                    self.target_angle_compliance,
                )

            if self.swing_min > float("-inf") or self.swing_max < float("inf"):
                self.update_global_frames()
                n = self.global_rot0.apply_to_vector(axis0)
                a0 = self.global_rot0.apply_to_vector(axis1)
                a1 = self.global_rot1.apply_to_vector(axis1)
                swing_compliance = 0.01 if self.angular_damping_coeff > 0 else 0.02
                self.limit_angle(
                    n, a0, a1, self.swing_min, self.swing_max, swing_compliance
                )

        elif (
            self.type == JointType.BALL
            or self.type == JointType.PRISMATIC
            or self.type == JointType.CYLINDER
        ):
            self.update_global_frames()
            a0 = self.global_rot0.apply_to_vector(axis0)
            a1 = self.global_rot1.apply_to_vector(axis0)
            n = a0.cross(a1)
            n.normalize()
            swing_compliance = 0.01 if self.angular_damping_coeff > 0 else 0.02
            self.limit_angle(
                n, a0, a1, self.swing_min, self.swing_max, swing_compliance
            )

            self.update_global_frames()
            a0 = self.global_rot0.apply_to_vector(axis0)
            a1 = self.global_rot1.apply_to_vector(axis0)
            n = a0 + a1
            n.normalize()
            a0 = self.global_rot0.apply_to_vector(axis1)
            a1 = self.global_rot1.apply_to_vector(axis1)
            a0.add_scaled(n, -n.dot(a0))
            a0.normalize()
            a1.add_scaled(n, -n.dot(a1))
            a1.normalize()
            twist_compliance = 0.01 if self.angular_damping_coeff > 0 else 0.02
            self.limit_angle(
                n, a0, a1, self.twist_min, self.twist_max, twist_compliance
            )

    def solve(self, dt):
        self.solve_position(dt)
        self.solve_orientation(dt)

    def apply_linear_damping(self, dt):
        self.update_global_frames()
        d_vel = self.body0.get_velocity_at(self.global_pos0)
        if self.body1:
            d_vel = d_vel - self.body1.get_velocity_at(self.global_pos1)

        n = (self.global_pos1 - self.global_pos0).normalized()
        n = n * (-d_vel.dot(n))
        n = n * min(self.linear_damping_coeff * dt, 1.0)
        self.body0.apply_correction(
            0.0, n, self.global_pos0, self.body1, self.global_pos1, True
        )

    def apply_angular_damping(self, dt, coeff=None):
        if coeff is None:
            coeff = self.angular_damping_coeff

        self.update_global_frames()
        d_omega = self.body0.omega.copy()
        if self.body1:
            d_omega = d_omega - self.body1.omega

        if self.type == JointType.HINGE:
            n = self.global_rot0.apply_to_vector(Vector3(1.0, 0.0, 0.0))
            n = n * d_omega.dot(n)
            d_omega = n

        if self.type == JointType.CYLINDER or self.type == JointType.PRISMATIC:
            d_omega = d_omega * -1.0
        else:
            d_omega = d_omega * (-min(coeff * dt, 1.0))

        self.body0.apply_correction(0.0, d_omega, None, self.body1, None, True)

    def render(self):
        if self.disabled:
            return

        self.update_global_frames()

        glLineWidth(2.0)
        glBegin(GL_LINES)
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(self.global_pos0.x, self.global_pos0.y, self.global_pos0.z)
        glVertex3f(self.global_pos1.x, self.global_pos1.y, self.global_pos1.z)
        glEnd()

        glPointSize(5.0)
        glBegin(GL_POINTS)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(self.global_pos0.x, self.global_pos0.y, self.global_pos0.z)
        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(self.global_pos1.x, self.global_pos1.y, self.global_pos1.z)
        glEnd()
