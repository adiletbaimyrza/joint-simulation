"""Rigid body physics object."""

import math
import numpy as np
from OpenGL.GL import *

from .math_utils import Vector3, Quaternion


class RigidBody:
    """Rigid body physics object"""

    def __init__(self, body_type, size, density, pos, angles):
        self.type = body_type
        self.size = size.copy() if isinstance(size, Vector3) else Vector3(*size)
        self.dt = 0.0

        self.pos = pos.copy() if isinstance(pos, Vector3) else Vector3(*pos)
        self.rot = Quaternion()
        self.rot.set_from_euler(angles.x, angles.y, angles.z)
        self.vel = Vector3(0, 0, 0)
        self.omega = Vector3(0, 0, 0)

        self.prev_pos = self.pos.copy()
        self.prev_rot = self.rot.copy()
        self.inv_rot = self.rot.copy()
        self.inv_rot.invert()

        self.inv_mass = 0.0
        self.inv_inertia = Vector3(0, 0, 0)

        self.visual_meshes = []

        if density > 0.0:
            if body_type == "box":
                mass = density * self.size.x * self.size.y * self.size.z
                self.inv_mass = 1.0 / mass if mass > 0 else 0.0
                Ix = 1.0 / 12.0 * mass * (self.size.y**2 + self.size.z**2)
                Iy = 1.0 / 12.0 * mass * (self.size.x**2 + self.size.z**2)
                Iz = 1.0 / 12.0 * mass * (self.size.x**2 + self.size.y**2)
                self.inv_inertia = Vector3(
                    1.0 / Ix if Ix > 0 else 0,
                    1.0 / Iy if Iy > 0 else 0,
                    1.0 / Iz if Iz > 0 else 0,
                )
            elif body_type == "sphere":
                radius = self.size.x
                mass = 4.0 / 3.0 * math.pi * radius**3 * density
                self.inv_mass = 1.0 / mass if mass > 0 else 0.0
                I = 2.0 / 5.0 * mass * radius**2
                self.inv_inertia = Vector3(
                    1.0 / I if I > 0 else 0,
                    1.0 / I if I > 0 else 0,
                    1.0 / I if I > 0 else 0,
                )

        self.show_visuals = True

    def get_velocity_at(self, pos):
        """Get velocity at a specific point"""
        vel = Vector3(0, 0, 0)
        if self.inv_mass > 0.0:
            r = pos - self.pos
            vel = self.vel - r.cross(self.omega)
        return vel

    def local_to_world(self, local_pos):
        """Convert local position to world position"""
        world_pos = self.rot.apply_to_vector(local_pos)
        return world_pos + self.pos

    def world_to_local(self, world_pos):
        """Convert world position to local position"""
        local_pos = world_pos - self.pos
        return self.inv_rot.apply_to_vector(local_pos)

    def integrate(self, dt, gravity):
        """Integrate motion"""
        self.dt = dt

        if self.inv_mass == 0.0:
            return

        # Apply global damping
        damping_factor = 0.98
        self.vel = self.vel * damping_factor
        self.omega = self.omega * damping_factor

        # Clamp velocities to prevent instability
        max_linear_velocity = 10.0
        max_angular_velocity = 15.0  # Reduced from 20.0
        
        vel_mag = self.vel.length()
        if vel_mag > max_linear_velocity:
            self.vel = self.vel * (max_linear_velocity / vel_mag)
        
        omega_mag = self.omega.length()
        if omega_mag > max_angular_velocity:
            self.omega = self.omega * (max_angular_velocity / omega_mag)

        self.prev_pos = self.pos.copy()
        self.vel.add_scaled(gravity, dt)
        self.pos.add_scaled(self.vel, dt)

        self.prev_rot = self.rot.copy()
        dq = Quaternion(
            self.omega.x * 0.5 * dt,
            self.omega.y * 0.5 * dt,
            self.omega.z * 0.5 * dt,
            0.0,
        )
        dq.multiply(self.rot)
        self.rot.x += dq.x
        self.rot.y += dq.y
        self.rot.z += dq.z
        self.rot.w += dq.w
        self.rot.normalize()
        self.inv_rot = self.rot.copy()
        self.inv_rot.invert()

    def update_velocities(self):
        """Update velocities from position changes"""
        if self.inv_mass == 0.0:
            return

        self.vel = (self.pos - self.prev_pos) * (1.0 / self.dt)

        dq = Quaternion().multiply_quaternions(self.rot, self.prev_rot.conjugate())
        self.omega = Vector3(
            dq.x * 2.0 / self.dt, dq.y * 2.0 / self.dt, dq.z * 2.0 / self.dt
        )
        if dq.w < 0.0:
            self.omega = self.omega * -1.0

    def get_inverse_mass(self, normal, pos=None):
        """Get effective inverse mass at a point"""
        if self.inv_mass == 0.0:
            return 0.0

        rn = normal.copy()
        if pos:
            rn = (pos - self.pos).cross(normal)
            rn = self.inv_rot.apply_to_vector(rn)
        else:
            rn = self.inv_rot.apply_to_vector(rn)

        w = (
            rn.x**2 * self.inv_inertia.x
            + rn.y**2 * self.inv_inertia.y
            + rn.z**2 * self.inv_inertia.z
        )

        if pos:
            w += self.inv_mass

        return w

    def apply_correction(
        self,
        compliance,
        corr,
        pos,
        other_body=None,
        other_pos=None,
        velocity_level=False,
    ):
        """Apply position/velocity correction"""
        if corr.length_sq() == 0.0:
            return 0.0

        C = corr.length()
        normal = corr.normalized()

        w = self.get_inverse_mass(normal, pos)
        if other_body:
            w += other_body.get_inverse_mass(normal, other_pos)

        if w == 0.0:
            return 0.0

        lambda_val = -C / w

        if not velocity_level:
            alpha = compliance / self.dt / self.dt
            lambda_val = -C / (w + alpha)

        correction = normal * (-lambda_val)
        self._apply_correction(correction, pos, velocity_level)

        if other_body:
            correction_other = normal * lambda_val
            other_body._apply_correction(correction_other, other_pos, velocity_level)

        return lambda_val / self.dt / self.dt

    def _apply_correction(self, corr, pos, velocity_level):
        """Internal correction application"""
        if self.inv_mass == 0.0:
            return

        if pos:
            if velocity_level:
                self.vel.add_scaled(corr, self.inv_mass)
            else:
                self.pos.add_scaled(corr, self.inv_mass)

        d_omega = corr.copy()
        if pos:
            d_omega = (pos - self.pos).cross(corr)

        d_omega = self.inv_rot.apply_to_vector(d_omega)
        d_omega = Vector3(
            d_omega.x * self.inv_inertia.x,
            d_omega.y * self.inv_inertia.y,
            d_omega.z * self.inv_inertia.z,
        )
        d_omega = self.rot.apply_to_vector(d_omega)

        if velocity_level:
            self.omega = self.omega + d_omega
        else:
            d_omega = d_omega * 0.5
            dq = Quaternion(d_omega.x, d_omega.y, d_omega.z, 0.0)
            dq.multiply(self.rot)
            self.rot.x += 0.5 * dq.x
            self.rot.y += 0.5 * dq.y
            self.rot.z += 0.5 * dq.z
            self.rot.w += 0.5 * dq.w
            self.rot.normalize()
            self.inv_rot = self.rot.copy()
            self.inv_rot.invert()

    def render(self):
        """Render the rigid body"""
        # Don't render zero-density (kinematic/static) bodies without visuals
        # as they're just anchors for joints
        if self.inv_mass == 0.0 and not self.visual_meshes:
            return
        
        glPushMatrix()
        glTranslatef(self.pos.x, self.pos.y, self.pos.z)

        matrix = self.rot.to_matrix()
        glMultMatrixf(matrix.T)

        if self.visual_meshes and self.show_visuals:
            for visual_mesh in self.visual_meshes:
                self._render_visual_mesh(visual_mesh)
        else:
            if self.type == "box":
                self._render_box()
            elif self.type == "sphere":
                self._render_sphere()

        glPopMatrix()

    def _render_visual_mesh(self, visual_mesh):
        """Render a visual mesh with vertices and triangles"""
        vertices = visual_mesh["vertices"]
        triangles = visual_mesh.get("triangles", [])
        color = visual_mesh.get("color", [1.0, 1.0, 1.0])

        glColor3f(color[0], color[1], color[2])

        if triangles:
            glBegin(GL_TRIANGLES)
            for i in range(0, len(triangles), 3):
                if i + 2 < len(triangles):
                    idx0 = triangles[i] * 3
                    idx1 = triangles[i + 1] * 3
                    idx2 = triangles[i + 2] * 3

                    if (
                        idx0 + 2 < len(vertices)
                        and idx1 + 2 < len(vertices)
                        and idx2 + 2 < len(vertices)
                    ):
                        glVertex3f(
                            vertices[idx0], vertices[idx0 + 1], vertices[idx0 + 2]
                        )
                        glVertex3f(
                            vertices[idx1], vertices[idx1 + 1], vertices[idx1 + 2]
                        )
                        glVertex3f(
                            vertices[idx2], vertices[idx2 + 1], vertices[idx2 + 2]
                        )
            glEnd()
        else:
            glBegin(GL_TRIANGLES)
            for i in range(0, len(vertices) - 6, 9):
                if i + 8 < len(vertices):
                    glVertex3f(vertices[i], vertices[i + 1], vertices[i + 2])
                    glVertex3f(vertices[i + 3], vertices[i + 4], vertices[i + 5])
                    glVertex3f(vertices[i + 6], vertices[i + 7], vertices[i + 8])
            glEnd()

    def _render_box(self):
        """Render a box"""
        s = self.size
        glBegin(GL_QUADS)
        glColor3f(1.0, 1.0, 1.0)
        glVertex3f(-s.x / 2, -s.y / 2, s.z / 2)
        glVertex3f(s.x / 2, -s.y / 2, s.z / 2)
        glVertex3f(s.x / 2, s.y / 2, s.z / 2)
        glVertex3f(-s.x / 2, s.y / 2, s.z / 2)
        glVertex3f(-s.x / 2, -s.y / 2, -s.z / 2)
        glVertex3f(-s.x / 2, s.y / 2, -s.z / 2)
        glVertex3f(s.x / 2, s.y / 2, -s.z / 2)
        glVertex3f(s.x / 2, -s.y / 2, -s.z / 2)
        glVertex3f(-s.x / 2, s.y / 2, -s.z / 2)
        glVertex3f(-s.x / 2, s.y / 2, s.z / 2)
        glVertex3f(s.x / 2, s.y / 2, s.z / 2)
        glVertex3f(s.x / 2, s.y / 2, -s.z / 2)
        glVertex3f(-s.x / 2, -s.y / 2, -s.z / 2)
        glVertex3f(s.x / 2, -s.y / 2, -s.z / 2)
        glVertex3f(s.x / 2, -s.y / 2, s.z / 2)
        glVertex3f(-s.x / 2, -s.y / 2, s.z / 2)
        glVertex3f(s.x / 2, -s.y / 2, -s.z / 2)
        glVertex3f(s.x / 2, s.y / 2, -s.z / 2)
        glVertex3f(s.x / 2, s.y / 2, s.z / 2)
        glVertex3f(s.x / 2, -s.y / 2, s.z / 2)
        glVertex3f(-s.x / 2, -s.y / 2, -s.z / 2)
        glVertex3f(-s.x / 2, -s.y / 2, s.z / 2)
        glVertex3f(-s.x / 2, s.y / 2, s.z / 2)
        glVertex3f(-s.x / 2, s.y / 2, -s.z / 2)
        glEnd()

    def _render_sphere(self):
        """Render a sphere"""
        radius = self.size.x
        slices = 20
        stacks = 20

        for i in range(stacks):
            lat1 = math.pi * (-0.5 + i / stacks)
            lat2 = math.pi * (-0.5 + (i + 1) / stacks)

            glBegin(GL_QUAD_STRIP)
            for j in range(slices + 1):
                lng = 2 * math.pi * j / slices

                x1 = radius * math.cos(lat1) * math.cos(lng)
                y1 = radius * math.sin(lat1)
                z1 = radius * math.cos(lat1) * math.sin(lng)

                x2 = radius * math.cos(lat2) * math.cos(lng)
                y2 = radius * math.sin(lat2)
                z2 = radius * math.cos(lat2) * math.sin(lng)

                glColor3f(
                    1.0 if i < stacks / 2 else 0.0, 0.0, 0.0 if i < stacks / 2 else 1.0
                )
                glVertex3f(x1, y1, z1)
                glVertex3f(x2, y2, z2)
            glEnd()
