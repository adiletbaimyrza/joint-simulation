#!/usr/bin/env python3
"""
Joint Simulation - Python 3D Physics Simulation
Replicated from 25-joints.html using PyOpenGL and pygame
"""

import pygame
import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from ctypes import c_double, c_float, c_int
import json
import math
from enum import Enum
from typing import Optional, List, Dict, Tuple
import os

# Initialize Pygame
pygame.init()

# Constants
FPS = 60
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 800


class Vector3:
    """3D Vector class similar to THREE.Vector3"""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
    
    def __add__(self, other):
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)
    
    def __sub__(self, other):
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)
    
    def __mul__(self, scalar):
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)
    
    def __rmul__(self, scalar):
        return self.__mul__(scalar)
    
    def __truediv__(self, scalar):
        return Vector3(self.x / scalar, self.y / scalar, self.z / scalar)
    
    def copy(self):
        return Vector3(self.x, self.y, self.z)
    
    def length(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)
    
    def length_sq(self):
        return self.x**2 + self.y**2 + self.z**2
    
    def normalize(self):
        l = self.length()
        if l > 0:
            self.x /= l
            self.y /= l
            self.z /= l
        return self
    
    def normalized(self):
        result = self.copy()
        result.normalize()
        return result
    
    def dot(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z
    
    def cross(self, other):
        return Vector3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x
        )
    
    def add_scaled(self, other, scale):
        self.x += other.x * scale
        self.y += other.y * scale
        self.z += other.z * scale
    
    def to_array(self):
        return [self.x, self.y, self.z]
    
    def to_numpy(self):
        return np.array([self.x, self.y, self.z])


class Quaternion:
    """Quaternion class for rotations"""
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.w = float(w)
    
    def copy(self):
        return Quaternion(self.x, self.y, self.z, self.w)
    
    def normalize(self):
        l = math.sqrt(self.x**2 + self.y**2 + self.z**2 + self.w**2)
        if l > 0:
            self.x /= l
            self.y /= l
            self.z /= l
            self.w /= l
        return self
    
    def conjugate(self):
        return Quaternion(-self.x, -self.y, -self.z, self.w)
    
    def invert(self):
        self.x = -self.x
        self.y = -self.y
        self.z = -self.z
        return self
    
    def multiply(self, other):
        """Multiply this quaternion by another"""
        w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
        x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
        y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x
        z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
        self.w = w
        self.x = x
        self.y = y
        self.z = z
        return self
    
    def multiply_quaternions(self, q1, q2):
        """Set this quaternion to q1 * q2"""
        self.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        self.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
        self.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
        self.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
        return self
    
    def set_from_euler(self, x, y, z):
        """Set quaternion from Euler angles"""
        c1 = math.cos(x / 2)
        c2 = math.cos(y / 2)
        c3 = math.cos(z / 2)
        s1 = math.sin(x / 2)
        s2 = math.sin(y / 2)
        s3 = math.sin(z / 2)
        
        self.w = c1 * c2 * c3 - s1 * s2 * s3
        self.x = s1 * c2 * c3 + c1 * s2 * s3
        self.y = c1 * s2 * c3 - s1 * c2 * s3
        self.z = c1 * c2 * s3 + s1 * s2 * c3
        return self
    
    def apply_to_vector(self, v):
        """Apply quaternion rotation to a vector"""
        # Convert vector to quaternion
        qv = Quaternion(v.x, v.y, v.z, 0)
        # q * v * q^-1
        q_inv = self.conjugate()
        temp = Quaternion().multiply_quaternions(self, qv)
        result = Quaternion().multiply_quaternions(temp, q_inv)
        return Vector3(result.x, result.y, result.z)
    
    def premultiply(self, other):
        """Premultiply: this = other * this"""
        result = Quaternion().multiply_quaternions(other, self)
        self.x = result.x
        self.y = result.y
        self.z = result.z
        self.w = result.w
        return self
    
    def to_matrix(self):
        """Convert quaternion to rotation matrix"""
        x, y, z, w = self.x, self.y, self.z, self.w
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w), 0],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w), 0],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y), 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)


class JointType(Enum):
    NONE = 'none'
    DISTANCE = 'distance'
    HINGE = 'hinge'
    SERVO = 'servo'
    MOTOR = 'motor'
    BALL = 'ball'
    PRISMATIC = 'prismatic'
    CYLINDER = 'cylinder'
    FIXED = 'fixed'


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
        
        # Visual meshes for detailed rendering
        self.visual_meshes = []
        
        # Calculate mass and inertia
        if density > 0.0:
            if body_type == "box":
                mass = density * self.size.x * self.size.y * self.size.z
                self.inv_mass = 1.0 / mass if mass > 0 else 0.0
                Ix = 1.0 / 12.0 * mass * (self.size.y**2 + self.size.z**2)
                Iy = 1.0 / 12.0 * mass * (self.size.x**2 + self.size.z**2)
                Iz = 1.0 / 12.0 * mass * (self.size.x**2 + self.size.y**2)
                self.inv_inertia = Vector3(1.0/Ix if Ix > 0 else 0, 1.0/Iy if Iy > 0 else 0, 1.0/Iz if Iz > 0 else 0)
            elif body_type == "sphere":
                radius = self.size.x
                mass = 4.0 / 3.0 * math.pi * radius**3 * density
                self.inv_mass = 1.0 / mass if mass > 0 else 0.0
                I = 2.0 / 5.0 * mass * radius**2
                self.inv_inertia = Vector3(1.0/I if I > 0 else 0, 1.0/I if I > 0 else 0, 1.0/I if I > 0 else 0)
        
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
        
        # Linear motion
        self.prev_pos = self.pos.copy()
        self.vel.add_scaled(gravity, dt)
        self.pos.add_scaled(self.vel, dt)
        
        # Angular motion
        self.prev_rot = self.rot.copy()
        # Simplified quaternion integration
        dq = Quaternion(
            self.omega.x * 0.5 * dt,
            self.omega.y * 0.5 * dt,
            self.omega.z * 0.5 * dt,
            0.0
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
        
        # Linear motion
        self.vel = (self.pos - self.prev_pos) * (1.0 / self.dt)
        
        # Angular motion (simplified)
        dq = Quaternion().multiply_quaternions(self.rot, self.prev_rot.conjugate())
        self.omega = Vector3(
            dq.x * 2.0 / self.dt,
            dq.y * 2.0 / self.dt,
            dq.z * 2.0 / self.dt
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
        
        w = (rn.x**2 * self.inv_inertia.x + 
             rn.y**2 * self.inv_inertia.y + 
             rn.z**2 * self.inv_inertia.z)
        
        if pos:
            w += self.inv_mass
        
        return w
    
    def apply_correction(self, compliance, corr, pos, other_body=None, other_pos=None, velocity_level=False):
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
            # XPBD
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
        
        # Linear correction
        if pos:
            if velocity_level:
                self.vel.add_scaled(corr, self.inv_mass)
            else:
                self.pos.add_scaled(corr, self.inv_mass)
        
        # Angular correction
        d_omega = corr.copy()
        if pos:
            d_omega = (pos - self.pos).cross(corr)
        
        d_omega = self.inv_rot.apply_to_vector(d_omega)
        d_omega = Vector3(
            d_omega.x * self.inv_inertia.x,
            d_omega.y * self.inv_inertia.y,
            d_omega.z * self.inv_inertia.z
        )
        d_omega = self.rot.apply_to_vector(d_omega)
        
        if velocity_level:
            self.omega = self.omega + d_omega
        else:
            # Stabilize rotation
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
        glPushMatrix()
        glTranslatef(self.pos.x, self.pos.y, self.pos.z)
        
        # Apply rotation
        matrix = self.rot.to_matrix()
        glMultMatrixf(matrix.T)
        
        # Render visual meshes if available, otherwise render simple shape
        if self.visual_meshes and self.show_visuals:
            for visual_mesh in self.visual_meshes:
                self._render_visual_mesh(visual_mesh)
        else:
            # Fallback to simple rendering
            if self.type == "box":
                self._render_box()
            elif self.type == "sphere":
                self._render_sphere()
        
        glPopMatrix()
    
    def _render_visual_mesh(self, visual_mesh):
        """Render a visual mesh with vertices and triangles"""
        vertices = visual_mesh['vertices']
        triangles = visual_mesh.get('triangles', [])
        color = visual_mesh.get('color', [1.0, 1.0, 1.0])
        
        glColor3f(color[0], color[1], color[2])
        
        if triangles:
            # Render using triangle indices
            glBegin(GL_TRIANGLES)
            for i in range(0, len(triangles), 3):
                if i + 2 < len(triangles):
                    idx0 = triangles[i] * 3
                    idx1 = triangles[i + 1] * 3
                    idx2 = triangles[i + 2] * 3
                    
                    if idx0 + 2 < len(vertices) and idx1 + 2 < len(vertices) and idx2 + 2 < len(vertices):
                        glVertex3f(vertices[idx0], vertices[idx0 + 1], vertices[idx0 + 2])
                        glVertex3f(vertices[idx1], vertices[idx1 + 1], vertices[idx1 + 2])
                        glVertex3f(vertices[idx2], vertices[idx2 + 1], vertices[idx2 + 2])
            glEnd()
        else:
            # Render as triangle strip if no indices
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
        # Front face
        glColor3f(1.0, 1.0, 1.0)
        glVertex3f(-s.x/2, -s.y/2, s.z/2)
        glVertex3f(s.x/2, -s.y/2, s.z/2)
        glVertex3f(s.x/2, s.y/2, s.z/2)
        glVertex3f(-s.x/2, s.y/2, s.z/2)
        # Back face
        glVertex3f(-s.x/2, -s.y/2, -s.z/2)
        glVertex3f(-s.x/2, s.y/2, -s.z/2)
        glVertex3f(s.x/2, s.y/2, -s.z/2)
        glVertex3f(s.x/2, -s.y/2, -s.z/2)
        # Top face
        glVertex3f(-s.x/2, s.y/2, -s.z/2)
        glVertex3f(-s.x/2, s.y/2, s.z/2)
        glVertex3f(s.x/2, s.y/2, s.z/2)
        glVertex3f(s.x/2, s.y/2, -s.z/2)
        # Bottom face
        glVertex3f(-s.x/2, -s.y/2, -s.z/2)
        glVertex3f(s.x/2, -s.y/2, -s.z/2)
        glVertex3f(s.x/2, -s.y/2, s.z/2)
        glVertex3f(-s.x/2, -s.y/2, s.z/2)
        # Right face
        glVertex3f(s.x/2, -s.y/2, -s.z/2)
        glVertex3f(s.x/2, s.y/2, -s.z/2)
        glVertex3f(s.x/2, s.y/2, s.z/2)
        glVertex3f(s.x/2, -s.y/2, s.z/2)
        # Left face
        glVertex3f(-s.x/2, -s.y/2, -s.z/2)
        glVertex3f(-s.x/2, -s.y/2, s.z/2)
        glVertex3f(-s.x/2, s.y/2, s.z/2)
        glVertex3f(-s.x/2, s.y/2, -s.z/2)
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
                
                glColor3f(1.0 if i < stacks/2 else 0.0, 0.0, 0.0 if i < stacks/2 else 1.0)
                glVertex3f(x1, y1, z1)
                glVertex3f(x2, y2, z2)
            glEnd()


class Joint:
    """Joint constraint between rigid bodies"""
    def __init__(self, body0, body1, global_frame_pos, global_frame_rot=None):
        self.type = JointType.NONE
        self.body0 = body0
        self.body1 = body1
        self.disabled = False
        
        if global_frame_rot is None:
            global_frame_rot = Quaternion(0, 0, 0, 1)
        
        # Distance constraints
        self.has_target_distance = False
        self.target_distance = 0.0
        self.distance_compliance = 0.0
        self.distance_min = float('-inf')
        self.distance_max = float('inf')
        self.linear_damping_coeff = 0.0
        
        # Orientation constraints
        self.swing_min = float('-inf')
        self.swing_max = float('inf')
        self.twist_min = float('-inf')
        self.twist_max = float('inf')
        self.target_angle = 0.0
        self.has_target_angle = False
        self.target_angle_compliance = 0.0
        self.angular_damping_coeff = 0.0
        
        # Motor
        self.velocity = 0.0
        
        # Frame positions and rotations
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
        """Set joint frames relative to bodies"""
        if global_frame_rot is None:
            global_frame_rot = Quaternion(0, 0, 0, 1)
        
        if self.body0:
            self.local_pos0 = self.body0.world_to_local(global_frame_pos)
            self.local_rot0 = Quaternion().multiply_quaternions(self.body0.inv_rot, global_frame_rot)
        else:
            self.local_pos0 = global_frame_pos.copy()
            self.local_rot0 = global_frame_rot.copy()
        
        if self.body1:
            self.local_pos1 = self.body1.world_to_local(global_frame_pos)
            self.local_rot1 = Quaternion().multiply_quaternions(self.body1.inv_rot, global_frame_rot)
        else:
            self.local_pos1 = global_frame_pos.copy()
            self.local_rot1 = global_frame_rot.copy()
    
    def update_global_frames(self):
        """Update global frame positions from local frames"""
        if self.body0:
            self.global_pos0 = self.body0.local_to_world(self.local_pos0)
            self.global_rot0 = Quaternion().multiply_quaternions(self.body0.rot, self.local_rot0)
        else:
            self.global_pos0 = self.local_pos0.copy()
            self.global_rot0 = self.local_rot0.copy()
        
        if self.body1:
            self.global_pos1 = self.body1.local_to_world(self.local_pos1)
            self.global_rot1 = Quaternion().multiply_quaternions(self.body1.rot, self.local_rot1)
        else:
            self.global_pos1 = self.local_pos1.copy()
            self.global_rot1 = self.local_rot1.copy()
    
    def init_hinge_joint(self, swing_min, swing_max, has_target_angle, target_angle, compliance, damping):
        """Initialize as hinge joint"""
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
        """Initialize as servo joint"""
        self.type = JointType.SERVO
        self.has_target_distance = True
        self.target_distance = 0.0
        self.swing_min = swing_min
        self.swing_max = swing_max
        self.has_target_angle = True
        self.target_angle = 0.0
        self.target_angle_compliance = 0.0
    
    def init_motor(self, velocity):
        """Initialize as motor joint"""
        self.type = JointType.MOTOR
        self.has_target_distance = True
        self.target_distance = 0.0
        self.velocity = velocity
        self.has_target_angle = True
        self.target_angle = 0.0
        self.target_angle_compliance = 0.0
    
    def init_ball_joint(self, swing_max, twist_min, twist_max, damping):
        """Initialize as ball joint"""
        self.type = JointType.BALL
        self.has_target_distance = True
        self.target_distance = 0.0
        self.swing_min = 0.0
        self.swing_max = swing_max
        self.twist_min = twist_min
        self.twist_max = twist_max
        self.angular_damping_coeff = damping
    
    def init_distance_joint(self, rest_distance, compliance, damping):
        """Initialize as distance joint"""
        self.type = JointType.DISTANCE
        self.has_target_distance = True
        self.target_distance = rest_distance
        self.distance_compliance = compliance
        self.linear_damping_coeff = damping
    
    def init_prismatic_joint(self, distance_min, distance_max, twist_min, twist_max, has_target, target_distance, target_compliance, damping):
        """Initialize as prismatic joint"""
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
    
    def init_cylinder_joint(self, distance_min, distance_max, twist_min, twist_max, has_target_distance, rest_distance, compliance, damping):
        """Initialize as cylinder joint"""
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
        """Get angle between two vectors around normal"""
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
        """Limit angle between vectors"""
        phi = self.get_angle(n, a, b)
        if min_angle <= phi <= max_angle:
            return
        phi = max(min_angle, min(phi, max_angle))
        
        # Apply rotation correction
        ra = a.copy()
        # Simplified: apply correction
        corr = ra.cross(b)
        self.body0.apply_correction(compliance, corr, None, self.body1, None)
    
    def solve_position(self, dt):
        """Solve position constraints"""
        if self.disabled or self.type == JointType.NONE:
            return
        
        hard_compliance = 0.0
        
        # Prismatic/Cylinder constraint
        if self.type == JointType.PRISMATIC or self.type == JointType.CYLINDER:
            self.target_distance = max(self.distance_min, min(self.target_distance, self.distance_max))
            self.update_global_frames()
            corr = self.global_pos1 - self.global_pos0
            
            # Transform to local frame
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
        
        # Distance constraint
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
            self.body0.apply_correction(self.distance_compliance, corr, self.global_pos0, self.body1, self.global_pos1)
    
    def solve_orientation(self, dt):
        """Solve orientation constraints"""
        if self.disabled or self.type == JointType.NONE or self.type == JointType.DISTANCE:
            return
        
        if self.type == JointType.MOTOR:
            a_angle = max(-1.0, min(self.velocity * dt, 1.0))
            self.target_angle += a_angle
        
        hard_compliance = 0.0
        axis0 = Vector3(1.0, 0.0, 0.0)
        axis1 = Vector3(0.0, 1.0, 0.0)
        
        if self.type == JointType.HINGE or self.type == JointType.SERVO or self.type == JointType.MOTOR:
            # Align axes
            self.update_global_frames()
            a0 = self.global_rot0.apply_to_vector(axis0)
            a1 = self.global_rot1.apply_to_vector(axis0)
            corr = a0.cross(a1)
            self.body0.apply_correction(hard_compliance, corr, None, self.body1, None)
            
            # Target angle
            if self.has_target_angle:
                self.update_global_frames()
                n = self.global_rot0.apply_to_vector(axis0)
                a0 = self.global_rot0.apply_to_vector(axis1)
                a1 = self.global_rot1.apply_to_vector(axis1)
                self.limit_angle(n, a0, a1, self.target_angle, self.target_angle, self.target_angle_compliance)
            
            # Joint limits
            if self.swing_min > float('-inf') or self.swing_max < float('inf'):
                self.update_global_frames()
                n = self.global_rot0.apply_to_vector(axis0)
                a0 = self.global_rot0.apply_to_vector(axis1)
                a1 = self.global_rot1.apply_to_vector(axis1)
                self.limit_angle(n, a0, a1, self.swing_min, self.swing_max, hard_compliance)
        
        elif self.type == JointType.BALL or self.type == JointType.PRISMATIC or self.type == JointType.CYLINDER:
            # Swing limit
            self.update_global_frames()
            a0 = self.global_rot0.apply_to_vector(axis0)
            a1 = self.global_rot1.apply_to_vector(axis0)
            n = a0.cross(a1)
            n.normalize()
            self.limit_angle(n, a0, a1, self.swing_min, self.swing_max, hard_compliance)
            
            # Twist limit
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
            self.limit_angle(n, a0, a1, self.twist_min, self.twist_max, hard_compliance)
    
    def solve(self, dt):
        """Solve all constraints"""
        self.solve_position(dt)
        self.solve_orientation(dt)
    
    def apply_linear_damping(self, dt):
        """Apply linear damping"""
        self.update_global_frames()
        d_vel = self.body0.get_velocity_at(self.global_pos0)
        if self.body1:
            d_vel = d_vel - self.body1.get_velocity_at(self.global_pos1)
        
        n = (self.global_pos1 - self.global_pos0).normalized()
        n = n * (-d_vel.dot(n))
        n = n * min(self.linear_damping_coeff * dt, 1.0)
        self.body0.apply_correction(0.0, n, self.global_pos0, self.body1, self.global_pos1, True)
    
    def apply_angular_damping(self, dt, coeff=None):
        """Apply angular damping"""
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
        """Render joint visualization"""
        if self.disabled:
            return
        
        self.update_global_frames()
        
        # Draw connection line
        glLineWidth(2.0)
        glBegin(GL_LINES)
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(self.global_pos0.x, self.global_pos0.y, self.global_pos0.z)
        glVertex3f(self.global_pos1.x, self.global_pos1.y, self.global_pos1.z)
        glEnd()
        
        # Draw frame indicators (simplified)
        glPointSize(5.0)
        glBegin(GL_POINTS)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(self.global_pos0.x, self.global_pos0.y, self.global_pos0.z)
        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(self.global_pos1.x, self.global_pos1.y, self.global_pos1.z)
        glEnd()


class RigidBodySimulator:
    """Main physics simulator"""
    def __init__(self, gravity):
        self.gravity = gravity.copy()
        self.dt = 1.0 / 30.0  # 30 FPS
        self.num_sub_steps = 20
        self.num_iterations = 1
        self.rigid_bodies = []
        self.joints = []
        self.drag_joint = None
        self.simulation_view = True
        self.control_vector = Vector3(0, 0, 0)
        self.is_dragging = False
        
        self.add_drag_joint()
    
    def add_drag_joint(self):
        """Add drag joint for mouse interaction"""
        drag_compliance = 0.01
        drag_damping = 0.1
        self.drag_joint = Joint(None, None, Vector3(0, 0, 0))
        self.drag_joint.init_distance_joint(0.0, drag_compliance, drag_damping)
        self.drag_joint.disabled = True
    
    def clear(self):
        """Clear all bodies and joints"""
        self.rigid_bodies = []
        self.joints = []
        self.add_drag_joint()
    
    def add_rigid_body(self, rigid_body):
        """Add a rigid body to simulation"""
        self.rigid_bodies.append(rigid_body)
    
    def add_joint(self, joint):
        """Add a joint to simulation"""
        self.joints.append(joint)
    
    def update_control(self):
        """Update control inputs"""
        # Apply control to motors and servos
        for joint in self.joints:
            if joint.type == JointType.MOTOR:
                joint.velocity = self.control_vector.y * 5.0
            elif joint.type == JointType.SERVO:
                joint.target_angle = self.control_vector.x * math.pi / 4
            elif joint.type == JointType.CYLINDER:
                joint.target_distance = -self.control_vector.y * 0.1
    
    def simulate(self):
        """Run one simulation step"""
        self.update_control()
        
        sdt = self.dt / self.num_sub_steps
        
        for sub_step in range(self.num_sub_steps):
            # Integrate
            for body in self.rigid_bodies:
                body.integrate(sdt, self.gravity)
            
            # Solve constraints
            for joint in self.joints:
                joint.solve(sdt)
            
            if self.drag_joint:
                self.drag_joint.solve(sdt)
            
            # Update velocities
            for body in self.rigid_bodies:
                body.update_velocities()
            
            # Apply damping
            for joint in self.joints:
                joint.apply_linear_damping(sdt)
                joint.apply_angular_damping(sdt)
    
    def start_drag(self, body, pos):
        """Start dragging a body"""
        self.drag_joint.body0 = body
        self.drag_joint.set_frames(pos)
        self.drag_joint.disabled = False
    
    def drag(self, pos):
        """Update drag position"""
        self.drag_joint.local_pos1 = pos
    
    def end_drag(self):
        """End dragging"""
        self.drag_joint.disabled = True


class SceneImporter:
    """Import scenes from JSON"""
    def __init__(self, simulator):
        self.simulator = simulator
        self.rigid_bodies = {}
    
    def load_scene(self, json_data):
        """Load scene from JSON data"""
        if isinstance(json_data, str):
            data = json.loads(json_data)
        else:
            data = json_data
        
        if not data or 'meshes' not in data or len(data['meshes']) == 0:
            print("Empty scene data")
            return
        
        self.simulator.clear()
        self.rigid_bodies = {}
        
        # Pass 1: Create rigid bodies
        for mesh in data['meshes']:
            if self._is_rigid_body(mesh):
                self._create_rigid_body(mesh)
        
        # Pass 2: Create joints
        for mesh in data['meshes']:
            if self._is_joint(mesh):
                self._create_joint(mesh)
        
        # Pass 3: Create visual meshes
        for mesh in data['meshes']:
            if self._is_visual(mesh):
                self._create_visual_mesh(mesh)
    
    def _is_rigid_body(self, mesh):
        """Check if mesh is a rigid body"""
        sim_type = mesh.get('properties', {}).get('simType', '')
        return sim_type.startswith('Rigid')
    
    def _is_joint(self, mesh):
        """Check if mesh is a joint"""
        sim_type = mesh.get('properties', {}).get('simType', '')
        return sim_type.endswith('Joint')
    
    def _is_visual(self, mesh):
        """Check if mesh is a visual mesh"""
        sim_type = mesh.get('properties', {}).get('simType', '')
        return sim_type == 'Visual'
    
    def _create_rigid_body(self, mesh):
        """Create a rigid body from mesh data"""
        props = mesh.get('properties', {})
        sim_type = props.get('simType', '')
        density = props.get('density', 0.0)
        
        transform = mesh.get('transform', {})
        pos = Vector3(*transform.get('position', [0, 0, 0]))
        rot_quat = transform.get('rotation', [0, 0, 0, 1])
        # JSON format is [x, y, z, w], Quaternion constructor expects (x, y, z, w)
        quat = Quaternion(rot_quat[0], rot_quat[1], rot_quat[2], rot_quat[3])
        quat.normalize()
        
        # Convert quaternion to euler angles for RigidBody constructor
        # Using standard conversion: roll (x), pitch (y), yaw (z)
        w, x, y, z = rot_quat[3], rot_quat[0], rot_quat[1], rot_quat[2]
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        angles = Vector3(roll, pitch, yaw)
        
        if sim_type == 'RigidBox':
            vertices = mesh.get('vertices', [])
            size = self._calculate_bounding_box(vertices)
            rigid_body = RigidBody("box", size, density, pos, angles)
        elif sim_type == 'RigidSphere':
            vertices = mesh.get('vertices', [])
            radius = self._calculate_bounding_sphere(vertices)
            size = Vector3(radius, radius, radius)
            rigid_body = RigidBody("sphere", size, density, pos, angles)
        else:
            return
        
        # Override with the exact quaternion from JSON (more accurate)
        rigid_body.rot = quat.copy()
        rigid_body.inv_rot = rigid_body.rot.copy()
        rigid_body.inv_rot.invert()
        
        self.rigid_bodies[mesh.get('name', '')] = rigid_body
        self.simulator.add_rigid_body(rigid_body)
    
    def _calculate_bounding_box(self, vertices):
        """Calculate bounding box from vertices"""
        if len(vertices) < 3:
            return Vector3(1, 1, 1)
        
        min_vals = [float('inf')] * 3
        max_vals = [float('-inf')] * 3
        
        for i in range(0, len(vertices), 3):
            for j in range(3):
                if i + j < len(vertices):
                    min_vals[j] = min(min_vals[j], vertices[i + j])
                    max_vals[j] = max(max_vals[j], vertices[i + j])
        
        return Vector3(max_vals[0] - min_vals[0], max_vals[1] - min_vals[1], max_vals[2] - min_vals[2])
    
    def _calculate_bounding_sphere(self, vertices):
        """Calculate bounding sphere radius"""
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
            radius = math.sqrt(dx*dx + dy*dy + dz*dz)
            max_radius = max(max_radius, radius)
        
        return max_radius
    
    def _create_joint(self, mesh):
        """Create a joint from mesh data"""
        props = mesh.get('properties', {})
        sim_type = props.get('simType', '')
        
        body0 = self.rigid_bodies.get(props.get('parent1', ''))
        body1 = self.rigid_bodies.get(props.get('parent2', ''))
        
        if not body0 or not body1:
            return
        
        transform = mesh.get('transform', {})
        joint_pos = Vector3(*transform.get('position', [0, 0, 0]))
        rot_quat = transform.get('rotation', [0, 0, 0, 1])
        joint_rot = Quaternion(*rot_quat)
        
        joint = Joint(body0, body1, joint_pos, joint_rot)
        
        if sim_type == 'BallJoint':
            swing_max = props.get('swingMax', float('inf'))
            twist_min = props.get('twistMin', float('-inf'))
            twist_max = props.get('twistMax', float('inf'))
            damping = props.get('damping', 0.0)
            joint.init_ball_joint(swing_max, twist_min, twist_max, damping)
        elif sim_type == 'HingeJoint':
            swing_min = props.get('swingMin', float('-inf'))
            swing_max = props.get('swingMax', float('inf'))
            has_target = props.get('targetAngle') is not None
            target_angle = props.get('targetAngle', 0.0)
            compliance = props.get('targetAngleCompliance', 0.0)
            damping = props.get('damping', 0.0)
            joint.init_hinge_joint(swing_min, swing_max, has_target, target_angle, compliance, damping)
        elif sim_type == 'ServoJoint':
            swing_min = props.get('swingMin', float('-inf'))
            swing_max = props.get('swingMax', float('inf'))
            joint.init_servo(swing_min, swing_max)
        elif sim_type == 'MotorJoint':
            velocity = props.get('velocity', 3.0)
            joint.init_motor(velocity)
        elif sim_type == 'DistanceJoint':
            rest_distance = props.get('restDistance', 0.0)
            compliance = props.get('compliance', 0.0)
            damping = props.get('damping', 0.0)
            joint.init_distance_joint(rest_distance, compliance, damping)
        elif sim_type == 'PrismaticJoint':
            distance_min = props.get('distanceMin', float('-inf'))
            distance_max = props.get('distanceMax', float('inf'))
            twist_min = props.get('twistMin', float('-inf'))
            twist_max = props.get('twistMax', float('inf'))
            has_target = props.get('distanceTarget') is not None
            target_distance = props.get('posTarget', 0.0)
            compliance = props.get('compliance', 0.0)
            damping = props.get('damping', 0.0)
            joint.init_prismatic_joint(distance_min, distance_max, twist_min, twist_max, has_target, target_distance, compliance, damping)
        elif sim_type == 'CylinderJoint':
            distance_min = props.get('distanceMin', float('-inf'))
            distance_max = props.get('distanceMax', float('inf'))
            twist_min = props.get('twistMin', float('-inf'))
            twist_max = props.get('twistMax', float('inf'))
            joint.init_cylinder_joint(distance_min, distance_max, twist_min, twist_max, True, 0.0, 0.0, 0.0)
        
        self.simulator.add_joint(joint)
    
    def _create_visual_mesh(self, mesh):
        """Create a visual mesh attached to a rigid body"""
        props = mesh.get('properties', {})
        parent_name = props.get('parent', '')
        
        # Look up parent body
        parent_body = self.rigid_bodies.get(parent_name)
        if not parent_body:
            print(f"Warning: Parent body '{parent_name}' not found for visual mesh")
            return
        
        # Get visual mesh transform
        transform = mesh.get('transform', {})
        visual_pos = Vector3(*transform.get('position', [0, 0, 0]))
        rot_quat = transform.get('rotation', [0, 0, 0, 1])
        visual_rot = Quaternion(rot_quat[0], rot_quat[1], rot_quat[2], rot_quat[3])
        visual_rot.normalize()
        
        # Get vertices and triangles
        vertices = mesh.get('vertices', [])
        triangles = mesh.get('triangles', [])
        color = props.get('color', [1.0, 1.0, 1.0])
        
        # Transform visual mesh vertices to parent body local space
        # Visual mesh vertices are in visual mesh local space
        # Visual mesh has transform (visual_pos, visual_rot) in world space
        # Parent body has transform (parent_body.pos, parent_body.rot) in world space
        # We need: vertex_parent_local = parent_body.inv_rot * (visual_rot * vertex_visual_local + visual_pos - parent_body.pos)
        
        # Relative position in parent's local space
        p_rel_world = visual_pos - parent_body.pos
        p_rel_local = parent_body.inv_rot.apply_to_vector(p_rel_world)
        
        # Relative rotation: what rotation to apply in parent's local space
        # visual_rot_world = parent_rot * q_rel, so q_rel = parent_rot^-1 * visual_rot
        q_rel = Quaternion().multiply_quaternions(parent_body.inv_rot, visual_rot)
        
        # Transform vertices from visual mesh local to parent body local
        transformed_vertices = []
        for i in range(0, len(vertices), 3):
            if i + 2 < len(vertices):
                # Vertex in visual mesh local space
                vertex_visual = Vector3(vertices[i], vertices[i + 1], vertices[i + 2])
                # Transform to world space: visual_rot * vertex + visual_pos
                vertex_world = visual_rot.apply_to_vector(vertex_visual) + visual_pos
                # Transform to parent local space: parent_inv_rot * (vertex_world - parent_pos)
                vertex_parent = vertex_world - parent_body.pos
                vertex_parent = parent_body.inv_rot.apply_to_vector(vertex_parent)
                transformed_vertices.extend([vertex_parent.x, vertex_parent.y, vertex_parent.z])
        
        # Store visual mesh data
        visual_mesh_data = {
            'vertices': transformed_vertices,
            'triangles': triangles,
            'color': color
        }
        
        parent_body.visual_meshes.append(visual_mesh_data)
        print(f"Added visual mesh to body '{parent_name}' with {len(transformed_vertices)//3} vertices")


# Main application class
class JointSimulationApp:
    """Main application"""
    def __init__(self):
        self.screen = None
        self.clock = None
        self.paused = True
        self.simulator = None
        self.scene_importer = None
        self.current_scene = 0
        
        # Camera
        self.camera_pos = Vector3(0, 0.4, 1.6)
        self.camera_target = Vector3(0, 0.4, 0)
        self.camera_up = Vector3(0, 1, 0)
        
        # Mouse interaction
        self.mouse_down = False
        self.drag_body = None
        self.drag_distance = 0.0
        self.ray_origin = Vector3(0, 0, 0)
        self.ray_direction = Vector3(0, 0, 0)
        
        # UI
        self.font = None
        
        # Matrix storage for unprojection
        from ctypes import c_float
        self._proj_matrix = (c_float * 16)()
        self._modelview_matrix = (c_float * 16)()
    
    def init(self):
        """Initialize application"""
        pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.DOUBLEBUF | pygame.OPENGL)
        pygame.display.set_caption("Joint Simulation")
        
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        
        # Setup lighting
        glLightfv(GL_LIGHT0, GL_POSITION, [2, 3, 2, 1])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.5, 0.5, 0.5, 1])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [1, 1, 1, 1])
        
        glClearColor(0.0, 0.0, 0.0, 1.0)
        
        self.clock = pygame.time.Clock()
        self.screen = pygame.display.get_surface()
        
        # Initialize simulator
        gravity = Vector3(0.0, -9.81, 0.0)
        self.simulator = RigidBodySimulator(gravity)
        self.scene_importer = SceneImporter(self.simulator)
        
        # Load scene from JSON file
        self.load_scene_from_file()
    
    def load_scene_from_file(self):
        """Load scene from basicJoints.json file"""
        try:
            json_path = os.path.join(os.path.dirname(__file__), 'basicJoints.json')
            with open(json_path, 'r') as f:
                scene_data = json.load(f)
            self.scene_importer.load_scene(scene_data)
            print(f"Loaded scene from {json_path}")
        except FileNotFoundError:
            print(f"Warning: basicJoints.json not found. Creating simple test scene.")
            self.create_test_scene()
        except Exception as e:
            print(f"Error loading scene: {e}")
            self.create_test_scene()
    
    def create_test_scene(self):
        """Create a simple test scene"""
        # Ground
        ground = RigidBody("box", Vector3(10, 0.1, 10), 0.0, Vector3(0, -0.5, 0), Vector3(0, 0, 0))
        self.simulator.add_rigid_body(ground)
        
        # Box on ground
        box = RigidBody("box", Vector3(0.2, 0.2, 0.2), 1.0, Vector3(0, 0.2, 0), Vector3(0, 0, 0))
        self.simulator.add_rigid_body(box)
        
        # Connect with distance joint
        joint = Joint(ground, box, Vector3(0, 0, 0))
        joint.init_distance_joint(0.3, 0.01, 0.1)
        self.simulator.add_joint(joint)
    
    def handle_events(self):
        """Handle input events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_r:
                    self.load_scene_from_file()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    self.handle_mouse_down(event.pos)
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    self.handle_mouse_up()
            elif event.type == pygame.MOUSEMOTION:
                if self.mouse_down:
                    self.handle_mouse_drag(event.pos)
        
        return True
    
    def handle_mouse_down(self, pos):
        """Handle mouse down"""
        self.mouse_down = True
        
        # Convert screen position to ray
        ray_origin, ray_dir = self.screen_to_ray(pos)
        self.ray_origin = ray_origin
        self.ray_direction = ray_dir
        
        # Find closest body intersection
        closest_body = None
        closest_t = float('inf')
        
        for body in self.simulator.rigid_bodies:
            t, _ = self.ray_intersect_body(ray_origin, ray_dir, body)
            if t is not None and t < closest_t and t > 0:
                closest_t = t
                closest_body = body
        
        # Only start dragging if we found a body and it's reasonably close
        if closest_body and closest_t < 100.0:  # 100 units max distance
            self.drag_body = closest_body
            self.drag_distance = closest_t
            # Calculate world position at intersection
            world_pos = ray_origin + ray_dir * closest_t
            self.simulator.start_drag(self.drag_body, world_pos)
            if self.paused:
                self.paused = False
        else:
            self.drag_body = None
            self.drag_distance = 0.0
    
    def handle_mouse_up(self):
        """Handle mouse up"""
        self.mouse_down = False
        if self.drag_body:
            self.simulator.end_drag()
            self.drag_body = None
    
    def handle_mouse_drag(self, pos):
        """Handle mouse drag"""
        if self.drag_body:
            # Convert screen position to ray
            ray_origin, ray_dir = self.screen_to_ray(pos)
            # Use the same distance as when we started dragging
            world_pos = ray_origin + ray_dir * self.drag_distance
            self.simulator.drag(world_pos)
    
    def screen_to_ray(self, screen_pos):
        """Convert screen coordinates to a ray (origin and direction)"""
        # Normalize screen coordinates to [-1, 1]
        x_ndc = (2.0 * screen_pos[0] / WINDOW_WIDTH) - 1.0
        y_ndc = 1.0 - (2.0 * screen_pos[1] / WINDOW_HEIGHT)  # Flip Y axis
        
        # Camera parameters
        fov = 70.0
        aspect = WINDOW_WIDTH / WINDOW_HEIGHT
        near = 0.01
        
        # Calculate camera direction and right vectors
        forward = self.camera_target - self.camera_pos
        forward.normalize()
        right = forward.cross(self.camera_up)
        right.normalize()
        up = right.cross(forward)
        up.normalize()
        
        # Calculate frustum dimensions at near plane
        fov_rad = math.radians(fov)
        tan_half_fov = math.tan(fov_rad / 2.0)
        near_height = 2.0 * tan_half_fov * near
        near_width = near_height * aspect
        
        # Calculate ray direction in camera space
        ray_dir_camera = Vector3(
            x_ndc * near_width / 2.0,
            y_ndc * near_height / 2.0,
            -near
        )
        
        # Transform ray direction to world space
        ray_dir_world = (
            right * ray_dir_camera.x +
            up * ray_dir_camera.y +
            forward * (-ray_dir_camera.z)
        )
        ray_dir_world.normalize()
        
        return self.camera_pos, ray_dir_world
    
    def ray_intersect_body(self, ray_origin, ray_dir, body):
        """Find intersection of ray with a body (simple sphere/box intersection)"""
        # Transform ray to body local space
        body_to_ray_origin = ray_origin - body.pos
        local_origin = body.inv_rot.apply_to_vector(body_to_ray_origin)
        local_dir = body.inv_rot.apply_to_vector(ray_dir)
        
        # Simple sphere intersection (using body's bounding sphere)
        if body.type == "sphere":
            radius = body.size.x  # Assuming size.x is the radius
            oc = local_origin
            a = local_dir.length_sq()
            b = 2.0 * (oc.x * local_dir.x + oc.y * local_dir.y + oc.z * local_dir.z)
            c = oc.length_sq() - radius * radius
            discriminant = b * b - 4 * a * c
            
            if discriminant < 0:
                return None, float('inf')
            
            t = (-b - math.sqrt(discriminant)) / (2.0 * a)
            if t < 0:
                t = (-b + math.sqrt(discriminant)) / (2.0 * a)
            if t < 0:
                return None, float('inf')
            
            return t, t
        
        # Box intersection
        elif body.type == "box":
            half_size = Vector3(body.size.x / 2.0, body.size.y / 2.0, body.size.z / 2.0)
            
            # Ray-box intersection using slab method
            tmin = -float('inf')
            tmax = float('inf')
            
            # Check X axis
            if abs(local_dir.x) < 1e-6:
                if local_origin.x < -half_size.x or local_origin.x > half_size.x:
                    return None, float('inf')
            else:
                ood = 1.0 / local_dir.x
                t1 = (-half_size.x - local_origin.x) * ood
                t2 = (half_size.x - local_origin.x) * ood
                if t1 > t2:
                    t1, t2 = t2, t1
                if t1 > tmin:
                    tmin = t1
                if t2 < tmax:
                    tmax = t2
                if tmin > tmax:
                    return None, float('inf')
            
            # Check Y axis
            if abs(local_dir.y) < 1e-6:
                if local_origin.y < -half_size.y or local_origin.y > half_size.y:
                    return None, float('inf')
            else:
                ood = 1.0 / local_dir.y
                t1 = (-half_size.y - local_origin.y) * ood
                t2 = (half_size.y - local_origin.y) * ood
                if t1 > t2:
                    t1, t2 = t2, t1
                if t1 > tmin:
                    tmin = t1
                if t2 < tmax:
                    tmax = t2
                if tmin > tmax:
                    return None, float('inf')
            
            # Check Z axis
            if abs(local_dir.z) < 1e-6:
                if local_origin.z < -half_size.z or local_origin.z > half_size.z:
                    return None, float('inf')
            else:
                ood = 1.0 / local_dir.z
                t1 = (-half_size.z - local_origin.z) * ood
                t2 = (half_size.z - local_origin.z) * ood
                if t1 > t2:
                    t1, t2 = t2, t1
                if t1 > tmin:
                    tmin = t1
                if t2 < tmax:
                    tmax = t2
                if tmin > tmax:
                    return None, float('inf')
            
            if tmin < 0:
                tmin = tmax
            if tmin < 0:
                return None, float('inf')
            
            return tmin, tmin
        
        return None, float('inf')
    
    def update(self):
        """Update simulation"""
        if not self.paused:
            self.simulator.simulate()
    
    def render(self):
        """Render scene"""
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        
        # Setup camera
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(70, WINDOW_WIDTH / WINDOW_HEIGHT, 0.01, 100)
        
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(
            self.camera_pos.x, self.camera_pos.y, self.camera_pos.z,
            self.camera_target.x, self.camera_target.y, self.camera_target.z,
            self.camera_up.x, self.camera_up.y, self.camera_up.z
        )
        
        # Render ground
        glPushMatrix()
        glTranslatef(0, -0.5, 0)
        glScalef(10, 0.1, 10)
        glColor3f(0.6, 0.7, 0.7)
        glBegin(GL_QUADS)
        glVertex3f(-0.5, -0.5, -0.5)
        glVertex3f(0.5, -0.5, -0.5)
        glVertex3f(0.5, -0.5, 0.5)
        glVertex3f(-0.5, -0.5, 0.5)
        glEnd()
        glPopMatrix()
        
        # Render bodies
        for body in self.simulator.rigid_bodies:
            body.render()
        
        # Render joints
        for joint in self.simulator.joints:
            joint.render()
        
        if self.simulator.drag_joint:
            self.simulator.drag_joint.render()
        
        pygame.display.flip()
    
    def run(self):
        """Main loop"""
        self.init()
        running = True
        
        while running:
            running = self.handle_events()
            self.update()
            self.render()
            self.clock.tick(FPS)
        
        pygame.quit()


if __name__ == "__main__":
    app = JointSimulationApp()
    app.run()
