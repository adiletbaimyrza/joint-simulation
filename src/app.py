import os
import json
import math
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *

from .config import FPS, WINDOW_WIDTH, WINDOW_HEIGHT
from .math_utils import Vector3
from .rigid_body import RigidBody
from .joint import Joint
from .simulator import RigidBodySimulator
from .scene_importer import SceneImporter


class JointSimulationApp:

    def __init__(self):
        self.screen = None
        self.clock = None
        self.paused = True
        self.simulator = None
        self.scene_importer = None
        self.current_scene = 0

        self.camera_pos = Vector3(0, 0.4, 1.6)
        self.camera_target = Vector3(0, 0.4, 0)
        self.camera_up = Vector3(0, 1, 0)

        self.mouse_down = False
        self.drag_body = None
        self.drag_distance = 0.0
        self.ray_origin = Vector3(0, 0, 0)
        self.ray_direction = Vector3(0, 0, 0)

        self.font = None
        self.debug_font = None
        self.mouse_pos = (0, 0)

    def init(self):
        pygame.display.set_mode(
            (WINDOW_WIDTH, WINDOW_HEIGHT), pygame.DOUBLEBUF | pygame.OPENGL
        )
        pygame.display.set_caption("Joint Simulation")

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

        glLightfv(GL_LIGHT0, GL_POSITION, [2, 3, 2, 1])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.5, 0.5, 0.5, 1])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [1, 1, 1, 1])

        glClearColor(0.0, 0.0, 0.0, 1.0)

        self.clock = pygame.time.Clock()
        self.screen = pygame.display.get_surface()

        self.debug_font = pygame.font.Font(None, 20)

        gravity = Vector3(0.0, -9.81, 0.0)
        self.simulator = RigidBodySimulator(gravity)
        self.scene_importer = SceneImporter(self.simulator)

        self.load_scene_from_file()

    def load_scene_from_file(self):
        project_root = os.path.dirname(os.path.dirname(__file__))
        json_path = os.path.join(project_root, "basicJoints.json")
        try:
            with open(json_path, "r") as f:
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
        ground = RigidBody(
            "box", Vector3(10, 0.1, 10), 0.0, Vector3(0, -0.5, 0), Vector3(0, 0, 0)
        )
        self.simulator.add_rigid_body(ground)

        box = RigidBody(
            "box", Vector3(0.2, 0.2, 0.2), 1.0, Vector3(0, 0.2, 0), Vector3(0, 0, 0)
        )
        self.simulator.add_rigid_body(box)

        joint = Joint(ground, box, Vector3(0, 0, 0))
        joint.init_distance_joint(0.3, 0.01, 0.1)
        self.simulator.add_joint(joint)

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_r:
                    self.load_scene_from_file()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    self.handle_mouse_down(event.pos)
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    self.handle_mouse_up()
            elif event.type == pygame.MOUSEMOTION:
                self.mouse_pos = event.pos
                if self.mouse_down:
                    self.handle_mouse_drag(event.pos)

        return True

    def handle_mouse_down(self, pos):
        self.mouse_down = True

        ray_origin, ray_dir = self.screen_to_ray(pos)
        self.ray_origin = ray_origin
        self.ray_direction = ray_dir

        closest_body = None
        closest_t = float("inf")

        for body in self.simulator.rigid_bodies:
            t, _ = self.ray_intersect_body(ray_origin, ray_dir, body)
            if t is not None and t < closest_t and t > 0:
                closest_t = t
                closest_body = body

        if closest_body and closest_t < 100.0:
            self.drag_body = closest_body
            self.drag_distance = closest_t
            world_pos = ray_origin + ray_dir * closest_t
            self.simulator.start_drag(self.drag_body, world_pos)
            if self.paused:
                self.paused = False
        else:
            self.drag_body = None
            self.drag_distance = 0.0

    def handle_mouse_up(self):
        self.mouse_down = False
        if self.drag_body:
            self.simulator.end_drag()
            self.drag_body = None

    def handle_mouse_drag(self, pos):
        if self.drag_body:
            ray_origin, ray_dir = self.screen_to_ray(pos)
            world_pos = ray_origin + ray_dir * self.drag_distance
            self.simulator.drag(world_pos)

    def screen_to_ray(self, screen_pos):
        x_ndc = (2.0 * screen_pos[0] / WINDOW_WIDTH) - 1.0
        y_ndc = 1.0 - (2.0 * screen_pos[1] / WINDOW_HEIGHT)

        fov = 70.0
        aspect = WINDOW_WIDTH / WINDOW_HEIGHT
        near = 0.01

        forward = self.camera_target - self.camera_pos
        forward.normalize()
        right = forward.cross(self.camera_up)
        right.normalize()
        up = right.cross(forward)
        up.normalize()

        fov_rad = math.radians(fov)
        tan_half_fov = math.tan(fov_rad / 2.0)
        near_height = 2.0 * tan_half_fov * near
        near_width = near_height * aspect

        ray_dir_camera = Vector3(
            x_ndc * near_width / 2.0, y_ndc * near_height / 2.0, -near
        )

        ray_dir_world = (
            right * ray_dir_camera.x
            + up * ray_dir_camera.y
            + forward * (-ray_dir_camera.z)
        )
        ray_dir_world.normalize()

        return self.camera_pos, ray_dir_world

    def ray_intersect_body(self, ray_origin, ray_dir, body):
        body_to_ray_origin = ray_origin - body.pos
        local_origin = body.inv_rot.apply_to_vector(body_to_ray_origin)
        local_dir = body.inv_rot.apply_to_vector(ray_dir)

        if body.type == "sphere":
            radius = body.size.x
            oc = local_origin
            a = local_dir.length_sq()
            b = 2.0 * (oc.x * local_dir.x + oc.y * local_dir.y + oc.z * local_dir.z)
            c = oc.length_sq() - radius * radius
            discriminant = b * b - 4 * a * c

            if discriminant < 0:
                return None, float("inf")

            t = (-b - math.sqrt(discriminant)) / (2.0 * a)
            if t < 0:
                t = (-b + math.sqrt(discriminant)) / (2.0 * a)
            if t < 0:
                return None, float("inf")

            return t, t

        elif body.type == "box":
            half_size = Vector3(body.size.x / 2.0, body.size.y / 2.0, body.size.z / 2.0)

            tmin = -float("inf")
            tmax = float("inf")

            if abs(local_dir.x) < 1e-6:
                if local_origin.x < -half_size.x or local_origin.x > half_size.x:
                    return None, float("inf")
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
                    return None, float("inf")

            if abs(local_dir.y) < 1e-6:
                if local_origin.y < -half_size.y or local_origin.y > half_size.y:
                    return None, float("inf")
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
                    return None, float("inf")

            if abs(local_dir.z) < 1e-6:
                if local_origin.z < -half_size.z or local_origin.z > half_size.z:
                    return None, float("inf")
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
                    return None, float("inf")

            if tmin < 0:
                tmin = tmax
            if tmin < 0:
                return None, float("inf")

            return tmin, tmin

        return None, float("inf")

    def update(self):
        if not self.paused:
            self.simulator.simulate()

    def render(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(70, WINDOW_WIDTH / WINDOW_HEIGHT, 0.01, 100)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(
            self.camera_pos.x,
            self.camera_pos.y,
            self.camera_pos.z,
            self.camera_target.x,
            self.camera_target.y,
            self.camera_target.z,
            self.camera_up.x,
            self.camera_up.y,
            self.camera_up.z,
        )

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

        for body in self.simulator.rigid_bodies:
            body.render()

        for joint in self.simulator.joints:
            joint.render()

        if self.simulator.drag_joint:
            self.simulator.drag_joint.render()

        pygame.display.flip()

        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(0, WINDOW_WIDTH, WINDOW_HEIGHT, 0, -1, 1)

        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()

        glDisable(GL_DEPTH_TEST)
        glDisable(GL_LIGHTING)

        y_offset = 10
        line_height = 22

        fps = int(self.clock.get_fps())

        debug_lines = [
            f"FPS: {fps}",
            f"Paused: {self.paused}",
            "",
            "=== CAMERA ===",
            f"Pos: ({self.camera_pos.x:.3f}, {self.camera_pos.y:.3f}, {self.camera_pos.z:.3f})",
            f"Target: ({self.camera_target.x:.3f}, {self.camera_target.y:.3f}, {self.camera_target.z:.3f})",
            f"Up: ({self.camera_up.x:.3f}, {self.camera_up.y:.3f}, {self.camera_up.z:.3f})",
            "",
            "=== MOUSE ===",
            f"Screen Pos: {self.mouse_pos}",
            f"Mouse Down: {self.mouse_down}",
            f"Drag Body: {self.drag_body.type if self.drag_body else 'None'}",
            f"Drag Distance: {self.drag_distance:.3f}",
            f"Ray Origin: ({self.ray_origin.x:.3f}, {self.ray_origin.y:.3f}, {self.ray_origin.z:.3f})",
            f"Ray Dir: ({self.ray_direction.x:.3f}, {self.ray_direction.y:.3f}, {self.ray_direction.z:.3f})",
            "",
            "=== BODIES ===",
            f"Count: {len(self.simulator.rigid_bodies)}",
        ]

        for i, body in enumerate(self.simulator.rigid_bodies[:5]):
            mass = 1.0 / body.inv_mass if body.inv_mass > 0 else 0.0
            debug_lines.append(f"  Body {i} ({body.type}):")
            debug_lines.append(
                f"    Pos: ({body.pos.x:.3f}, {body.pos.y:.3f}, {body.pos.z:.3f})"
            )
            debug_lines.append(
                f"    Vel: ({body.vel.x:.3f}, {body.vel.y:.3f}, {body.vel.z:.3f})"
            )
            debug_lines.append(
                f"    AngVel: ({body.omega.x:.3f}, {body.omega.y:.3f}, {body.omega.z:.3f})"
            )
            debug_lines.append(
                f"    Rot: ({body.rot.x:.3f}, {body.rot.y:.3f}, {body.rot.z:.3f}, {body.rot.w:.3f})"
            )
            debug_lines.append(f"    Mass: {mass:.3f} (inv: {body.inv_mass:.3f})")
            debug_lines.append("")

        if len(self.simulator.rigid_bodies) > 5:
            debug_lines.append(
                f"  ... and {len(self.simulator.rigid_bodies) - 5} more bodies"
            )

        debug_lines.extend(
            [
                "",
                "=== JOINTS ===",
                f"Count: {len(self.simulator.joints)}",
            ]
        )

        for i, joint in enumerate(self.simulator.joints[:5]):
            debug_lines.append(f"  Joint {i} ({joint.type.name}):")
            debug_lines.append(
                f"    Body 0: {joint.body0.type if joint.body0 else 'None'}"
            )
            debug_lines.append(
                f"    Body 1: {joint.body1.type if joint.body1 else 'None'}"
            )
            debug_lines.append(
                f"    Global Pos 0: ({joint.global_pos0.x:.3f}, {joint.global_pos0.y:.3f}, {joint.global_pos0.z:.3f})"
            )
            debug_lines.append(
                f"    Global Pos 1: ({joint.global_pos1.x:.3f}, {joint.global_pos1.y:.3f}, {joint.global_pos1.z:.3f})"
            )
            debug_lines.append("")

        if len(self.simulator.joints) > 5:
            debug_lines.append(
                f"  ... and {len(self.simulator.joints) - 5} more joints"
            )

        if self.simulator.drag_joint and not self.simulator.drag_joint.disabled:
            debug_lines.extend(
                [
                    "",
                    "=== DRAG JOINT ===",
                    f"Active: True",
                    f"Body: {self.simulator.drag_joint.body0.type if self.simulator.drag_joint.body0 else 'None'}",
                    f"Global Pos 0: ({self.simulator.drag_joint.global_pos0.x:.3f}, {self.simulator.drag_joint.global_pos0.y:.3f}, {self.simulator.drag_joint.global_pos0.z:.3f})",
                    f"Local Pos 1: ({self.simulator.drag_joint.local_pos1.x:.3f}, {self.simulator.drag_joint.local_pos1.y:.3f}, {self.simulator.drag_joint.local_pos1.z:.3f})",
                ]
            )

        for line in debug_lines:
            if line:
                text_surface = self.debug_font.render(line, True, (255, 255, 255))
                bg_surface = pygame.Surface(
                    (text_surface.get_width() + 4, text_surface.get_height() + 2)
                )
                bg_surface.fill((0, 0, 0))
                bg_surface.blit(text_surface, (2, 1))

                text_data = pygame.image.tostring(bg_surface, "RGBA", True)
                width, height = bg_surface.get_size()

                texture = glGenTextures(1)
                glBindTexture(GL_TEXTURE_2D, texture)
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
                glTexImage2D(
                    GL_TEXTURE_2D,
                    0,
                    GL_RGBA,
                    width,
                    height,
                    0,
                    GL_RGBA,
                    GL_UNSIGNED_BYTE,
                    text_data,
                )

                glEnable(GL_BLEND)
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
                glEnable(GL_TEXTURE_2D)
                glBindTexture(GL_TEXTURE_2D, texture)

                glColor3f(1.0, 1.0, 1.0)
                glBegin(GL_QUADS)
                glTexCoord2f(0, 1)
                glVertex2f(10, y_offset)
                glTexCoord2f(1, 1)
                glVertex2f(10 + width, y_offset)
                glTexCoord2f(1, 0)
                glVertex2f(10 + width, y_offset + height)
                glTexCoord2f(0, 0)
                glVertex2f(10, y_offset + height)
                glEnd()

                glDisable(GL_TEXTURE_2D)
                glDisable(GL_BLEND)
                glDeleteTextures(1, [texture])

            y_offset += line_height

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)

        glPopMatrix()
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)

    def run(self):
        self.init()
        running = True

        while running:
            running = self.handle_events()
            self.update()
            self.render()
            self.clock.tick(FPS)

        pygame.quit()
