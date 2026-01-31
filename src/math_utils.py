import math
import numpy as np


class Vector3:

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
            self.x * other.y - self.y * other.x,
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
        self.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        self.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
        self.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
        self.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
        return self

    def set_from_euler(self, x, y, z):
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
        qv = Quaternion(v.x, v.y, v.z, 0)
        q_inv = self.conjugate()
        temp = Quaternion().multiply_quaternions(self, qv)
        result = Quaternion().multiply_quaternions(temp, q_inv)
        return Vector3(result.x, result.y, result.z)

    def premultiply(self, other):
        result = Quaternion().multiply_quaternions(other, self)
        self.x = result.x
        self.y = result.y
        self.z = result.z
        self.w = result.w
        return self

    def to_matrix(self):
        x, y, z, w = self.x, self.y, self.z, self.w
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0],
                [0, 0, 0, 1],
            ],
            dtype=np.float32,
        )
