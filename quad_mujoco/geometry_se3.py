"""SE3 geometry utilities for quadrotor control.

This module provides geometric tools for SE(3) based quadrotor control,
including quaternion operations, SO(3) and SE(3) transformations.
Based on the Quadrotor_SE3_Control repository implementation.
"""

import numpy as np
from typing import Tuple


class GeoQuaternion:
    """Quaternion class for SE3 control operations.

    Quaternion order: x, y, z, w (scalar last)
    """

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
        self.normalize()

    def vec(self) -> np.ndarray:
        """Return quaternion as numpy array [x, y, z, w]."""
        return np.array([self.x, self.y, self.z, self.w])

    def setFromAngleAxis(self, angle: float, axis: np.ndarray) -> 'GeoQuaternion':
        """Set quaternion from angle-axis representation."""
        self.x = np.sin(angle/2) * axis[0]
        self.y = np.sin(angle/2) * axis[1]
        self.z = np.sin(angle/2) * axis[2]
        self.w = np.cos(angle/2)
        self.normalize()
        return self

    def setFromRotationMatrix(self, mat: np.ndarray) -> 'GeoQuaternion':
        """Set quaternion from rotation matrix."""
        assert mat.shape == (3, 3)
        t = np.trace(mat)
        if t > 0:
            t = np.sqrt(t + 1.0)
            self.w = 0.5 * t
            t = 0.5 / t
            self.x = (mat[2, 1] - mat[1, 2]) * t
            self.y = (mat[0, 2] - mat[2, 0]) * t
            self.z = (mat[1, 0] - mat[0, 1]) * t
        else:
            i = 0
            if mat[1, 1] > mat[0, 0]:
                i = 1
            if mat[2, 2] > mat[i, i]:
                i = 2
            j = (i + 1) % 3
            k = (j + 1) % 3

            t = np.sqrt(mat[i, i] - mat[j, j] - mat[k, k] + 1.0)

            if i == 0:
                self.x = 0.5 * t
            elif i == 1:
                self.y = 0.5 * t
            elif i == 2:
                self.z = 0.5 * t

            t = 0.5 / t
            self.w = (mat[k, j] - mat[j, k]) * t

            if j == 0:
                self.x = (mat[j, i] + mat[i, j]) * t
            elif j == 1:
                self.y = (mat[j, i] + mat[i, j]) * t
            elif j == 2:
                self.z = (mat[j, i] + mat[i, j]) * t

            if k == 0:
                self.x = (mat[k, i] + mat[i, k]) * t
            elif k == 1:
                self.y = (mat[k, i] + mat[i, k]) * t
            elif k == 2:
                self.z = (mat[k, i] + mat[i, k]) * t

        self.normalize()
        return self

    def norm(self) -> float:
        """Return quaternion norm."""
        return np.sqrt(self.x**2 + self.y**2 + self.z**2 + self.w**2)

    def normalize(self) -> 'GeoQuaternion':
        """Normalize quaternion."""
        n = self.norm()
        if n > 1e-10:
            self.x /= n
            self.y /= n
            self.z /= n
            self.w /= n
        return self

    def getRotationMatrix(self) -> np.ndarray:
        """Convert quaternion to rotation matrix."""
        tx = 2.0 * self.x
        ty = 2.0 * self.y
        tz = 2.0 * self.z
        twx = tx * self.w
        twy = ty * self.w
        twz = tz * self.w
        txx = tx * self.x
        txy = ty * self.x
        txz = tz * self.x
        tyy = ty * self.y
        tyz = tz * self.y
        tzz = tz * self.z

        R = np.zeros((3, 3))
        R[0, 0] = 1.0 - (tyy + tzz)
        R[0, 1] = txy - twz
        R[0, 2] = txz + twy
        R[1, 0] = txy + twz
        R[1, 1] = 1.0 - (txx + tzz)
        R[1, 2] = tyz - twx
        R[2, 0] = txz - twy
        R[2, 1] = tyz + twx
        R[2, 2] = 1.0 - (txx + tyy)
        return R

    def slerp(self, t: float, other: 'GeoQuaternion') -> 'GeoQuaternion':
        """Spherical linear interpolation between quaternions."""
        vec0 = self.vec()
        vec1 = other.vec()

        thresh = float(1.0) - np.spacing(1.0)
        d = np.dot(vec0, vec1)
        abs_d = np.abs(d)

        scale0 = 0
        scale1 = 0

        if abs_d >= thresh:
            scale0 = 1.0 - t
            scale1 = t
        else:
            theta = np.arccos(abs_d)
            sin_theta = np.sin(theta)

            scale0 = np.sin((1.0 - t) * theta) / sin_theta
            scale1 = np.sin((t * theta)) / sin_theta

        if d < 0:
            scale1 = -scale1

        new_vec = scale0 * vec0 + scale1 * vec1
        q = GeoQuaternion(new_vec[0], new_vec[1], new_vec[2], new_vec[3])
        return q.normalize()

    def __str__(self) -> str:
        return f"[{self.x:.17f}, {self.y:.17f}, {self.z:.17f}, {self.w:.17f}]"


def veemap(A: np.ndarray) -> np.ndarray:
    """Extract vector from so(3) skew-symmetric matrix.

    Args:
        A: 3x3 skew-symmetric matrix

    Returns:
        3D vector
    """
    assert A.shape == (3, 3)
    ret = np.zeros(3)
    ret[0] = A[2, 1]
    ret[1] = A[0, 2]
    ret[2] = A[1, 0]
    return ret


def hatmap(w: np.ndarray) -> np.ndarray:
    """Convert 3D vector to so(3) skew-symmetric matrix.

    Args:
        w: 3D vector

    Returns:
        3x3 skew-symmetric matrix
    """
    assert len(w) == 3
    ret = np.zeros((3, 3))
    ret[1, 0] = w[2]
    ret[2, 0] = -w[1]
    ret[0, 1] = -w[2]
    ret[2, 1] = w[0]
    ret[0, 2] = w[1]
    ret[1, 2] = -w[0]
    return ret


def skewSym(w: np.ndarray) -> np.ndarray:
    """Convert 3D vector to skew-symmetric matrix (alias for hatmap)."""
    return hatmap(w)


def so3LieToMat(w: np.ndarray) -> np.ndarray:
    """Convert SO(3) Lie algebra to rotation matrix using Rodrigues formula.

    Args:
        w: 3D vector representing rotation in Lie algebra

    Returns:
        3x3 rotation matrix
    """
    assert len(w) == 3
    theta = np.linalg.norm(w)

    if theta > 1e-3:
        A = np.sin(theta) / theta
        B = (1 - np.cos(theta)) / (theta**2)
    else:
        A = 1.0
        B = 0.5

    wx = skewSym(w)
    R = np.eye(3)
    R += A * wx + B * np.dot(wx, wx)
    return R


def se3LieToRotTrans3(w: np.ndarray, u: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Convert SE(3) Lie algebra to rotation matrix and translation vector.

    Args:
        w: 3D rotation vector in Lie algebra
        u: 3D translation vector in Lie algebra

    Returns:
        Tuple of (3x3 rotation matrix, 3D translation vector)
    """
    assert len(w) == 3
    assert len(u) == 3

    # Get rotation matrix
    theta = np.linalg.norm(w)
    if theta > 1e-3:
        A = np.sin(theta) / theta
        B = (1 - np.cos(theta)) / (theta**2)
    else:
        A = 1.0
        B = 0.5

    wx = skewSym(w)
    R = np.eye(3)
    R += A * wx + B * np.dot(wx, wx)

    # Get translation vector
    if theta > 1e-3:
        C = (1 - A) / (theta**2)
    else:
        C = 1.0 / 6.0

    V = np.eye(3)
    V += B * wx + C * np.dot(wx, wx)
    t = np.dot(V, u)

    return R, t


def se3LieToMat4(w: np.ndarray, u: np.ndarray) -> np.ndarray:
    """Convert SE(3) Lie algebra to 4x4 homogeneous transformation matrix.

    Args:
        w: 3D rotation vector in Lie algebra
        u: 3D translation vector in Lie algebra

    Returns:
        4x4 homogeneous transformation matrix
    """
    R, t = se3LieToRotTrans3(w, u)
    T = np.zeros((4, 4))
    T[0:3, 0:3] = R
    T[0:3, 3] = t
    T[3, 3] = 1.0
    return T


def rotTrans3ToMat4(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Convert rotation matrix and translation vector to 4x4 homogeneous matrix.

    Args:
        R: 3x3 rotation matrix
        t: 3D translation vector

    Returns:
        4x4 homogeneous transformation matrix
    """
    assert R.shape == (3, 3)
    assert len(t) == 3
    T = np.zeros((4, 4))
    T[0:3, 0:3] = R
    T[0:3, 3] = t
    T[3, 3] = 1.0
    return T


def quaternion_xyzw_to_wxyz(quat_xyzw: np.ndarray) -> np.ndarray:
    """Convert quaternion from [x, y, z, w] to [w, x, y, z] format.

    Args:
        quat_xyzw: Quaternion in [x, y, z, w] format

    Returns:
        Quaternion in [w, x, y, z] format
    """
    assert len(quat_xyzw) == 4
    return np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])


def quaternion_wxyz_to_xyzw(quat_wxyz: np.ndarray) -> np.ndarray:
    """Convert quaternion from [w, x, y, z] to [x, y, z, w] format.

    Args:
        quat_wxyz: Quaternion in [w, x, y, z] format

    Returns:
        Quaternion in [x, y, z, w] format
    """
    assert len(quat_wxyz) == 4
    return np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])