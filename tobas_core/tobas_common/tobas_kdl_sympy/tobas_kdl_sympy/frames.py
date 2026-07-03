# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

from __future__ import annotations  # Required to annotate methods returning their own class.

from functools import singledispatchmethod
import math
from typing import Dict, Tuple

import numpy as np
import sympy
from sympy import Symbol, Matrix
from sympy.logic.boolalg import BooleanTrue


class Vector:
    def __init__(self, x: Symbol, y: Symbol, z: Symbol) -> None:
        # Keeping the three components as separate `Symbol`s is easier to use than storing them together in a `Matrix`.
        self.x = x
        self.y = y
        self.z = z

    @classmethod
    def Zero(cls) -> Vector:
        return cls(0, 0, 0)

    @classmethod
    def TransX(cls, x: Symbol) -> Vector:
        return cls(x, 0, 0)

    @classmethod
    def TransY(cls, y: Symbol) -> Vector:
        return cls(0, y, 0)

    @classmethod
    def TransZ(cls, z: Symbol) -> Vector:
        return cls(0, 0, z)

    @classmethod
    def UnitX(cls) -> Vector:
        return cls(1, 0, 0)

    @classmethod
    def UnitY(cls) -> Vector:
        return cls(0, 1, 0)

    @classmethod
    def UnitZ(cls) -> Vector:
        return cls(0, 0, 1)

    def norm(self) -> Symbol:
        """Return the norm."""
        return sympy.sqrt(self.x**2 + self.y**2 + self.z**2)

    def normalize(self) -> Vector:
        """Normalize."""
        return self / self.norm()

    def cross_mat(self) -> Matrix:
        """Return the matrix corresponding to the vector cross product."""
        return Matrix([[0, -self.z, self.y], [self.z, 0, -self.x], [-self.y, self.x, 0]])

    def dot(self, other: Vector) -> Symbol:
        """Compute the dot product of two `Vector`s."""
        return self.x * other.x + self.y * other.y + self.z * other.z

    def argument(self, other: Vector) -> Symbol:
        """Compute the angle between two `Vector`s."""
        v1 = self.normalize()
        v2 = other.normalize()
        return sympy.acos(v1.dot(v2))

    def is_parallel(self, other: Vector, tol: float = 1e-6) -> bool:
        """Return `True` if this vector is always parallel to the other vector in the same direction."""
        # Compute the angle.
        angle = self.argument(other)

        # Return `False` when the angle cannot be evaluated
        # because it cannot be converted to `float`, for example when it is a `Symbol`.
        try:
            angle = float(angle)
        except Exception as e:
            print(f"Failed to evaluate the argument of 2 vectors: {e}")
            return False

        return angle < tol

    def is_parallel_legacy(self, other: Vector, same_direction_only: bool = False) -> True:
        """Return `True` if this vector is always parallel to the other vector.

        This method cannot set a tolerance.
        """
        # Define the proportional coefficient.
        k = sympy.symbols("k")

        # Conditions for two vectors to be parallel.
        # Add conditions with `k` on both the left-hand side and right-hand side
        # to prevent conditions from disappearing due to multiplication or division by 0.
        eq_x1 = sympy.Eq(other.x, self.x * k)
        eq_y1 = sympy.Eq(other.y, self.y * k)
        eq_z1 = sympy.Eq(other.z, self.z * k)
        eq_x2 = sympy.Eq(other.x / k, self.x)
        eq_y2 = sympy.Eq(other.y / k, self.y)
        eq_z2 = sympy.Eq(other.z / k, self.z)

        # Solve equations only for the proportional coefficient.
        sol: Dict[Symbol, Symbol] = sympy.solve((eq_x1, eq_y1, eq_z1, eq_x2, eq_y2, eq_z2), (k), dict=False)

        if len(sol) == 0:
            # Return `False` if there is no solution.
            return False
        elif len(sol) == 1:
            # Return `True` only when the proportional coefficient is always positive.
            return isinstance(sol[k] > 0, BooleanTrue) if same_direction_only else True
        else:
            # Raise an exception if there are multiple solutions.
            raise RuntimeError("Equation has multiple solutions.")

    def simplify(self, chop=False) -> Vector:
        return Vector(
            sympy.simplify(self.x, chop=chop),
            sympy.simplify(self.y, chop=chop),
            sympy.simplify(self.z, chop=chop),
        )

    def __add__(self, rhs: Vector) -> Vector:
        x = self.x + rhs.x
        y = self.y + rhs.y
        z = self.z + rhs.z
        return Vector(x, y, z)

    def __sub__(self, rhs: Vector) -> Vector:
        x = self.x - rhs.x
        y = self.y - rhs.y
        z = self.z - rhs.z
        return Vector(x, y, z)

    def __truediv__(self, rhs: Symbol) -> Vector:
        x = self.x / rhs
        y = self.y / rhs
        z = self.z / rhs
        return Vector(x, y, z)

    def __repr__(self) -> str:
        return f"x: {self.x}, y: {self.y}, z: {self.z}"


class Rotation:
    def __init__(
        self,
        Xx: Symbol,
        Yx: Symbol,
        Zx: Symbol,
        Xy: Symbol,
        Yy: Symbol,
        Zy: Symbol,
        Xz: Symbol,
        Yz: Symbol,
        Zz: Symbol,
    ) -> None:
        self.data = Matrix([[Xx, Yx, Zx], [Xy, Yy, Zy], [Xz, Yz, Zz]])

    @classmethod
    def Identity(cls) -> Rotation:
        return cls(1, 0, 0, 0, 1, 0, 0, 0, 1)

    @classmethod
    def RotX(cls, roll: Symbol) -> Rotation:
        cs = sympy.cos(roll)
        sn = sympy.sin(roll)
        return cls(1, 0, 0, 0, cs, -sn, 0, sn, cs)

    @classmethod
    def RotY(cls, pitch: Symbol) -> Rotation:
        cs = sympy.cos(pitch)
        sn = sympy.sin(pitch)
        return cls(cs, 0, sn, 0, 1, 0, -sn, 0, cs)

    @classmethod
    def RotZ(cls, yaw: Symbol) -> Rotation:
        cs = sympy.cos(yaw)
        sn = sympy.sin(yaw)
        return cls(cs, -sn, 0, sn, cs, 0, 0, 0, 1)

    @classmethod
    def RPY(cls, roll: Symbol, pitch: Symbol, yaw: Symbol) -> Rotation:
        return cls.RotZ(yaw) * cls.RotY(pitch) * cls.RotX(roll)

    @classmethod
    def Rodrigues(cls, axis: Vector, angle: Symbol) -> Rotation:
        """
        Compute a rotation matrix from a rotation axis and rotation angle.

        Using Robotics (2.80) is faster,
        but this implementation follows Rodrigues' formula directly.

        Parameters
        ----------
        axis: Vector
            Rotation axis.
        angle: Symbol
            Rotation angle [rad].

        Returns
        ----------
        Rotation
        """
        sn = sympy.sin(angle)
        cs = sympy.cos(angle)
        cross = axis.normalize().cross_mat()
        I = np.identity(3)  # With `sympy.Identity(3)`, the following matrix sum stayed as `MatAdd`.

        data: Matrix = I + sn * cross + (1 - cs) * (cross @ cross)
        return cls(*list(data))

    def inverse(self) -> Rotation:
        data: Matrix = self.data.T
        return Rotation(*list(data))

    def get_rpy(self) -> Tuple[Symbol, Symbol, Symbol]:
        epsilon = 1e-12
        pitch = sympy.atan2(-self.data[2, 0], sympy.sqrt(self.data[0, 0] ** 2 + self.data[1, 0] ** 2))
        if type(pitch) is float and abs(pitch) > math.pi / 2 - epsilon:
            roll = 0.0
            yaw = sympy.atan2(-self.data[0, 1], self.data[1, 1])
        else:
            roll = sympy.atan2(self.data[2, 1], self.data[2, 2])
            yaw = sympy.atan2(self.data[1, 0], self.data[0, 1])
        return roll, pitch, yaw

    @singledispatchmethod
    def __mul__(self, rhs: Rotation) -> Rotation:
        data: Matrix = self.data @ rhs.data
        return Rotation(*list(data))

    @__mul__.register
    def _(self, rhs: Vector) -> Vector:
        data: Matrix = self.data @ Matrix([rhs.x, rhs.y, rhs.z])
        return Vector(*data)

    def __repr__(self) -> str:
        res = ""
        for r in range(3):
            for c in range(3):
                res += f"{self.data[r, c]}\t"
            res += "\n"
        return res


class Frame:
    """
    Represents translation and rotation in 3D space.

    Note that the order is always translation -> rotation.
    """

    def __init__(self, p: Vector, M: Rotation) -> None:
        self.p = p
        self.M = M

    @classmethod
    def Identity(cls) -> Frame:
        return cls(Vector.Zero(), Rotation.Identity())

    @classmethod
    def Trans(cls, p: Vector) -> None:
        return cls(p, Rotation.Identity())

    @classmethod
    def Rot(cls, M: Rotation) -> None:
        return cls(Vector.Zero(), M)

    @classmethod
    def TransX(cls, x: Symbol) -> Frame:
        return cls.Trans(Vector.TransX(x))

    @classmethod
    def TransY(cls, y: Symbol) -> Frame:
        return cls.Trans(Vector.TransY(y))

    @classmethod
    def TransZ(cls, z: Symbol) -> Frame:
        return cls.Trans(Vector.TransZ(z))

    @classmethod
    def RotX(cls, roll: Symbol) -> Frame:
        return cls.Rot(Rotation.RotX(roll))

    @classmethod
    def RotY(cls, pitch: Symbol) -> Frame:
        return cls.Rot(Rotation.RotY(pitch))

    @classmethod
    def RotZ(cls, yaw: Symbol) -> Frame:
        return cls.Rot(Rotation.RotZ(yaw))

    @classmethod
    def DH(cls, alpha: Symbol, a: Symbol, theta: Symbol, d: Symbol) -> Frame:
        """
        Coordinate transformation using Denavit-Hartenberg parameters.

        Parameters
        ----------
        alpha: Symbol
            Angle between the two axes [rad].
        a: Symbol
            Normal distance between the two axes.
        theta: Symbol
            Angle between the two normals [rad].
        d: Symbol
            Distance between the two normals.

        Returns
        -------
        Frame
        """

        cs_alpha = sympy.cos(alpha)
        sn_alpha = sympy.sin(alpha)
        cs_theta = sympy.cos(theta)
        sn_theta = sympy.sin(theta)

        # Robotics (3.6)
        p = Matrix([a, -sn_alpha * d, cs_alpha * d])
        M = Matrix(
            [
                [cs_theta, -sn_theta, 0],
                [sn_theta * cs_alpha, cs_theta * cs_alpha, -sn_alpha],
                [sn_theta * sn_alpha, cs_theta * sn_alpha, cs_alpha],
            ]
        )

        return cls(p, M)

    def inverse(self) -> Frame:
        M_inv = self.M.inverse()
        return Frame(p=-M_inv @ self.p, M=M_inv)

    @singledispatchmethod
    def __mul__(self, rhs: Frame) -> Frame:
        p = self.M * rhs.p + self.p
        M = self.M * rhs.M
        return Frame(p=p, M=M)

    @__mul__.register
    def _(self, rhs: Vector) -> Vector:
        return self.M * rhs + self.p

    def __repr__(self) -> str:
        return f"p:\n{self.p.__repr__()}\n" + f"M:\n{self.M.__repr__()}\n"
