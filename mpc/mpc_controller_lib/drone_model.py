#!/usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Acados Model definition."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from acados_template import AcadosModel
import casadi as ca
import numpy as np


class CaBase():
    """Base class for CasADi structures (State or Control)."""

    def __init__(self, names_shapes):
        """
        Initialize a CasADi structure with symbolic variables and index mappings.

        :param names_shapes (list): List of tuples containing (name, shape).
        """
        self._params_index = {}
        self._vector = []

        current_index = 0
        for name, shape in names_shapes:
            symbol = ca.SX.sym(f'{name}', *shape)
            self._params_index[name] = slice(current_index, current_index + shape[0])
            self._vector.append(symbol)
            current_index += shape[0]

        self._vector = ca.vertcat(*self._vector)

    @property
    def vector(self):
        """
        Get the CasADi vector.

        :return (ca.SX): The CasADi vector.
        """
        return self._vector

    @vector.setter
    def vector(self, value):
        """
        Set the CasADi vector.

        :param value (ca.SX): The vector to set.
        """
        self._vector = value

    def get_variable(self, name):
        """
        Get a specific variable by name.

        :param name (str): Variable name.
        :return (ca.SX): The requested variable.
        """
        return self._vector[self._params_index[name]]

    def set_variable(self, name, value):
        """
        Set a specific variable by name and ensure the shape is correct.

        :param name (str): Variable name.
        :param value (ca.SX): Value to set.
        """
        expected_shape = self._params_index[name].stop - self._params_index[name].start
        if value.shape[0] != expected_shape:
            raise ValueError(f'{name} must have shape ({expected_shape}, 1)')
        self._vector[self._params_index[name]] = value

    def __str__(self):
        return f'{self._vector}'


class CaState(CaBase):
    """CasADi state."""

    def __init__(self):
        """Multirotor MPC State x = [position, orientation, linear_velocity]."""
        super().__init__([
            ('x_p', (3,)),  # position
            ('x_q', (4,)),  # orientation
            ('x_v', (3,))   # linear_velocity
        ])

    @staticmethod
    def get_state(
            position: np.array = np.zeros(3),
            orientation: np.array = np.array([1, 0, 0, 0]),
            linear_velocity: np.array = np.zeros(3)) -> np.array:
        """
        Get the initial state.

        :param position (np.array): The initial position.
        :param orientation (np.array): The initial orientation.
        :param linear_velocity (np.array): The initial linear velocity.

        :return (np.array): The initial state.
        """
        if orientation[0] == 0:
            raise ValueError('Quaternion malformed: w==0')
        return np.concatenate((position, orientation, linear_velocity))

    @staticmethod
    def get_cost_matrix(
            position_weight: np.array = np.ones(3),
            orientation_weight: np.array = np.ones(4),
            linear_velocity_weight: np.array = np.ones(3)) -> np.array:
        """
        Get the cost matrix Q.

        :param position_weight (np.array): The position weight.
        :param orientation_weight (np.array): The orientation weight.
        :param linear_velocity_weight (np.array): The linear velocity weight.

        :return (np.array): The cost matrix Q.
        """
        return np.diag(np.concatenate((
            position_weight, orientation_weight, linear_velocity_weight)))

    @property
    def position(self) -> ca.SX:
        """
        Get the position.

        :return (ca.SX): The position.
        """
        return self.get_variable('x_p')

    @position.setter
    def position(self, value: ca.SX):
        """
        Set the position.

        :param value (ca.SX): The position.
        """
        self.set_variable('x_p', value)

    @property
    def orientation(self) -> ca.SX:
        """
        Get the orientation.

        :return (ca.SX): The orientation.
        """
        return self.get_variable('x_q')

    @orientation.setter
    def orientation(self, value: ca.SX):
        """
        Set the orientation.

        :param value (ca.SX): The orientation.
        """
        self.set_variable('x_q', value)

    @property
    def linear_velocity(self) -> ca.SX:
        """
        Get the linear velocity.

        :return (ca.SX): The linear velocity.
        """
        return self.get_variable('x_v')

    @linear_velocity.setter
    def linear_velocity(self, value: ca.SX):
        """
        Set the linear velocity.

        :param value (ca.SX): The linear velocity.
        """
        self.set_variable('x_v', value)


class CaControl(CaBase):
    """CasADi control."""

    def __init__(self):
        """Multirotor control signals u = [thrust, angular_velocity]."""
        super().__init__([
            ('u_t', (1,)),  # thrust
            ('u_w', (3,))   # angular_velocity
        ])

    @staticmethod
    def get_control(
            thrust: float = 0.0,
            angular_velocity: np.array = np.zeros(3)) -> np.array:
        """
        Get the initial control.

        :param thrust (float): The initial thrust.
        :param angular_velocity (np.array): The initial angular velocity.

        :return (np.array): The initial control.
        """
        return np.concatenate((np.array([thrust]), angular_velocity))

    @staticmethod
    def get_cost_matrix(
            thrust_weight: np.array = np.ones(1),
            angular_velocity_weight: np.array = np.ones(3)) -> np.array:
        """
        Get the cost matrix R.

        :param thrust_weight (float): The thrust weight.
        :param angular_velocity_weight (np.array): The angular velocity weight.

        :return (np.array): The cost matrix R.
        """
        return np.diag(np.concatenate((thrust_weight, angular_velocity_weight)))

    @property
    def thrust(self) -> ca.SX:
        """
        Get the thrust.

        :return (ca.SX): The thrust.
        """
        return self.get_variable('u_t')

    @thrust.setter
    def thrust(self, value: ca.SX):
        """
        Set the thrust.

        :param value (ca.SX): The thrust.
        """
        self.set_variable('u_t', value)

    @property
    def angular_velocity(self) -> ca.SX:
        """
        Get the angular velocity.

        :return (ca.SX): The angular velocity.
        """
        return self.get_variable('u_w')

    @angular_velocity.setter
    def angular_velocity(self, value: ca.SX):
        """
        Set the angular velocity.

        :param value (ca.SX): The angular velocity.
        """
        self.set_variable('u_w', value)


class CaDynamics(CaBase):
    """CasADi dynamics."""

    def __init__(
            self,
            x: CaState = CaState(),
            u: CaControl = CaControl()) -> None:
        """
        Multirotor dynamics x_dot = f(x, u).

        :param x (CaState): Multirotor state.
        :param u (CaControl): Multirotor control signals.
        """
        super().__init__([
            ('x_p_dot', (3,)),  # position_dot
            ('x_q_dot', (4,)),  # orientation_dot
            ('x_v_dot', (3,))   # linear_velocity_dot
        ])
        self._x = x
        self._u = u
        self._p = []

        # Parameters
        mass = ca.SX.sym('mass')
        gravity = ca.DM(9.81)

        self._p = ca.vertcat(mass)

        # Model equations
        p_dot = x.linear_velocity
        q_dot = self.quaternion_derivate(x.orientation, u.angular_velocity)
        v_dot = self.velocity_derivate(
            x.orientation, u.thrust, gravity, mass)

        self._f_expl = ca.vertcat(p_dot, q_dot, v_dot)

    @staticmethod
    def quaternion_derivate(quaternion: ca.SX, angular_velocity: ca.SX) -> ca.SX:
        """
        Compute the quaternion derivative.

        q_dot = 0.5 * q x omega = 0.5 * [qw, qx, qy, qz] * [0, ,wx, wy, wz]

        :param quaternion (ca.SX): The quaternion [qw, qx, qy, qz].
        :param angular_velocity (ca.SX): The angular velocity [wx, wy, wz].

        :return (ca.SX): The quaternion derivative [qw_dot, qx_dot, qy_dot, qz_dot].
        """
        w_qx = angular_velocity[0]
        w_qy = angular_velocity[1]
        w_qz = angular_velocity[2]
        w_q = ca.vertcat(
            0.0,
            w_qx,
            w_qy,
            w_qz)

        return 0.5 * CaDynamics.quaternion_multiply(
            CaDynamics.normalize_quaternion(quaternion), w_q)

    @staticmethod
    def quaternion_multiply(q1: ca.SX, q2: ca.SX) -> ca.SX:
        """
        Multiply two quaternions.

        q = q1 x q2 = [qw1, qx1, qy1, qz1] x [qw2, qx2, qy2, qz2]

        :param q1 (ca.SX): The first quaternion [qw1, qx1, qy1, qz1].
        :param q2 (ca.SX): The second quaternion [qw2, qx2, qy2, qz2].

        :return (ca.SX): The resulting quaternion [qw, qx, qy, qz].
        """
        qw1 = q1[0]
        qx1 = q1[1]
        qy1 = q1[2]
        qz1 = q1[3]

        qw2 = q2[0]
        qx2 = q2[1]
        qy2 = q2[2]
        qz2 = q2[3]

        qw = qw1 * qw2 - qx1 * qx2 - qy1 * qy2 - qz1 * qz2
        qx = qw1 * qx2 + qx1 * qw2 + qy1 * qz2 - qz1 * qy2
        qy = qw1 * qy2 - qx1 * qz2 + qy1 * qw2 + qz1 * qx2
        qz = qw1 * qz2 + qx1 * qy2 - qy1 * qx2 + qz1 * qw2

        return ca.vertcat(qw, qx, qy, qz)

    @staticmethod
    def velocity_derivate(
            quaternion: ca.SX,
            thrust: ca.SX,
            gravity: ca.SX,
            mass: ca.SX) -> ca.SX:
        """
        Compute the linear velocity derivative.

        v_dot = 2 * q_v x t = 2 * [qw, qx, qy, qz] x t

        :param quaternion (ca.SX): The quaternion [qw, qx, qy, qz]
        rotation from world to body frame.
        :param thrust (ca.SX): The thrust (N) in body frame.
        :param gravity (ca.SX): The gravity (m/s^2) in world frame.
        :param mass (ca.SX): The mass (kg).

        :return (ca.SX): The linear velocity derivative [vx_dot, vy_dot, vz_dot]
        in world frame.
        """
        # Compute inverse rotation
        acceleration_body_frame = ca.vertcat(0, 0, thrust / mass)
        acceleration_world = CaDynamics.apply_rotation(
            CaDynamics.normalize_quaternion(quaternion),
            acceleration_body_frame)

        v_dot = acceleration_world - ca.vertcat(0, 0, gravity)

        return v_dot

    @staticmethod
    def apply_rotation(q: ca.SX, v: ca.SX) -> ca.SX:
        """
        Apply a rotation to a vector.

        v_rotated = q x v x q_conj

        :param q (ca.SX): The quaternion [qw, qx, qy, qz].
        :param v (ca.SX): The vector [vx, vy, vz].

        :return (ca.SX): The rotated vector [vx_rotated, vy_rotated, vz_rotated].
        """
        qw = q[0]
        qx = q[1]
        qy = q[2]
        qz = q[3]

        vx = v[0]
        vy = v[1]
        vz = v[2]

        q_conj = ca.vertcat(qw, -qx, -qy, -qz)

        v_rotated = CaDynamics.quaternion_multiply(
            CaDynamics.quaternion_multiply(q, ca.vertcat(0, vx, vy, vz)),
            q_conj)

        return ca.vertcat(v_rotated[1], v_rotated[2], v_rotated[3])

    @staticmethod
    def apply_inverse_rotation(q: ca.SX, v: ca.SX) -> ca.SX:
        """
        Apply the inverse rotation to a vector.

        v_rotated = q_conj x v x q

        :param q (ca.SX): The quaternion [qw, qx, qy, qz].
        :param v (ca.SX): The vector [vx, vy, vz].

        :return (ca.SX): The rotated vector [vx_rotated, vy_rotated, vz_rotated].
        """
        qw = q[0]
        qx = q[1]
        qy = q[2]
        qz = q[3]

        vx = v[0]
        vy = v[1]
        vz = v[2]

        q_conj = ca.vertcat(qw, -qx, -qy, -qz)

        v_rotated = CaDynamics.quaternion_multiply(
            CaDynamics.quaternion_multiply(q_conj, ca.vertcat(0, vx, vy, vz)),
            q)

        return ca.vertcat(v_rotated[1], v_rotated[2], v_rotated[3])

    @staticmethod
    def quaternion_inverse(q: np.array) -> np.array:
        """
        Calculate the inverse of a quaternion.

        q_inv = q_conjugate / q_norm^2

        :param q (np.array): The input quaternion [q_w, q_x, q_y, q_z].

        :return (np.array): The inverse quaternion.
        """
        q_conjugate = np.array([q[0], -q[1], -q[2], -q[3]])
        q_norm_sq = CaDynamics.normalize_quaternion(q) ** 2
        if q_norm_sq == 0:
            q_norm_sq = 1e-3
        return q_conjugate / q_norm_sq

    @staticmethod
    def quaternion_error(q_desired: ca.SX, q_current: ca.SX) -> ca.SX:
        """
        Compute the quaternion error.

        q_error = q_desired x q_current_conj

        q_norm = sqrt(q_error[0] ** 2 + q_error[3] ** 2 + 1e-3)

        v_error = [q_error[0] * q_error[1] - q_error[2] * q_error[3],
                     q_error[0] * q_error[2] + q_error[1] * q_error[3],
                     q_error[3]] / q_norm

        :param q_desired (ca.SX): The desired quaternion [qw, qx, qy, qz].
        :param q_current (ca.SX): The current quaternion [qw, qx, qy, qz].

        :return (ca.SX): The vector error [vx_error, vy_error, vz_error].
        """
        q_error = CaDynamics.quaternion_multiply(
            q_desired,
            ca.vertcat(q_current[0], -q_current[1], -q_current[2], -q_current[3]))

        # Project the error to the 3D space to get z-axis error
        q_norm = ca.sqrt(q_error[0] ** 2 + q_error[3] ** 2 + 1e-3)
        v_error = ca.vertcat(
            q_error[0] * q_error[1] - q_error[2] * q_error[3],
            q_error[0] * q_error[2] + q_error[1] * q_error[3],
            q_error[3]) / q_norm
        return v_error
    
    @staticmethod
    def normalize_quaternion(q: ca.SX) -> ca.SX:
        """
        Normalize a quaternion.

        :param q (ca.SX): The quaternion to normalize.

        :return (ca.SX): The normalized quaternion.
        """
        q_norm = ca.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2)
        return q / q_norm

    @property
    def f_expl(self) -> ca.SX:
        """
        Get the explicit dynamics f(x, u).

        :return (ca.SX): The explicit dynamics.
        """
        return self._f_expl

    @property
    def xdot(self) -> ca.SX:
        """
        Get the state derivative x_dot.

        :return (ca.SX): The state derivative x_dot.
        """
        return self.vector

    @property
    def x(self) -> ca.SX:
        """
        Get the state x.

        :return (ca.SX): The state x.
        """
        return self._x.vector

    @property
    def state(self) -> ca.SX:
        """
        Get the state.

        :return (CaState): The state.
        """
        return self._x

    @property
    def u(self) -> ca.SX:
        """
        Get the control u.

        :return (ca.SX): The control u.
        """
        return self._u.vector

    @property
    def control(self) -> ca.SX:
        """
        Get the control.

        :return (CaControl): The control.
        """
        return self._u

    @property
    def p(self) -> ca.SX:
        """
        Get the parameters.

        :return (ca.SX): The parameters.
        """
        return self._p


def get_acados_model() -> AcadosModel:
    """Initialize the Acados multirotor model."""
    model_name = 'mpc'

    # System Dynamics
    dynamics = CaDynamics()

    # Explicit and Implicit functions
    f_expl = dynamics.f_expl
    f_impl = dynamics.xdot - dynamics.f_expl

    # Algebraic variables
    z = []

    # Dynamics
    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = dynamics.x
    model.xdot = dynamics.xdot
    model.u = dynamics.u
    model.z = z
    model.p = dynamics.p
    model.name = model_name

    return model


if __name__ == '__main__':
    # Create a state
    x = CaState()
    print('State:    ', x)

    # Create a control
    u = CaControl()
    print('Control:  ', u)

    # Create dynamics
    dynamics = CaDynamics(x, u)
    print('Dynamics: ', dynamics)

    # Create an Acados model
    acados_model = get_acados_model()
    
    def format_output(label, expr):
        print(f'{label}:')
        terms = ca.vertsplit(expr, 1)
        for term in terms:
            print(f'    {term}')
        print()

    format_output('f_impl', acados_model.f_impl_expr)
    format_output('f_expl', acados_model.f_expl_expr)
