#!/usr/bin/env python3

# Copyright 2025 Universidad Politécnica de Madrid
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
"""Casadi UAV Model Dynamics definition."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'


import casadi as ca
from mpcc.utils import quaternion_utils as q_utils


class CaModelActuation:
    """CasADi model actuation dynamics definition."""

    @staticmethod
    def get_motor_angular_velocity_derivate(
            motor_angular_velocity_reference: ca.SX,
            motor_angular_velocity_state: ca.SX,
            tau: ca.SX) -> ca.SX:
        """
        Get the motor angular velocity derivative.

        :param motor_angular_velocity_reference: The motor angular velocity reference (rad/s).
        :type motor_angular_velocity_reference: ca.SX
        :param motor_angular_velocity_state: The motor angular velocity state (rad/s).
        :type motor_angular_velocity_state: ca.SX
        :param tau: The time constant (s).
        :type tau: ca.SX
        :return: The motor angular velocity derivative (rad/s^2).
        :rtype: ca.SX
        """
        return (motor_angular_velocity_reference - motor_angular_velocity_state) / tau

    @staticmethod
    def clamp_motor_angular_velocity(
            motor_angular_velocity: ca.SX,
            motor_angular_velocity_min: ca.SX,
            motor_angular_velocity_max: ca.SX) -> ca.SX:
        """
        Clamp the motor angular velocity.

        :param motor_angular_velocity: The motor angular velocity (rad/s).
        :type motor_angular_velocity: ca.SX
        :param motor_angular_velocity_min: The minimum motor angular velocity (rad/s).
        :type motor_angular_velocity_min: ca.SX
        :param motor_angular_velocity_max: The maximum motor angular velocity (rad/s).
        :type motor_angular_velocity_max: ca.SX
        :return: The clamped motor angular velocity (rad/s).
        :rtype: ca.SX
        """
        return ca.fmax(motor_angular_velocity_min,
                       ca.fmin(motor_angular_velocity, motor_angular_velocity_max))


class CaModelDynamics:
    """CasADi model dynamics definition."""

    @staticmethod
    def get_thrust_force(motor_angular_velocity: ca.SX, cf: ca.SX, orientation: ca.SX) -> ca.SX:
        """
        Get the thrust force.

        :param motor_angular_velocity: The motor angular velocity (rad/s).
        :type motor_angular_velocity: ca.SX
        :param cf: The motor force constant (N/rad^2).
        :type cf: ca.SX
        :return: The thrust force (N) in world frame.
        :rtype: ca.SX
        """
        thrust_force_b = ca.vertcat(0, 0, ca.sum1(cf * motor_angular_velocity**2))
        return q_utils.apply_rotation(orientation, thrust_force_b)

    @staticmethod
    def get_gravity_force(mass: ca.SX, gravity: ca.SX) -> ca.SX:
        """
        Get the gravity force.

        :param mass: The mass (kg).
        :type mass: ca.SX
        :param gravity: The gravity (m/s^2) (positive in -z direction).
        :type gravity: ca.SX
        :return: The gravity force (N) in world frame.
        :rtype: ca.SX
        """
        gravity_force = ca.vertcat(0, 0, -mass * gravity)
        return gravity_force

    @staticmethod
    def get_drag_force(linear_velocity: ca.SX, drag_coefficient: ca.SX) -> ca.SX:
        """
        Get the drag force.

        :param linear_velocity: The linear velocity (m/s) in world frame.
        :type linear_velocity: ca.SX
        :param drag_coefficient: The drag coefficient:
        [cdx1, cdy1, cdz1, cdx3, cdy3, cdz3]
        where:
        - cdx1, cdy1, cdz1: Linear drag coefficient.
        - cdx3, cdy3, cdz3: Cubic drag coefficient.
        :type drag_coefficient: ca.SX

        :return: The drag force (N) in world frame.
        :rtype: ca.SX
        """
        drag_force = ca.vertcat(
            -drag_coefficient[0] * linear_velocity[0] +
            drag_coefficient[3] * linear_velocity[0]**3,
            -drag_coefficient[1] * linear_velocity[1] +
            drag_coefficient[4] * linear_velocity[1]**3,
            -drag_coefficient[2] * linear_velocity[2] +
            drag_coefficient[5] * linear_velocity[2]**3)
        return drag_force

    @staticmethod
    def get_stocastic_force(sigma: ca.SX, time_step: ca.SX) -> ca.DM:
        """
        Get the stocastic force.

        :param sigma: Normal distribution standard deviation (N).
        :type sigma: ca.DM
        :param time_step: The time step (s).
        :type time_step: float
        :return: The stocastic force (N) in body frame.
        :rtype: ca.DM
        """
        # Casadi not support random number generation. Just use with casadi dm
        return ca.sqrt(sigma / time_step) * ca.DM.rand(3, 1)

    @staticmethod
    def get_force(
            thrust_force: ca.SX,
            gravity_force: ca.SX,
            external_foce: ca.SX) -> ca.SX:
        """
        Get the force.

        :param thrust_force: The thrust force (N) in world frame.
        :type thrust_force: ca.SX
        :param gravity_force: The gravity force (N) in world frame.
        :type gravity_force: ca.SX
        :param external_foce: The external force (N) in world frame.
        :type external_foce: ca.SX
        :return: The total force (N) in world frame.
        :rtype: ca.SX
        """
        return thrust_force + gravity_force + external_foce

    @staticmethod
    def get_external_force_world_frame(orientation: ca.SX, external_force: ca.SX) -> ca.SX:
        """
        Get the external force in world frame.

        :param orientation: The orientation of base frame in world frame.
        :type orientation: ca.SX
        :param external_force: The external force (N) in body frame.
        :type external_force: ca.SX
        :return: The external force (N) in world frame.
        :rtype: ca.SX
        """
        return q_utils.apply_rotation(orientation, external_force)

    @staticmethod
    def get_thrust_torque(
            angular_velocity: ca.SX, cf: ca.SX, ct: ca.SX, dx: ca.SX, dy: ca.SX,
            motors_direction: ca.SX) -> ca.SX:
        """
        Get the motor torque.

        :param angular_velocity: The angular velocity of each motor (rad/s).
        :type angular_velocity: ca.SX
        :param cf: The motor force constant (N/rad^2).
        :type cf: ca.SX
        :param ct: The motor torque constant (N*m/rad^2).
        :type ct: ca.SX
        :param dx: The distance from the center of mass to the motor in x-axis (m).
        :type dx: ca.SX
        :param dy: The distance from the center of mass to the motor in y-axis (m).
        :type dy: ca.SX
        :param motors_direction: The motor direction (1 clockwise, -1 counterclockwise).
        :type motors_direction: ca.SX
        :return: The collective torque (N*m) in body frame.
        :rtype: ca.SX
        """
        # torque = ca.vertcat(torque_x, torque_y, torque_z)
        # Compute angular velocity squared for each motor: w[i] = w[i]^2
        angular_velocity_squared = angular_velocity**2

        # Compute the thrust force for each rotor: thrust[i] = cf[i] * w[i]^2
        thrust = cf * angular_velocity_squared

        # Torque around the x-axis: torque_x[i] = dy[i] * cf[i] * w[i]^2
        torque_x = dy * thrust

        # Torque around the y-axis: torque_y[i] = -dx[i] * cf[i] * w[i]^2
        torque_y = -dx * thrust

        # Torque around the z-axis: torque_z[i] = motors_direction[i] * ct[i] * w[i]^2
        torque_z = motors_direction * ct * angular_velocity_squared

        # Combine the torques into a single vector
        torque = ca.vertcat(ca.sum1(torque_x), ca.sum1(torque_y), ca.sum1(torque_z))
        return torque

    @staticmethod
    def get_drag_torque(angular_velocity: ca.SX, drag_coefficient: ca.SX) -> ca.SX:
        """
        Get the drag force.

        :param angular_velocity: The angular velocity (rad/s) in body frame.
        :type angular_velocity: ca.SX
        :param drag_coefficient: The drag coefficient:
        [cdx1, cdy1, cdz1, cdx3, cdy3, cdz3]
        where:
        - cdx1, cdy1, cdz1: Linear drag coefficient.
        - cdx3, cdy3, cdz3: Cubic drag coefficient.
        :type drag_coefficient: ca.SX

        :return: The drag torque (N*m) in body frame.
        :rtype: ca.SX
        """
        drag_force = ca.vertcat(
            -drag_coefficient[0] * angular_velocity[0] +
            drag_coefficient[3] * angular_velocity[0]**3,
            -drag_coefficient[1] * angular_velocity[1] +
            drag_coefficient[4] * angular_velocity[1]**3,
            -drag_coefficient[2] * angular_velocity[2] +
            drag_coefficient[5] * angular_velocity[2]**3)
        return drag_force

    @staticmethod
    def get_stocastic_torque(sigma: ca.DM, time_step: float) -> ca.DM:
        """
        Get the stocastic torque.

        :param sigma: Normal distribution standard deviation (N*m).
        :type sigma: ca.DM
        :param time_step: The time step (s).
        :type time_step: float
        :return: The stocastic torque (N*m) in body frame.
        :rtype: ca.DM
        """
        # Casadi not support random number generation. Just use with casadi dm
        return ca.sqrt(sigma / time_step) * ca.DM.rand(3, 1)

    @staticmethod
    def get_torque(thrust_torque: ca.SX, drag_torque: ca.SX, stocastic_torque: ca.SX,
                   external_torque: ca.SX) -> ca.SX:
        """
        Get the torque.

        :param thrust_torque: The thrust torque (N*m) in body frame.
        :type thrust_torque: ca.SX
        :param drag_torque: The drag torque (N*m) in body frame.
        :type drag_torque: ca.SX
        :param stocastic_torque: The stocastic torque (N*m) in body frame.
        :type stocastic_torque: ca.SX
        :param external_torque: The external torque (N*m) in body frame.
        :type external_torque: ca.SX
        :return: The total torque (N*m) in body frame.
        :rtype: ca.SX
        """
        return thrust_torque + drag_torque + stocastic_torque + external_torque

    @staticmethod
    def get_external_torque_body_frame(orientation: ca.SX, external_torque: ca.SX) -> ca.SX:
        """
        Get the external torque in body frame.

        :param orientation: The orientation of base frame in world frame.
        :type orientation: ca.SX
        :param external_torque: The external torque (N*m) in world frame.
        :type external_torque: ca.SX
        :return: The external torque (N*m) in body frame.
        :rtype: ca.SX
        """
        return q_utils.apply_inverse_rotation(orientation, external_torque)


class CaModelKinematics:
    """CasADi model kinematcis definition."""

    @staticmethod
    def get_position_derivate(lineal_velocity: ca.SX) -> ca.SX:
        """
        Get the position derivate.

        :param lineal_velocity: The lineal velocity (m/s) in world frame.
        :type lineal_velocity: ca.SX
        :return: The position derivate (m/s) in world frame.
        :rtype: ca.SX
        """
        return lineal_velocity

    @staticmethod
    def get_orientation_derivate(
            orientation: ca.SX,
            angular_velocity: ca.SX) -> ca.SX:
        """
        Get the orientation derivate.

        :param orientation: The orientation of base frame in world frame.
        :type orientation: ca.SX
        :param angular_velocity: The angular velocity in body frame.
        :type angular_velocity: ca.SX
        :return: The orientation derivate (quaternion).
        :rtype: ca.SX
        """
        angular_velocity = q_utils.quaternion_derivate(orientation, angular_velocity)
        return angular_velocity

    @staticmethod
    def get_lineal_velocity_derivate(
            force: ca.SX,
            mass: ca.SX) -> ca.SX:
        """
        Get the lineal velocity derivate.

        :param force: The force (N) in world frame.
        :type force: ca.SX
        :param mass: The mass (kg).
        :type mass: ca.SX
        :return: The lineal velocity derivate (m/s^2) in world
        :rtype: ca.SX
            frame.
        """
        acceleration = force / mass
        return acceleration

    @staticmethod
    def get_angular_velocity_derivate(
            torque: ca.SX,
            vehicle_angular_velocity: ca.SX,
            inertia: ca.SX) -> ca.SX:
        """
        Get the angular velocity derivate.

        :param torque: Sum of torque (N*m) in body frame.
        :type torque: ca.SX
        :param vehicle_angular_velocity: The angular velocity (rad/s) in body frame.
        :type vehicle_angular_velocity: ca.SX
        :param inertia: The inertia matrix (kg*m^2) in body frame.
        :type inertia: ca.SX
        """
        # Angular acceleration in body frame
        angular_acceleration =\
            ca.mtimes(
                ca.inv(inertia),
                (torque -
                 ca.cross(vehicle_angular_velocity, ca.mtimes(inertia, vehicle_angular_velocity))))
        return angular_acceleration
