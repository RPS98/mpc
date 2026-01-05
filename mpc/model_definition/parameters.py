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
"""Casadi MAV Model datatype Parameters."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

# THIS CODE HAS BEEN AUTOMATICALLY GENERATED USING templ_model_definition.py.j2

from typing import ClassVar, List, Union

from mpc.utils.datatypes_utils import VectorBase
import casadi as ca
import numpy as np


class _ParametersDef:
    """Parameters definition for MAV."""

    _names: ClassVar[List[str]] = [
        'mass',
        'desired_orientation',
        'inertia',
        'motors_dx',
        'motors_dy',
        'motors_cf',
        'motors_ct',
        'motors_tau',
        'motors_direction',
        'motors_min_angular_velocity',
        'motors_max_angular_velocity'
    ]
    _names_sx: ClassVar[List[str]] = [
        'sx_mass',
        'sx_desired_orientation',
        'sx_inertia',
        'sx_motors_dx',
        'sx_motors_dy',
        'sx_motors_cf',
        'sx_motors_ct',
        'sx_motors_tau',
        'sx_motors_direction',
        'sx_motors_min_angular_velocity',
        'sx_motors_max_angular_velocity'
    ]
    _sizes: ClassVar[List[int]] = [
        1,
        4,
        3,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4
    ]

    mass: Union[ca.SX, ca.DM]
    desired_orientation: Union[ca.SX, ca.DM]
    inertia: Union[ca.SX, ca.DM]
    motors_dx: Union[ca.SX, ca.DM]
    motors_dy: Union[ca.SX, ca.DM]
    motors_cf: Union[ca.SX, ca.DM]
    motors_ct: Union[ca.SX, ca.DM]
    motors_tau: Union[ca.SX, ca.DM]
    motors_direction: Union[ca.SX, ca.DM]
    motors_min_angular_velocity: Union[ca.SX, ca.DM]
    motors_max_angular_velocity: Union[ca.SX, ca.DM]


class CaParameters(_ParametersDef, VectorBase):
    """
    CasADi SX Parameters for the UAV.

    :param mass: Mass of the MAV (kg)
    :type mass: ca.SX
    :param desired_orientation: Desired orientation as a quaternion [qw, qx, qy, qz]
    :type desired_orientation: ca.SX
    :param inertia: Inertia matrix diagonal elements [Ixx, Iyy, Izz] (kg*m^2)
    :type inertia: ca.SX
    :param motors_dx: Distance from the center of mass to the motors in x direction (m)
    :type motors_dx: ca.SX
    :param motors_dy: Distance from the center of mass to the motors in y direction (m)
    :type motors_dy: ca.SX
    :param motors_cf: Thrust coefficient for each motor (N/(rad/s)^2)
    :type motors_cf: ca.SX
    :param motors_ct: Torque coefficient for each motor (N*m/(rad/s)^2)
    :type motors_ct: ca.SX
    :param motors_tau: Time constant for each motor (s)
    :type motors_tau: ca.SX
    :param motors_direction: Direction of each motor (1 for clockwise, -1 for counter-clockwise)
    :type motors_direction: ca.SX
    :param motors_min_angular_velocity: Minimum angular velocity for each motor (rad/s)
    :type motors_min_angular_velocity: ca.SX
    :param motors_max_angular_velocity: Maximum angular velocity for each motor (rad/s)
    :type motors_max_angular_velocity: ca.SX
    """

    _type = 'ca.SX'


class Parameters(_ParametersDef, VectorBase):
    """Parameters for the UAV Model."""

    _type = 'np.array'

    def __init__(
            self,
            mass: np.array = np.array(1.0),
            desired_orientation: np.array = np.array([1.0, 0.0, 0.0, 0.0]),
            inertia: np.array = np.array([0.0049, 0.0049, 0.0069]),
            motors_dx: np.array = np.array([-0.08, 0.08, -0.08, 0.08]),
            motors_dy: np.array = np.array([-0.08, -0.08, 0.08, 0.08]),
            motors_cf: np.array = np.array([1.91e-06, 1.91e-06, 1.91e-06, 1.91e-06]),
            motors_ct: np.array = np.array([2.6e-07, 2.6e-07, 2.6e-07, 2.6e-07]),
            motors_tau: np.array = np.array([0.02, 0.02, 0.02, 0.02]),
            motors_direction: np.array = np.array([-1, 1, 1, -1]),
            motors_min_angular_velocity: np.array = np.array([0.0, 0.0, 0.0, 0.0]),
            motors_max_angular_velocity: np.array = np.array([2200.0, 2200.0, 2200.0, 2200.0])):
        """
        Parameters for the UAV Model.

        :param mass: Mass of the MAV (kg)
        :type mass: np.array
        :param desired_orientation: Desired orientation as a quaternion [qw, qx, qy, qz]
        :type desired_orientation: np.array
        :param inertia: Inertia matrix diagonal elements [Ixx, Iyy, Izz] (kg*m^2)
        :type inertia: np.array
        :param motors_dx: Distance from the center of mass to the motors in x direction (m)
        :type motors_dx: np.array
        :param motors_dy: Distance from the center of mass to the motors in y direction (m)
        :type motors_dy: np.array
        :param motors_cf: Thrust coefficient for each motor (N/(rad/s)^2)
        :type motors_cf: np.array
        :param motors_ct: Torque coefficient for each motor (N*m/(rad/s)^2)
        :type motors_ct: np.array
        :param motors_tau: Time constant for each motor (s)
        :type motors_tau: np.array
        :param motors_direction: Direction of each motor (1 for clockwise, -1 for counter-clockwise)
        :type motors_direction: np.array
        :param motors_min_angular_velocity: Minimum angular velocity for each motor (rad/s)
        :type motors_min_angular_velocity: np.array
        :param motors_max_angular_velocity: Maximum angular velocity for each motor (rad/s)
        :type motors_max_angular_velocity: np.array
        """
        super().__init__()
        self.mass = mass
        self.desired_orientation = desired_orientation
        self.inertia = inertia
        self.motors_dx = motors_dx
        self.motors_dy = motors_dy
        self.motors_cf = motors_cf
        self.motors_ct = motors_ct
        self.motors_tau = motors_tau
        self.motors_direction = motors_direction
        self.motors_min_angular_velocity = motors_min_angular_velocity
        self.motors_max_angular_velocity = motors_max_angular_velocity


if __name__ == '__main__':
    # Test the Parameters class
    parameters = Parameters()
    print('CasADi SX:', parameters._names)
    print('CasADi SX Sizes:', parameters._sizes)
    print('Vector:', parameters.vector)
    print('mass:', parameters.mass)
    print('desired_orientation:', parameters.desired_orientation)
    print('inertia:', parameters.inertia)
    print('motors_dx:', parameters.motors_dx)
    print('motors_dy:', parameters.motors_dy)
    print('motors_cf:', parameters.motors_cf)
    print('motors_ct:', parameters.motors_ct)
    print('motors_tau:', parameters.motors_tau)
    print('motors_direction:', parameters.motors_direction)
    print('motors_min_angular_velocity:', parameters.motors_min_angular_velocity)
    print('motors_max_angular_velocity:', parameters.motors_max_angular_velocity)
    print(parameters)
