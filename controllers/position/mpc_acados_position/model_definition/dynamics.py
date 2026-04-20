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
"""Casadi MAV Model datatype Dynamics."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

# THIS CODE HAS BEEN AUTOMATICALLY GENERATED USING templ_model_definition.py.j2

from typing import ClassVar, List, Union

from mpc_acados_core.utils.datatypes_utils import VectorBase
import casadi as ca
import numpy as np


class _DynamicsDef:
    """Dynamics definition for MAV."""

    _names: ClassVar[List[str]] = [
        'position_dot',
        'orientation_dot',
        'linear_velocity_dot'
    ]
    _names_sx: ClassVar[List[str]] = [
        'sx_position_dot',
        'sx_orientation_dot',
        'sx_linear_velocity_dot'
    ]
    _sizes: ClassVar[List[int]] = [
        3,
        4,
        3
    ]

    position_dot: Union[ca.SX, ca.DM]
    orientation_dot: Union[ca.SX, ca.DM]
    linear_velocity_dot: Union[ca.SX, ca.DM]


class CaDynamics(_DynamicsDef, VectorBase):
    """
    CasADi SX Dynamics for the UAV.

    :param position_dot: Time derivative of position in world frame [vx, vy, vz] (m/s)
    :type position_dot: ca.SX
    :param orientation_dot: Time derivative of orientation as a quaternion [dqw, dqx, dqy, dqz]
    :type orientation_dot: ca.SX
    :param linear_velocity_dot: Time derivative of linear velocity in world frame [ax, ay, az] (m/s^2)
    :type linear_velocity_dot: ca.SX
    """

    _type = 'ca.SX'


class Dynamics(_DynamicsDef, VectorBase):
    """Dynamics for the UAV Model."""

    _type = 'np.array'

    def __init__(
            self,
            position_dot: np.array = np.array([0.0, 0.0, 0.0]),
            orientation_dot: np.array = np.array([0.0, 0.0, 0.0, 0.0]),
            linear_velocity_dot: np.array = np.array([0.0, 0.0, 0.0])):
        """
        Dynamics for the UAV Model.

        :param position_dot: Time derivative of position in world frame [vx, vy, vz] (m/s)
        :type position_dot: np.array
        :param orientation_dot: Time derivative of orientation as a quaternion [dqw, dqx, dqy, dqz]
        :type orientation_dot: np.array
        :param linear_velocity_dot: Time derivative of linear velocity in world frame [ax, ay, az] (m/s^2)
        :type linear_velocity_dot: np.array
        """
        super().__init__()
        self.position_dot = position_dot
        self.orientation_dot = orientation_dot
        self.linear_velocity_dot = linear_velocity_dot


if __name__ == '__main__':
    # Test the Dynamics class
    dynamics = Dynamics()
    print('CasADi SX:', dynamics._names)
    print('CasADi SX Sizes:', dynamics._sizes)
    print('Vector:', dynamics.vector)
    print('position_dot:', dynamics.position_dot)
    print('orientation_dot:', dynamics.orientation_dot)
    print('linear_velocity_dot:', dynamics.linear_velocity_dot)
    print(dynamics)
