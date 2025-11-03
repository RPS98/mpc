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
"""Casadi MAV Model datatype State."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

# THIS CODE HAS BEEN AUTOMATICALLY GENERATED USING templ_model_definition.py.j2

from typing import ClassVar, List, Union

from mpc.utils.datatypes_utils import VectorBase
import casadi as ca
import numpy as np


class _StateDef:
    """State definition for MAV."""

    _names: ClassVar[List[str]] = [
        'position',
        'orientation',
        'linear_velocity'
    ]
    _names_sx: ClassVar[List[str]] = [
        'sx_position',
        'sx_orientation',
        'sx_linear_velocity'
    ]
    _sizes: ClassVar[List[int]] = [
        3,
        4,
        3
    ]

    position: Union[ca.SX, ca.DM]
    orientation: Union[ca.SX, ca.DM]
    linear_velocity: Union[ca.SX, ca.DM]


class CaState(_StateDef, VectorBase):
    """
    CasADi SX State for the UAV.

    :param position: Position in world frame [x, y, z] (m)
    :type position: ca.SX
    :param orientation: Orientation of the body frame in world frame as a quaternion [qw, qx, qy, qz]
    :type orientation: ca.SX
    :param linear_velocity: Linear velocity in world frame [vx, vy, vz] (m/s)
    :type linear_velocity: ca.SX
    """

    _type = 'ca.SX'


class State(_StateDef, VectorBase):
    """State for the UAV Model."""

    _type = 'np.array'

    def __init__(
            self,
            position: np.array = np.array([0.0, 0.0, 0.0]),
            orientation: np.array = np.array([1.0, 0.0, 0.0, 0.0]),
            linear_velocity: np.array = np.array([0.0, 0.0, 0.0])):
        """
        State for the UAV Model.

        :param position: Position in world frame [x, y, z] (m)
        :type position: np.array
        :param orientation: Orientation of the body frame in world frame as a quaternion [qw, qx, qy, qz]
        :type orientation: np.array
        :param linear_velocity: Linear velocity in world frame [vx, vy, vz] (m/s)
        :type linear_velocity: np.array
        """
        super().__init__()
        self.position = position
        self.orientation = orientation
        self.linear_velocity = linear_velocity


if __name__ == '__main__':
    # Test the State class
    state = State()
    print('CasADi SX:', state._names)
    print('CasADi SX Sizes:', state._sizes)
    print('Vector:', state.vector)
    print('position:', state.position)
    print('orientation:', state.orientation)
    print('linear_velocity:', state.linear_velocity)
    print(state)
