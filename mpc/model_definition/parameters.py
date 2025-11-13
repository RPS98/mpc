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
        'theta_knot_1',
        'position_knot_1',
        'm_knot_1',
        'theta_knot_2',
        'position_knot_2',
        'm_knot_2',
        'theta_knot_3',
        'position_knot_3',
        'm_knot_3'
    ]
    _names_sx: ClassVar[List[str]] = [
        'sx_mass',
        'sx_desired_orientation',
        'sx_theta_knot_1',
        'sx_position_knot_1',
        'sx_m_knot_1',
        'sx_theta_knot_2',
        'sx_position_knot_2',
        'sx_m_knot_2',
        'sx_theta_knot_3',
        'sx_position_knot_3',
        'sx_m_knot_3'
    ]
    _sizes: ClassVar[List[int]] = [
        1,
        4,
        1,
        3,
        3,
        1,
        3,
        3,
        1,
        3,
        3
    ]

    mass: Union[ca.SX, ca.DM]
    desired_orientation: Union[ca.SX, ca.DM]
    theta_knot_1: Union[ca.SX, ca.DM]
    position_knot_1: Union[ca.SX, ca.DM]
    m_knot_1: Union[ca.SX, ca.DM]
    theta_knot_2: Union[ca.SX, ca.DM]
    position_knot_2: Union[ca.SX, ca.DM]
    m_knot_2: Union[ca.SX, ca.DM]
    theta_knot_3: Union[ca.SX, ca.DM]
    position_knot_3: Union[ca.SX, ca.DM]
    m_knot_3: Union[ca.SX, ca.DM]


class CaParameters(_ParametersDef, VectorBase):
    """
    CasADi SX Parameters for the UAV.

    :param mass: Mass of the MAV (kg)
    :type mass: ca.SX
    :param desired_orientation: Desired orientation as a quaternion [qw, qx, qy, qz]
    :type desired_orientation: ca.SX
    :param theta_knot_1: Knot_1 for contouring progress along the path
    :type theta_knot_1: ca.SX
    :param position_knot_1: Position at Knot_1 [x, y, z] (m)
    :type position_knot_1: ca.SX
    :param m_knot_1: derivate at Knot_1 for contouring progress along the path
    :type m_knot_1: ca.SX
    :param theta_knot_2: Knot_2 for contouring progress along the path
    :type theta_knot_2: ca.SX
    :param position_knot_2: Position at Knot_2 [x, y, z] (m)
    :type position_knot_2: ca.SX
    :param m_knot_2: derivate at Knot_2 for contouring progress along the path
    :type m_knot_2: ca.SX
    :param theta_knot_3: Knot_3 for contouring progress along the path
    :type theta_knot_3: ca.SX
    :param position_knot_3: Position at Knot_3 [x, y, z] (m)
    :type position_knot_3: ca.SX
    :param m_knot_3: derivate at Knot_3 for contouring progress along the path
    :type m_knot_3: ca.SX
    """

    _type = 'ca.SX'


class Parameters(_ParametersDef, VectorBase):
    """Parameters for the UAV Model."""

    _type = 'np.array'

    def __init__(
            self,
            mass: np.array = np.array(1.0),
            desired_orientation: np.array = np.array([1.0, 0.0, 0.0, 0.0]),
            theta_knot_1: np.array = np.array(0.0),
            position_knot_1: np.array = np.array([0.0, 0.0, 0.0]),
            m_knot_1: np.array = np.array([0.0, 0.0, 0.0]),
            theta_knot_2: np.array = np.array(0.0),
            position_knot_2: np.array = np.array([0.0, 0.0, 0.0]),
            m_knot_2: np.array = np.array([0.0, 0.0, 0.0]),
            theta_knot_3: np.array = np.array(0.0),
            position_knot_3: np.array = np.array([0.0, 0.0, 0.0]),
            m_knot_3: np.array = np.array([0.0, 0.0, 0.0])):
        """
        Parameters for the UAV Model.

        :param mass: Mass of the MAV (kg)
        :type mass: np.array
        :param desired_orientation: Desired orientation as a quaternion [qw, qx, qy, qz]
        :type desired_orientation: np.array
        :param theta_knot_1: Knot_1 for contouring progress along the path
        :type theta_knot_1: np.array
        :param position_knot_1: Position at Knot_1 [x, y, z] (m)
        :type position_knot_1: np.array
        :param m_knot_1: derivate at Knot_1 for contouring progress along the path
        :type m_knot_1: np.array
        :param theta_knot_2: Knot_2 for contouring progress along the path
        :type theta_knot_2: np.array
        :param position_knot_2: Position at Knot_2 [x, y, z] (m)
        :type position_knot_2: np.array
        :param m_knot_2: derivate at Knot_2 for contouring progress along the path
        :type m_knot_2: np.array
        :param theta_knot_3: Knot_3 for contouring progress along the path
        :type theta_knot_3: np.array
        :param position_knot_3: Position at Knot_3 [x, y, z] (m)
        :type position_knot_3: np.array
        :param m_knot_3: derivate at Knot_3 for contouring progress along the path
        :type m_knot_3: np.array
        """
        super().__init__()
        self.mass = mass
        self.desired_orientation = desired_orientation
        self.theta_knot_1 = theta_knot_1
        self.position_knot_1 = position_knot_1
        self.m_knot_1 = m_knot_1
        self.theta_knot_2 = theta_knot_2
        self.position_knot_2 = position_knot_2
        self.m_knot_2 = m_knot_2
        self.theta_knot_3 = theta_knot_3
        self.position_knot_3 = position_knot_3
        self.m_knot_3 = m_knot_3


if __name__ == '__main__':
    # Test the Parameters class
    parameters = Parameters()
    print('CasADi SX:', parameters._names)
    print('CasADi SX Sizes:', parameters._sizes)
    print('Vector:', parameters.vector)
    print('mass:', parameters.mass)
    print('desired_orientation:', parameters.desired_orientation)
    print('theta_knot_1:', parameters.theta_knot_1)
    print('position_knot_1:', parameters.position_knot_1)
    print('m_knot_1:', parameters.m_knot_1)
    print('theta_knot_2:', parameters.theta_knot_2)
    print('position_knot_2:', parameters.position_knot_2)
    print('m_knot_2:', parameters.m_knot_2)
    print('theta_knot_3:', parameters.theta_knot_3)
    print('position_knot_3:', parameters.position_knot_3)
    print('m_knot_3:', parameters.m_knot_3)
    print(parameters)
