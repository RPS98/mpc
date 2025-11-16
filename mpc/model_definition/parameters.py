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
        'gains_contour_error',
        'gain_lag_error',
        'gains_orientation',
        'gains_actuation',
        'gains_actuation_theta_velocity',
        'gain_progress',
        's1_p',
        's1_m',
        's2_p',
        's2_m',
        's3_p',
        's3_m',
        's_length',
        's_poly_coeffs'
    ]
    _names_sx: ClassVar[List[str]] = [
        'sx_mass',
        'sx_desired_orientation',
        'sx_gains_contour_error',
        'sx_gain_lag_error',
        'sx_gains_orientation',
        'sx_gains_actuation',
        'sx_gains_actuation_theta_velocity',
        'sx_gain_progress',
        'sx_s1_p',
        'sx_s1_m',
        'sx_s2_p',
        'sx_s2_m',
        'sx_s3_p',
        'sx_s3_m',
        'sx_s_length',
        'sx_s_poly_coeffs'
    ]
    _sizes: ClassVar[List[int]] = [
        1,
        4,
        3,
        1,
        3,
        4,
        1,
        1,
        3,
        3,
        3,
        3,
        3,
        3,
        1,
        6
    ]

    mass: Union[ca.SX, ca.DM]
    desired_orientation: Union[ca.SX, ca.DM]
    gains_contour_error: Union[ca.SX, ca.DM]
    gain_lag_error: Union[ca.SX, ca.DM]
    gains_orientation: Union[ca.SX, ca.DM]
    gains_actuation: Union[ca.SX, ca.DM]
    gains_actuation_theta_velocity: Union[ca.SX, ca.DM]
    gain_progress: Union[ca.SX, ca.DM]
    s1_p: Union[ca.SX, ca.DM]
    s1_m: Union[ca.SX, ca.DM]
    s2_p: Union[ca.SX, ca.DM]
    s2_m: Union[ca.SX, ca.DM]
    s3_p: Union[ca.SX, ca.DM]
    s3_m: Union[ca.SX, ca.DM]
    s_length: Union[ca.SX, ca.DM]
    s_poly_coeffs: Union[ca.SX, ca.DM]


class CaParameters(_ParametersDef, VectorBase):
    """
    CasADi SX Parameters for the UAV.

    :param mass: Mass of the MAV (kg)
    :type mass: ca.SX
    :param desired_orientation: Desired orientation as a quaternion [qw, qx, qy, qz]
    :type desired_orientation: ca.SX
    :param gains_contour_error: Gains for contour error [e_cx,e_cy,e_cz]
    :type gains_contour_error: ca.SX
    :param gain_lag_error: Gains for lag error [e_l]
    :type gain_lag_error: ca.SX
    :param gains_orientation: Gains for orientation error [e_roll,e_pitch,e_yaw]
    :type gains_orientation: ca.SX
    :param gains_actuation: Gains for actuation inputs [thrust, wx, wy, wz]
    :type gains_actuation: ca.SX
    :param gains_actuation_theta_velocity: Gain for actuation input [v_theta]
    :type gains_actuation_theta_velocity: ca.SX
    :param gain_progress: Gain for progress maximization
    :type gain_progress: ca.SX
    :param s1_p: Spline: Position at Knot_1 [x, y, z] (m)
    :type s1_p: ca.SX
    :param s1_m: Spline: Tangent at Knot_1 [tx, ty, tz]
    :type s1_m: ca.SX
    :param s2_p: Spline: Position at Knot_2 [x, y, z] (m)
    :type s2_p: ca.SX
    :param s2_m: Spline: Tangent at Knot_2 [tx, ty, tz]
    :type s2_m: ca.SX
    :param s3_p: Spline: Position at Knot_3 [x, y, z] (m)
    :type s3_p: ca.SX
    :param s3_m: Spline: Tangent at Knot_3 [tx, ty, tz]
    :type s3_m: ca.SX
    :param s_length: Total length of the spline path
    :type s_length: ca.SX
    :param s_poly_coeffs: Coefficients of the polynomial for reparametrization
    :type s_poly_coeffs: ca.SX
    """

    _type = 'ca.SX'


class Parameters(_ParametersDef, VectorBase):
    """Parameters for the UAV Model."""

    _type = 'np.array'

    def __init__(
            self,
            mass: np.array = np.array(1.0),
            desired_orientation: np.array = np.array([1.0, 0.0, 0.0, 0.0]),
            gains_contour_error: np.array = np.array([0.0, 0.0, 0.0]),
            gain_lag_error: np.array = np.array([0.0]),
            gains_orientation: np.array = np.array([0.0, 0.0, 0.0]),
            gains_actuation: np.array = np.array([0.0, 0.0, 0.0, 0.0]),
            gains_actuation_theta_velocity: np.array = np.array(0.0),
            gain_progress: np.array = np.array(1.0),
            s1_p: np.array = np.array([0.0, 0.0, 0.0]),
            s1_m: np.array = np.array([0.0, 0.0, 0.0]),
            s2_p: np.array = np.array([0.0, 0.0, 0.0]),
            s2_m: np.array = np.array([0.0, 0.0, 0.0]),
            s3_p: np.array = np.array([0.0, 0.0, 0.0]),
            s3_m: np.array = np.array([0.0, 0.0, 0.0]),
            s_length: np.array = np.array(1.0),
            s_poly_coeffs: np.array = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])):
        """
        Parameters for the UAV Model.

        :param mass: Mass of the MAV (kg)
        :type mass: np.array
        :param desired_orientation: Desired orientation as a quaternion [qw, qx, qy, qz]
        :type desired_orientation: np.array
        :param gains_contour_error: Gains for contour error [e_cx,e_cy,e_cz]
        :type gains_contour_error: np.array
        :param gain_lag_error: Gains for lag error [e_l]
        :type gain_lag_error: np.array
        :param gains_orientation: Gains for orientation error [e_roll,e_pitch,e_yaw]
        :type gains_orientation: np.array
        :param gains_actuation: Gains for actuation inputs [thrust, wx, wy, wz]
        :type gains_actuation: np.array
        :param gains_actuation_theta_velocity: Gain for actuation input [v_theta]
        :type gains_actuation_theta_velocity: np.array
        :param gain_progress: Gain for progress maximization
        :type gain_progress: np.array
        :param s1_p: Spline: Position at Knot_1 [x, y, z] (m)
        :type s1_p: np.array
        :param s1_m: Spline: Tangent at Knot_1 [tx, ty, tz]
        :type s1_m: np.array
        :param s2_p: Spline: Position at Knot_2 [x, y, z] (m)
        :type s2_p: np.array
        :param s2_m: Spline: Tangent at Knot_2 [tx, ty, tz]
        :type s2_m: np.array
        :param s3_p: Spline: Position at Knot_3 [x, y, z] (m)
        :type s3_p: np.array
        :param s3_m: Spline: Tangent at Knot_3 [tx, ty, tz]
        :type s3_m: np.array
        :param s_length: Total length of the spline path
        :type s_length: np.array
        :param s_poly_coeffs: Coefficients of the polynomial for reparametrization
        :type s_poly_coeffs: np.array
        """
        super().__init__()
        self.mass = mass
        self.desired_orientation = desired_orientation
        self.gains_contour_error = gains_contour_error
        self.gain_lag_error = gain_lag_error
        self.gains_orientation = gains_orientation
        self.gains_actuation = gains_actuation
        self.gains_actuation_theta_velocity = gains_actuation_theta_velocity
        self.gain_progress = gain_progress
        self.s1_p = s1_p
        self.s1_m = s1_m
        self.s2_p = s2_p
        self.s2_m = s2_m
        self.s3_p = s3_p
        self.s3_m = s3_m
        self.s_length = s_length
        self.s_poly_coeffs = s_poly_coeffs


if __name__ == '__main__':
    # Test the Parameters class
    parameters = Parameters()
    print('CasADi SX:', parameters._names)
    print('CasADi SX Sizes:', parameters._sizes)
    print('Vector:', parameters.vector)
    print('mass:', parameters.mass)
    print('desired_orientation:', parameters.desired_orientation)
    print('gains_contour_error:', parameters.gains_contour_error)
    print('gain_lag_error:', parameters.gain_lag_error)
    print('gains_orientation:', parameters.gains_orientation)
    print('gains_actuation:', parameters.gains_actuation)
    print('gains_actuation_theta_velocity:', parameters.gains_actuation_theta_velocity)
    print('gain_progress:', parameters.gain_progress)
    print('s1_p:', parameters.s1_p)
    print('s1_m:', parameters.s1_m)
    print('s2_p:', parameters.s2_p)
    print('s2_m:', parameters.s2_m)
    print('s3_p:', parameters.s3_p)
    print('s3_m:', parameters.s3_m)
    print('s_length:', parameters.s_length)
    print('s_poly_coeffs:', parameters.s_poly_coeffs)
    print(parameters)
