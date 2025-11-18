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
"""Spline evaluation definition."""

__authors__ = 'Rafael Pérez Seguí, Carmen De Rojas Pita-Romero'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'
import numpy as np


class SplineNumPy:
    """
    Static utility class to evaluate Hermite spline using NumPy.
    
    This is designed to be used for numerical evaluations with NumPy arrays.
    
    All methods are static and work with NumPy arrays.
    """
    
    @staticmethod
    def hermite_segment(
            t: float | np.ndarray,
            p0: np.ndarray,
            p1: np.ndarray,
            m0: np.ndarray,
            m1: np.ndarray,
            t0: float = 0.0,
            t1: float = 1.0) -> np.ndarray:
        """
        Evaluate a Hermite spline segment at a given parameter t.
        
        :param t: Parameter at which to evaluate the spline (scalar or array)
        :type t: float | np.ndarray
        :param p0: Start point of the segment
        :type p0: np.ndarray
        :param p1: End point of the segment
        :type p1: np.ndarray
        :param m0: Tangent (derivative dp/dt) at the start point
        :type m0: np.ndarray
        :param m1: Tangent (derivative dp/dt) at the end point
        :type m1: np.ndarray
        :param t0: Start of the parameter range for this segment (default is 0.0)
        :type t0: float
        :param t1: End of the parameter range for this segment (default is 1.0)
        :type t1: float
        :return: NumPy array representing the value of the Hermite spline at parameter t
        :rtype: np.ndarray
        """
        # Normalize t to [0, 1]
        eps = 1e-9
        s = (t - t0) / (t1 - t0 + eps)
        
        h00 = 2 * s**3 - 3 * s**2 + 1
        h10 = s**3 - 2 * s**2 + s
        h01 = -2 * s**3 + 3 * s**2
        h11 = s**3 - s**2
        
        return h00 * p0 + h10 * m0 * (t1 - t0) + h01 * p1 + h11 * m1 * (t1 - t0)    

    @staticmethod
    def spline(
            t: float | np.ndarray,
            points: list[np.ndarray],
            tangents: list[np.ndarray],
            ti: list[float]) -> np.ndarray:
        """
        Evaluate a piecewise Hermite spline at a given parameter t.
        
        :param t: Parameter at which to evaluate the spline (scalar or array)
        :type t: float | np.ndarray
        :param points: List of NumPy arrays for the control points
        :type points: list[np.ndarray]
        :param tangents: List of NumPy arrays for the tangents at the control points
        :type tangents: list[np.ndarray]
        :param ti: List of parameter values at the control points
        :type ti: list[float]
        :return: NumPy array representing the value of the piecewise Hermite spline at parameter t
        :rtype: np.ndarray
        """
        n_segments = len(points) - 1
        if n_segments < 1:
            raise ValueError("Need at least two points to build a spline.")

        # Handle scalar and array inputs
        t_array = np.atleast_1d(t)
        is_scalar = np.isscalar(t)
        
        # Find which segment each t value belongs to
        segment_indices = np.searchsorted(ti[1:], t_array, side='right')
        segment_indices = np.clip(segment_indices, 0, n_segments - 1)
        
        # Initialize output
        if points[0].ndim == 1:
            output = np.zeros((len(t_array), len(points[0])))
        else:
            output = np.zeros((len(t_array),) + points[0].shape)
        
        # Evaluate each segment
        for i in range(n_segments):
            mask = segment_indices == i
            if np.any(mask):
                output[mask] = SplineNumPy.hermite_segment(
                    t_array[mask],
                    points[i],
                    points[i + 1],
                    tangents[i],
                    tangents[i + 1],
                    ti[i],
                    ti[i + 1]
                )
        
        # Handle values before first knot
        before_mask = t_array < ti[0]
        if np.any(before_mask):
            output[before_mask] = points[0]
        
        # Handle values after last knot
        after_mask = t_array > ti[-1]
        if np.any(after_mask):
            output[after_mask] = SplineNumPy.hermite_segment(
                t_array[after_mask],
                points[-2],
                points[-1],
                tangents[-2],
                tangents[-1],
                ti[-2],
                ti[-1]
            )
        
        return output[0] if is_scalar else output

    @staticmethod
    def hermite_segment_derivative(
            t: float | np.ndarray,
            p0: np.ndarray,
            p1: np.ndarray,
            m0: np.ndarray,
            m1: np.ndarray,
            t0: float = 0.0,
            t1: float = 1.0) -> np.ndarray:
        """
        Evaluate the derivative of a Hermite spline segment at a given parameter t.
        
        :param t: Parameter at which to evaluate the spline derivative (scalar or array)
        :type t: float | np.ndarray
        :param p0: Start point of the segment
        :type p0: np.ndarray
        :param p1: End point of the segment
        :type p1: np.ndarray
        :param m0: Tangent (derivative dp/dt) at the start point
        :type m0: np.ndarray
        :param m1: Tangent (derivative dp/dt) at the end point
        :type m1: np.ndarray
        :param t0: Start of the parameter range for this segment (default is 0.0)
        :type t0: float
        :param t1: End of the parameter range for this segment (default is 1.0)
        :type t1: float
        :return: NumPy array representing the derivative of the Hermite spline at parameter t
        :rtype: np.ndarray
        """
        # Normalize t to [0, 1]
        eps = 1e-9
        s = (t - t0) / (t1 - t0 + eps)
        ds_dt = 1 / (t1 - t0 + eps)
        
        h00_deriv = 6 * s**2 - 6 * s
        h10_deriv = 3 * s**2 - 4 * s + 1
        h01_deriv = -6 * s**2 + 6 * s
        h11_deriv = 3 * s**2 - 2 * s
        
        dp_ds = (
            h00_deriv * p0 +
            h10_deriv * m0 * (t1 - t0) +
            h01_deriv * p1 +
            h11_deriv * m1 * (t1 - t0)
        )
        
        return dp_ds * ds_dt
    
    @staticmethod
    def spline_derivative(
            t: float | np.ndarray,
            points: list[np.ndarray],
            tangents: list[np.ndarray],
            ti: list[float]) -> np.ndarray:
        """
        Evaluate the derivative of a piecewise Hermite spline at a given parameter t.
        
        :param t: Parameter at which to evaluate the spline derivative (scalar or array)
        :type t: float | np.ndarray
        :param points: List of NumPy arrays for the control points
        :type points: list[np.ndarray]
        :param tangents: List of NumPy arrays for the tangents at the control points
        :type tangents: list[np.ndarray]
        :param ti: List of parameter values at the control points
        :type ti: list[float]
        :return: NumPy array representing the derivative of the piecewise Hermite spline at parameter t
        :rtype: np.ndarray
        """
        n_segments = len(points) - 1
        if n_segments < 1:
            raise ValueError("Need at least two points to build a spline.")

        # Handle scalar and array inputs
        t_array = np.atleast_1d(t)
        is_scalar = np.isscalar(t)
        
        # Find which segment each t value belongs to
        segment_indices = np.searchsorted(ti[1:], t_array, side='right')
        segment_indices = np.clip(segment_indices, 0, n_segments - 1)
        
        # Initialize output
        if points[0].ndim == 1:
            output = np.zeros((len(t_array), len(points[0])))
        else:
            output = np.zeros((len(t_array),) + points[0].shape)
        
        # Evaluate each segment
        for i in range(n_segments):
            mask = segment_indices == i
            if np.any(mask):
                output[mask] = SplineNumPy.hermite_segment_derivative(
                    t_array[mask],
                    points[i],
                    points[i + 1],
                    tangents[i],
                    tangents[i + 1],
                    ti[i],
                    ti[i + 1]
                )
        
        # Handle values before first knot (derivative is zero)
        before_mask = t_array < ti[0]
        if np.any(before_mask):
            output[before_mask] = 0.0
        
        # Handle values after last knot
        after_mask = t_array > ti[-1]
        if np.any(after_mask):
            output[after_mask] = SplineNumPy.hermite_segment_derivative(
                t_array[after_mask],
                points[-2],
                points[-1],
                tangents[-2],
                tangents[-1],
                ti[-2],
                ti[-1]
            )
        
        return output[0] if is_scalar else output


def evaluate_arc_length_spline(
    s_eval: float | np.ndarray,
    points: list[np.ndarray],
    tangents: list[np.ndarray],
    ti: list[float],
    total_length: float,
    poly_coeffs: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """
    Evaluate arc length parametrized spline at a specific arc length s.
    
    This function takes the parameters computed by compute_arc_length_reparametrization
    and a specific arc length value s, and returns NumPy arrays for
    the position and tangent at that point.
    
    :param s_eval: Arc length value at which to evaluate the spline (scalar or array)
    :type s_eval: float | np.ndarray
    :param points: List of NumPy arrays for control points (each 3x1 or similar)
    :type points: list[np.ndarray]
    :param tangents: List of NumPy arrays for tangent vectors (each 3x1 or similar)
    :type tangents: list[np.ndarray]
    :param ti: List of parameter values at control points
    :type ti: list[float]
    :param total_length: Total arc length (scalar)
    :type total_length: float
    :param poly_coeffs: NumPy array with polynomial coefficients for t(s_normalized)
    :type poly_coeffs: np.ndarray
    :return: Tuple of (position, tangent) where:
             - position: np.ndarray for 3D position at s_eval
             - tangent: np.ndarray for unit tangent at s_eval
    :rtype: tuple[np.ndarray, np.ndarray]
    """
    # Normalize s to [0, 1]
    s_norm = s_eval / total_length
    
    # Get number of coefficients
    n_coeffs = len(poly_coeffs)
    
    # Polynomial approximation: t(s_norm) = sum(c_i * s_norm^i)
    t_of_s = 0.0
    for i in range(n_coeffs):
        t_of_s += poly_coeffs[n_coeffs - 1 - i] * s_norm**i
    
    # Clamp t to valid range
    t_min = ti[0]
    t_max = ti[-1]
    t_of_s = np.maximum(t_min, np.minimum(t_max, t_of_s))
    
    # Evaluate p(t(s))
    p_s = SplineNumPy.spline(t_of_s, points, tangents, ti)
    
    # Compute dp/ds using chain rule: dp/ds = (dp/dt) * (dt/ds)
    dp_dt_val = SplineNumPy.spline_derivative(t_of_s, points, tangents, ti)
    
    # dt/ds from polynomial derivative - compute analytically
    dt_ds = 0.0
    for i in range(1, n_coeffs):
        dt_ds += i * poly_coeffs[n_coeffs - 1 - i] * s_norm**(i-1) / total_length
    
    # dp/ds (raw, not normalized)
    dp_ds_raw = dp_dt_val * dt_ds
    
    # Normalize to get unit tangent
    norm_dp_ds = np.linalg.norm(dp_ds_raw, axis=-1, keepdims=True)
    dp_ds_unit = dp_ds_raw / (norm_dp_ds + 1e-10)
    
    return p_s, dp_ds_unit, t_of_s
