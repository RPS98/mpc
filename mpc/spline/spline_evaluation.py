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
import casadi as ca


class SplineCasADi:
    """
    Static utility class to evaluate Hermite spline in CasADi from parameters.
    
    This is designed to be used directly in Acados models where you have
    the spline parameters as symbolic variables.
    
    All methods are static and work with CasADi symbolic expressions.
    """
    
    @staticmethod
    def hermite_segment(
            t: ca.SX,
            p0: ca.SX,
            p1: ca.SX,
            m0: ca.SX,
            m1: ca.SX,
            t0: ca.SX = 0,
            t1: ca.SX = 1) -> ca.SX:
        """
        Evaluate a Hermite spline segment at a given parameter t.
        
        :param t CasADi symbolic variable representing the parameter at which to evaluate the spline.
        :type t: ca.SX
        :param p0: CasADi symbolic variable for the start point of the segment.
        :type p0: ca.SX
        :param p1: CasADi symbolic variable for the end point of the segment.
        :type p1: ca.SX
        :param m0: tangent (derivative dp/dt) at the start point
        :type m0: ca.SX
        :param m1: tangent (derivative dp/dt) at the end point
        :type m1: ca.SX
        :param t0: Start of the parameter range for this segment (default is 0).
        :type t0: ca.SX
        :param t1: End of the parameter range for this segment (default is 1).
        :type t1: ca.SX
        :return: A CasADi symbolic expression representing the value of the Hermite spline at parameter t.
        :rtype: ca.SX
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
            t: ca.SX,
            points: list[ca.SX],
            tangents: list[ca.SX],
            ti: list[ca.SX]) -> ca.SX:
        """
        Evaluate a piecewise Hermite spline at a given parameter t.
        
        :param t CasADi symbolic variable representing the parameter at which to evaluate the spline.
        :type t: ca.SX
        :param points: List of CasADi symbolic variables for the control points.
        :type points: list[ca.SX]
        :param tangents: List of CasADi symbolic variables for the tangents at the control points.
        :type tangents: list[ca.SX]
        :param ti: List of CasADi symbolic variables for the parameter values at the control points.
        :type ti: list[ca.SX]
        :return: A CasADi symbolic expression representing the value of the piecewise Hermite spline at parameter t.
        :rtype: ca.SX
        """
        n_segments = len(points) - 1
        if n_segments < 1:
            raise ValueError("Need at least two points to build a spline.")

        # Start with the last segment as the default (used for t > ti[-1])
        last_idx = n_segments - 1
        spline_expr = SplineCasADi.hermite_segment(
            t,
            points[last_idx],
            points[last_idx + 1],
            tangents[last_idx],
            tangents[last_idx + 1],
            ti[last_idx],
            ti[last_idx + 1]
        )

        # Build nested if_else from last-1 down to first segment:
        # if t <= ti[i+1] then use segment i else keep previous expr
        for i in range(n_segments - 2, -1, -1):
            seg_expr = SplineCasADi.hermite_segment(
                t,
                points[i],
                points[i + 1],
                tangents[i],
                tangents[i + 1],
                ti[i],
                ti[i + 1]
            )
            spline_expr = ca.if_else(t <= ti[i + 1], seg_expr, spline_expr)

        # If t is before the first knot, clamp to the first control point
        spline_expr = ca.if_else(t < ti[0], points[0], spline_expr)

        return spline_expr

    @staticmethod
    def hermite_segment_derivative(
            t: ca.SX,
            p0: ca.SX,
            p1: ca.SX,
            m0: ca.SX,
            m1: ca.SX,
            t0: ca.SX = 0,
            t1: ca.SX = 1) -> ca.SX:
        """
        Evaluate the derivative of a Hermite spline segment at a given parameter t.
        
        :param t CasADi symbolic variable representing the parameter at which to evaluate the spline derivative.
        :type t: ca.SX
        :param p0: CasADi symbolic variable for the start point of the segment.
        :type p0: ca.SX
        :param p1: CasADi symbolic variable for the end point of the segment.
        :type p1: ca.SX
        :param m0: tangent (derivative dp/dt) at the start point
        :type m0: ca.SX
        :param m1: tangent (derivative dp/dt) at the end point
        :type m1: ca.SX
        :param t0: Start of the parameter range for this segment (default is 0).
        :type t0: ca.SX
        :param t1: End of the parameter range for this segment (default is 1).
        :type t1: ca.SX
        :return: A CasADi symbolic expression representing the derivative of the Hermite spline at parameter t.
        :rtype: ca.SX
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
            t: ca.SX,
            points: list[ca.SX],
            tangents: list[ca.SX],
            ti: list[ca.SX]) -> ca.SX:
        """
        Evaluate the derivative of a piecewise Hermite spline at a given parameter t.
        
        :param t CasADi symbolic variable representing the parameter at which to evaluate the spline derivative.
        :type t: ca.SX
        :param points: List of CasADi symbolic variables for the control points.
        :type points: list[ca.SX]
        :param tangents: List of CasADi symbolic variables for the tangents at the control points.
        :type tangents: list[ca.SX]
        :param ti: List of CasADi symbolic variables for the parameter values at the control points.
        :type ti: list[ca.SX]
        :return: A CasADi symbolic expression representing the derivative of the piecewise Hermite spline at parameter t.
        :rtype: ca.SX
        """
        n_segments = len(points) - 1
        if n_segments < 1:
            raise ValueError("Need at least two points to build a spline.")

        # Start with the last segment as the default (used for t > ti[-1])
        last_idx = n_segments - 1
        spline_deriv_expr = SplineCasADi.hermite_segment_derivative(
            t,
            points[last_idx],
            points[last_idx + 1],
            tangents[last_idx],
            tangents[last_idx + 1],
            ti[last_idx],
            ti[last_idx + 1]
        )

        # Build nested if_else from last-1 down to first segment:
        # if t <= ti[i+1] then use segment i else keep previous expr
        for i in range(n_segments - 2, -1, -1):
            seg_deriv_expr = SplineCasADi.hermite_segment_derivative(
                t,
                points[i],
                points[i + 1],
                tangents[i],
                tangents[i + 1],
                ti[i],
                ti[i + 1]
            )
            spline_deriv_expr = ca.if_else(t <= ti[i + 1], seg_deriv_expr, spline_deriv_expr)

        # If t is before the first knot, derivative is zero
        zero_deriv = ca.DM.zeros(points[0].shape[0], 1)
        spline_deriv_expr = ca.if_else(t < ti[0], zero_deriv, spline_deriv_expr)
        return spline_deriv_expr


def evaluate_arc_length_spline(
    s_eval: ca.SX,
    points: list[ca.SX],
    tangents: list[ca.SX],
    ti: list[ca.SX],
    total_length: ca.SX,
    poly_coeffs: ca.SX
) -> tuple[ca.SX, ca.SX]:
    """
    Evaluate arc length parametrized spline at a specific arc length s.
    
    This function takes the parameters computed by compute_arc_length_reparametrization
    and a specific arc length value s, and returns symbolic CasADi expressions for
    the position and tangent at that point.
    
    :param s_eval: Arc length value at which to evaluate the spline (CasADi symbolic)
    :type s_eval: ca.SX
    :param points: List of CasADi symbolic variables for control points (each 3x1)
    :type points: list[ca.SX]
    :param tangents: List of CasADi symbolic variables for tangent vectors (each 3x1)
    :type tangents: list[ca.SX]
    :param ti: List of CasADi symbolic variables for parameter values at control points
    :type ti: list[ca.SX]
    :param total_length: Total arc length (CasADi symbolic scalar)
    :type total_length: ca.SX
    :param poly_coeffs: CasADi symbolic variable with polynomial coefficients for t(s_normalized)
    :type poly_coeffs: ca.SX
    :return: Tuple of (position, tangent) where:
             - position: ca.SX expression for 3D position at s_eval
             - tangent: ca.SX expression for unit tangent at s_eval
    :rtype: tuple[ca.SX, ca.SX]
    """
    # Points, tangents, and ti are already CasADi lists
    points_sx = points
    tangents_sx = tangents
    ti_sx = ti
    
    # Normalize s to [0, 1]
    s_norm = s_eval / total_length
    
    # Get number of coefficients (handle both numpy arrays and CasADi objects)
    if isinstance(poly_coeffs, (ca.SX, ca.MX, ca.DM)):
        n_coeffs = poly_coeffs.shape[0]
    else:
        n_coeffs = len(poly_coeffs)
    
    # Polynomial approximation: t(s_norm) = sum(c_i * s_norm^i)
    t_of_s = 0.0
    for i in range(n_coeffs):
        t_of_s += poly_coeffs[n_coeffs - 1 - i] * s_norm**i
    
    # Clamp t to valid range
    t_min = ti_sx[0]
    t_max = ti_sx[-1]
    t_of_s = ca.fmax(t_min, ca.fmin(t_max, t_of_s))
    
    # Evaluate p(t(s))
    p_s = SplineCasADi.spline(t_of_s, points_sx, tangents_sx, ti_sx)
    
    # Compute dp/ds using chain rule: dp/ds = (dp/dt) * (dt/ds)
    dp_dt_val = SplineCasADi.spline_derivative(t_of_s, points_sx, tangents_sx, ti_sx)
    
    # dt/ds from polynomial derivative - compute analytically
    dt_ds = 0.0
    for i in range(1, n_coeffs):
        dt_ds += i * poly_coeffs[n_coeffs - 1 - i] * s_norm**(i-1) / total_length
    
    # dp/ds (raw, not normalized)
    dp_ds_raw = dp_dt_val * dt_ds
    
    # Normalize to get unit tangent
    norm_dp_ds = ca.norm_2(dp_ds_raw)
    dp_ds_unit = dp_ds_raw / (norm_dp_ds + 1e-10)
    
    return p_s, dp_ds_unit
