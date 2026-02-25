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
"""Casadi quaternion utilities."""

__authors__ = 'Rafael Perez-Segui'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import casadi as ca


def quaternion_derivate(quaternion: ca.SX, angular_velocity: ca.SX) -> ca.SX:
    """
    Compute the quaternion derivative.

    q_dot = 0.5 * q x omega = 0.5 * [qw, qx, qy, qz] * [0, ,wx, wy, wz]

    :param quaternion: The quaternion [qw, qx, qy, qz].
    :type quaternion: ca.SX
    :param angular_velocity: The angular velocity [wx, wy, wz].
    :type angular_velocity: ca.SX
    :return: The quaternion derivative [qw_dot, qx_dot, qy_dot, qz_dot].
    :rtype: ca.SX
    """
    w_qx = angular_velocity[0]
    w_qy = angular_velocity[1]
    w_qz = angular_velocity[2]
    w_q = ca.vertcat(
        0.0,
        w_qx,
        w_qy,
        w_qz)

    return 0.5 * quaternion_multiply(
        normalize_quaternion(quaternion), w_q)


def quaternion_multiply(q1: ca.SX, q2: ca.SX) -> ca.SX:
    """
    Multiply two quaternions.

    q = q1 x q2 = [qw1, qx1, qy1, qz1] x [qw2, qx2, qy2, qz2]

    :param q1: The first quaternion [qw1, qx1, qy1, qz1].
    :type q1: ca.SX
    :param q2: The second quaternion [qw2, qx2, qy2, qz2].
    :type q2: ca.SX
    :return: The resulting quaternion [qw, qx, qy, qz].
    :rtype: ca.SX
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


def apply_rotation(q: ca.SX, v: ca.SX) -> ca.SX:
    """
    Apply a rotation to a vector.

    v_rotated = q x v x q_conj

    :param q: The quaternion [qw, qx, qy, qz].
    :type q: ca.SX
    :param v: The vector [vx, vy, vz].
    :type v: ca.SX
    :return: The rotated vector [vx_rotated, vy_rotated,
    :rtype: ca.SX
        vz_rotated].
    """
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]

    vx = v[0]
    vy = v[1]
    vz = v[2]

    q_conj = ca.vertcat(qw, -qx, -qy, -qz)

    v_rotated = quaternion_multiply(
        quaternion_multiply(q, ca.vertcat(0, vx, vy, vz)),
        q_conj)

    return ca.vertcat(v_rotated[1], v_rotated[2], v_rotated[3])


def apply_inverse_rotation(q: ca.SX, v: ca.SX) -> ca.SX:
    """
    Apply the inverse rotation to a vector.

    v_rotated = q_conj x v x q

    :param q: The quaternion [qw, qx, qy, qz].
    :type q: ca.SX
    :param v: The vector [vx, vy, vz].
    :type v: ca.SX
    :return: The rotated vector [vx_rotated, vy_rotated, vz_rotated].
    :rtype: ca.SX
    """
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]

    vx = v[0]
    vy = v[1]
    vz = v[2]

    q_conj = ca.vertcat(qw, -qx, -qy, -qz)

    v_rotated = quaternion_multiply(
        quaternion_multiply(q_conj, ca.vertcat(0, vx, vy, vz)),
        q)

    return ca.vertcat(v_rotated[1], v_rotated[2], v_rotated[3])


def quaternion_inverse(q: ca.DM) -> ca.DM:
    """
    Calculate the inverse of a quaternion.

    q_inv = q_conjugate / q_norm^2

    :param q: The input quaternion [q_w, q_x, q_y, q_z].
    :type q: ca.DM
    :return: The inverse quaternion.
    :rtype: ca.DM
    """
    q_conjugate = ca.DM([q[0], -q[1], -q[2], -q[3]])
    q_normalize = normalize_quaternion(q)

    q_norm = ca.sqrt(
        q_normalize[0]**2 +
        q_normalize[1]**2 +
        q_normalize[2]**2 +
        q_normalize[3]**2)

    if q_norm < 1e-3:
        q_norm = 1e-3
    return q_conjugate / q_norm


def quaternion_error(q_desired: ca.SX, q_current: ca.SX) -> ca.SX:
    """
    Compute the quaternion error.

    q_error = q_desired x q_current_conj

    q_norm = sqrt(q_error[0] ** 2 + q_error[3] ** 2 + 1e-3)

    v_error = [q_error[0] * q_error[1] - q_error[2] * q_error[3],
                    q_error[0] * q_error[2] + q_error[1] * q_error[3],
                    q_error[3]] / q_norm

    :param q_desired: The desired quaternion [qw, qx, qy, qz].
    :type q_desired: ca.SX
    :param q_current: The current quaternion [qw, qx, qy, qz].
    :type q_current: ca.SX

    :return: The vector error [vx_error, vy_error, vz_error].
    :rtype: ca.SX
    """
    q_error = quaternion_multiply(
        q_desired,
        ca.vertcat(q_current[0], -q_current[1], -q_current[2], -q_current[3]))

    # Project the error to the 3D space to get z-axis error
    q_norm = ca.sqrt(q_error[0] ** 2 + q_error[3] ** 2 + 1e-3)
    v_error = ca.vertcat(
        q_error[0] * q_error[1] - q_error[2] * q_error[3],
        q_error[0] * q_error[2] + q_error[1] * q_error[3],
        q_error[3]) / q_norm
    return v_error


def normalize_quaternion(q: ca.SX) -> ca.SX:
    """
    Normalize a quaternion.

    :param q: The quaternion to normalize.
    :type q: ca.SX
    :return: The normalized quaternion.
    :rtype: ca.SX
    """
    q_norm = ca.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2)
    return q / q_norm


def quaternion_to_euler(q: ca.SX) -> ca.SX:
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw).

    :param q: The quaternion [qw, qx, qy, qz].
    :type q: ca.SX
    :return: The Euler angles [roll, pitch, yaw].
    :rtype: ca.SX
    """
    # First, normalize the quaternion to ensure a valid rotation.
    q_normed = normalize_quaternion(q)

    # Extract individual components
    qw = q_normed[0]
    qx = q_normed[1]
    qy = q_normed[2]
    qz = q_normed[3]

    # --- Roll (x-axis rotation) ---
    # sinr_cosp = 2 * (qw * qx + qy * qz)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    # cosr_cosp = 1 - 2 * (qx^2 + qy^2)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    # roll = atan2(sinr_cosp, cosr_cosp)
    roll = ca.atan2(sinr_cosp, cosr_cosp)

    # --- Pitch (y-axis rotation) ---
    # sinp = 2 * (qw * qy - qz * qx)
    sinp = 2 * (qw * qy - qz * qx)
    # pitch = asin(sinp)
    pitch = ca.asin(sinp)

    # --- Yaw (z-axis rotation) ---
    # siny_cosp = 2 * (qw * qz + qx * qy)
    siny_cosp = 2 * (qw * qz + qx * qy)
    # cosy_cosp = 1 - 2 * (qy^2 + qz^2)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    # yaw = atan2(siny_cosp, cosy_cosp)
    yaw = ca.atan2(siny_cosp, cosy_cosp)

    # Return the Euler angles in the order [roll, pitch, yaw]
    return ca.vertcat(roll, pitch, yaw)
