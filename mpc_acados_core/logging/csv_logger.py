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

"""CSV logger and orientation helpers shared by MPC example runners.

``CsvLogger`` writes a fixed schema (37 columns) that matches the layout the
plotting utilities in :mod:`mpc_acados_core.plotting.plot_results` expect.
Helper functions to convert between quaternions and Euler angles in NumPy
(distinct from the CasADi versions in :mod:`mpc_acados_core.utils.quaternion_utils`)
are also provided because they are needed both by the logger and by the
examples that produce reference orientations.
"""

__authors__ = 'Rafael Perez-Segui'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import math
import os

import numpy as np


SIMULATOR_LOGS_DIR = 'simulator_logs'


def normalize_log_path(file_name: str) -> str:
    """Resolve plain file names into ``simulator_logs/<name>``.

    :param file_name: File name or path.
    :type file_name: str
    :return: Normalized path.
    :rtype: str
    """
    if os.path.dirname(file_name):
        return file_name
    return os.path.join(SIMULATOR_LOGS_DIR, file_name)


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert ZYX Euler angles to a unit quaternion ``[w, x, y, z]``."""
    sr = math.sin(roll * 0.5)
    cr = math.cos(roll * 0.5)
    sp = math.sin(pitch * 0.5)
    cp = math.cos(pitch * 0.5)
    sy = math.sin(yaw * 0.5)
    cy = math.cos(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def quaternion_to_euler(q: np.ndarray) -> np.ndarray:
    """Convert quaternion ``[w, x, y, z]`` to ZYX Euler angles."""
    w, x, y, z = q

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])


def compute_path_facing(direction: np.ndarray) -> np.ndarray:
    """Quaternion ``[w, x, y, z]`` facing the given planar direction vector."""
    yaw = math.atan2(direction[1], direction[0])
    return euler_to_quaternion(0.0, 0.0, yaw)


def get_desired_orientation(
        waypoint: np.ndarray,
        current_position: np.ndarray,
        current_orientation: np.ndarray,
        path_facing: bool) -> np.ndarray:
    """Desired orientation for the current waypoint.

    When ``path_facing`` is enabled the drone yaws to face the direction of
    travel. Yaw is held unchanged when the drone is within 0.1 m of the
    waypoint to avoid discontinuities near the goal.

    :param waypoint: Target waypoint position (m).
    :type waypoint: np.ndarray
    :param current_position: Current drone position (m).
    :type current_position: np.ndarray
    :param current_orientation: Current orientation quaternion ``[w, x, y, z]``.
    :type current_orientation: np.ndarray
    :param path_facing: Whether to align yaw with the direction of travel.
    :type path_facing: bool
    :return: Desired orientation quaternion ``[w, x, y, z]``.
    :rtype: np.ndarray
    """
    if not path_facing:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    diff = waypoint[:2] - current_position[:2]
    if np.linalg.norm(diff) < 0.1:
        return current_orientation.copy()

    return compute_path_facing(diff)


class CsvLogger:
    """Log MPC + simulator data to a CSV file.

    Columns (37 total): ``time``, state (pose + Euler + velocity +
    angular rate), reference (position + orientation quaternion + Euler),
    actuation (thrust + angular rate command), motor states, and control
    metadata (solver time, waypoint index, hover flag, max speed).
    """

    def __init__(self, file_name: str) -> None:
        """Open the CSV file and write the header row.

        :param file_name: Output file name. Plain names are placed under
            ``simulator_logs/``.
        :type file_name: str
        """
        self.file_name = normalize_log_path(file_name)
        output_dir = os.path.dirname(self.file_name)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        print(f'Saving to file: {self.file_name}')
        self.file = open(self.file_name, 'w')
        self.file.write(
            'time,'
            'x,y,z,qw,qx,qy,qz,roll,pitch,yaw,'
            'vx,vy,vz,wx,wy,wz,'
            'x_ref,y_ref,z_ref,'
            'qw_ref,qx_ref,qy_ref,qz_ref,roll_ref,pitch_ref,yaw_ref,'
            'thrust,wx_cmd,wy_cmd,wz_cmd,'
            'motor_w0,motor_w1,motor_w2,motor_w3,'
            'controller_solve_time_us,waypoint_index,hover_active,max_speed\n')

    def _write_value(self, value: float, comma: bool = True) -> None:
        self.file.write(f'{value}')
        if comma:
            self.file.write(',')

    def _write_vector(self, data: np.ndarray, comma: bool = True) -> None:
        for i in range(data.size):
            last = (i == data.size - 1)
            self._write_value(data[i], comma or not last)

    def save(
            self,
            time: float,
            position: np.ndarray,
            orientation: np.ndarray,
            linear_velocity: np.ndarray,
            angular_velocity: np.ndarray,
            reference_position: np.ndarray,
            reference_orientation: np.ndarray,
            thrust: float,
            command_angular_velocity: np.ndarray,
            motor_w: np.ndarray,
            controller_solve_time_us: float,
            waypoint_index: int,
            hover_active: bool,
            max_speed: float) -> None:
        """Save one simulation step."""
        euler = quaternion_to_euler(orientation)
        euler_ref = quaternion_to_euler(reference_orientation)

        self._write_value(time)
        self._write_vector(position)
        self._write_vector(orientation)
        self._write_vector(euler)
        self._write_vector(linear_velocity)
        self._write_vector(angular_velocity)
        self._write_vector(reference_position)
        self._write_vector(reference_orientation)
        self._write_vector(euler_ref)
        self._write_value(thrust)
        self._write_vector(command_angular_velocity)
        self._write_vector(motor_w)
        self._write_value(controller_solve_time_us)
        self._write_value(float(waypoint_index))
        self._write_value(1.0 if hover_active else 0.0)
        self._write_value(float(max_speed), comma=False)
        self.file.write('\n')

    def close(self) -> None:
        """Close the CSV file."""
        self.file.close()
