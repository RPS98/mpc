#!/usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
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

"""Example utils shared across MPC examples."""

__authors__ = 'Rafael Perez-Segui'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import math
import numpy as np


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convert Euler angles to a quaternion.

    :param roll: Roll angle in radians.
    :param pitch: Pitch angle in radians.
    :param yaw: Yaw angle in radians.
    :return: np.ndarray object.
    """
    # Calculate half angles
    roll_half = roll * 0.5
    pitch_half = pitch * 0.5
    yaw_half = yaw * 0.5

    # Calculate sine and cosine of the half angles
    sr = math.sin(roll_half)
    cr = math.cos(roll_half)
    sp = math.sin(pitch_half)
    cp = math.cos(pitch_half)
    sy = math.sin(yaw_half)
    cy = math.cos(yaw_half)

    # Calculate quaternion components
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    # Return the quaternion
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def quaternion_to_euler(q: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw).

    :param q: Quaternion as a numpy array [w, x, y, z].
    :return: Numpy array with Euler angles [roll, pitch, yaw] in radians.
    """
    w, x, y, z = q

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])


def compute_path_facing(velocity: np.ndarray) -> list:
    """
    Compute the quaternion facing based on velocity.

    :param velocity: 3D velocity vector.
    :return: Quaternion as a list [w, x, y, z].
    """
    yaw = math.atan2(velocity[1], velocity[0])
    pitch, roll = 0.0, 0.0

    q = euler_to_quaternion(roll, pitch, yaw)
    return np.array([q[0], q[1], q[2], q[3]])


class CsvLogger:
    """Log simulation data to a csv file."""

    def __init__(self, file_name: str) -> None:
        """
        Log simulation data to a csv file.

        :param file_name(str): Name of the file to save the data.
        """
        self.file_name = file_name
        print(f'Saving to file: {self.file_name}')
        self.file = open(self.file_name, 'w')
        self.file.write(
            'time,'
            'x,y,z,qw,qx,qy,qz,roll,pitch,yaw,vx,vy,vz,'
            'x_ref,y_ref,z_ref,qw_ref,qx_ref,qy_ref,qz_ref,roll_ref,pitch_ref,yaw_ref,vx_ref,vy_ref,vz_ref,'
            'thrust,wx,wy,wz\n')

    def add_double(self, data: float) -> None:
        """
        Add a double data to the csv file.

        :param data(float): Double data to add.
        """
        if data is None:
            raise ValueError('Data is None')
        self.file.write(f'{data},')

    def add_string(self, data: str, add_final_comma: bool = True) -> None:
        """
        Add a string data to the csv file.

        :param data(str): String data to add.
        :param add_final_comma(bool): Add a final comma to the string.
        """
        self.file.write(f'{data}')
        if add_final_comma:
            self.file.write(',')

    def add_vector_row(self, data: np.array, add_final_comma: bool = True) -> None:
        """
        Add a vector data to the csv file.

        :param data(np.array): Vector data to add.
        :param add_final_comma(bool): Add a final comma to the string.
        """
        for i in range(data.size):
            if data[i] is None:
                raise ValueError('Data is None')
            self.file.write(f'{data[i]}')
            if i < data.size - 1:
                self.file.write(',')
            elif add_final_comma:
                self.file.write(',')

    def save(
            self,
            time: float,
            state_position: np.ndarray,
            state_orientation: np.ndarray,
            state_velocity: np.ndarray,
            reference_position: np.ndarray,
            reference_orientation: np.ndarray,
            reference_velocity: np.ndarray,
            control_thrust: np.ndarray,
            control_angular_velocity: np.ndarray) -> None:
        """
        Save the simulation data to the csv file.

        :param time: Current simulation time.
        :type time: float
        :param state_position: Current position of the drone.
        :type state_position: np.ndarray
        :param state_orientation: Current orientation of the drone as a quaternion.
        :type state_orientation: np.ndarray
        :param state_velocity: Current linear velocity of the drone.
        :type state_velocity: np.ndarray
        :param reference_position: Desired position of the drone.
        :type reference_position: np.ndarray
        :param reference_orientation: Desired orientation of the drone as a quaternion.
        :type reference_orientation: np.ndarray
        :param reference_velocity: Desired linear velocity of the drone.
        :type reference_velocity: np.ndarray
        :param control_thrust: Control thrust applied to the drone.
        :type control_thrust: np.ndarray
        :param control_angular_velocity: Control angular velocity applied to the drone.
        :type control_angular_velocity: np.ndarray
        :return: None
        """
        self.add_double(time)

        # State
        state_euler = quaternion_to_euler(state_orientation)
        self.add_vector_row(state_position)
        self.add_vector_row(state_orientation)
        self.add_vector_row(state_euler)
        self.add_vector_row(state_velocity)

        # Reference
        reference_euler = quaternion_to_euler(reference_orientation)
        self.add_vector_row(reference_position)
        self.add_vector_row(reference_orientation)
        self.add_vector_row(reference_euler)
        self.add_vector_row(reference_velocity)

        # Control
        self.add_double(control_thrust)
        self.add_vector_row(control_angular_velocity, add_final_comma=False)

        # End line
        self.file.write('\n')

    def close(self) -> None:
        """Close the csv file."""
        self.file.close()
