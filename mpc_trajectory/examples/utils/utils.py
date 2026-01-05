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

"""Example utils."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import ctypes
import os
import sys
import math
import numpy as np
import importlib.util
import casadi as ca

from mpc.model_definition.parameters import Parameters
from mpc.model_definition.actuation import Actuation
from mpc.model_definition.state import State


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


def get_trajectory_generator(initial_position: np.ndarray, waypoints: list, speed: float):
    """
    Initialize and generate a dynamic trajectory.

    :param initial_position: Initial position of the vehicle.
    :param waypoints: List of waypoints.
    :param speed: Speed of the trajectory generator.
    :return: Generated trajectory.
    """
    # Load trajectory_generator.so file from the current file directory
    current_dir = os.path.dirname(os.path.abspath(__file__))
    so_file_path = os.path.join(current_dir, 'trajectory_generator.so')
    
    # Check existence of the shared library
    if not os.path.isfile(so_file_path):
        raise FileNotFoundError(f"Shared library not found at path: {so_file_path}")
    
    # Load dependencies first with RTLD_GLOBAL so they are available for trajectory_generator.so
    dep_lib_path = os.path.join(current_dir, 'libdynamic_trajectory_generator.so')
    if os.path.isfile(dep_lib_path):
        ctypes.CDLL(dep_lib_path, mode=ctypes.RTLD_GLOBAL)
    
    mav_lib_path = os.path.join(current_dir, 'libmav_trajectory_generation.so')
    if os.path.isfile(mav_lib_path):
        ctypes.CDLL(mav_lib_path, mode=ctypes.RTLD_GLOBAL)
    
    # Add current directory to sys.path temporarily to allow import
    if current_dir not in sys.path:
        sys.path.insert(0, current_dir)
    
    # Import the trajectory_generator module (it's a Python extension module)
    spec = importlib.util.spec_from_file_location("trajectory_generator", so_file_path)
    dtb = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(dtb)

    # Initialize dynamic trajectory generator
    trajectory_generator = dtb.DynamicTrajectory()

    # Set waypoints
    trajectory_generator.set_path_facing(False)
    trajectory_generator.generate_trajectory(
        initial_position,
        0.0,
        waypoints,
        speed)

    # Generate trajectory
    max_time = trajectory_generator.get_max_time()

    print(f"Trajectory generated with max time: {max_time}")
    return trajectory_generator


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
            'x,y,z,qw,qx,qy,qz,roll,pitch,yaw,vx,vy,vz,wx,wy,wz,w0,w1,w2,w3,fz,'
            'x_ref,y_ref,z_ref,qw_ref,qx_ref,qy_ref,qz_ref,roll_ref,pitch_ref,yaw_ref,vx_ref,vy_ref,vz_ref,'
            'fz_ref,w0_ref,w1_ref,w2_ref,w3_ref\n')

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

    def save(self, time: float, x: State, y: np.ndarray, u: Actuation, 
            p: Parameters,) -> None:
        """
        Save the simulation data to the csv file.

        :param time(float): Current simulation time.
        :param simulator(ms.Simulator): Simulator object.
        """
        self.add_double(time)

        # Aux
        thrust_force_b_state = ca.vertcat(0, 0, ca.sum1(p.motors_cf * x.motor_angular_velocity**2))
        motor_angular_velocity_ref = u.motor_angular_velocity * (p.motors_max_angular_velocity - p.motors_min_angular_velocity) + p.motors_min_angular_velocity
        thrust_force_b_ref = ca.vertcat(0, 0, ca.sum1(p.motors_cf * motor_angular_velocity_ref**2))
        euler_q = quaternion_to_euler(x.orientation)

        # State
        self.add_vector_row(x.position)
        self.add_vector_row(x.orientation)
        self.add_vector_row(euler_q)
        self.add_vector_row(x.linear_velocity)
        self.add_vector_row(x.angular_velocity)
        self.add_vector_row(x.motor_angular_velocity)
        self.add_double(thrust_force_b_state[2])  # Thrust force in z

        # Reference position
        y_pos = y[0:3]
        q_ref = y[3:7]
        euler_q_ref = quaternion_to_euler(q_ref)
        v_ref = y[7:10]
        self.add_vector_row(y_pos)
        self.add_vector_row(q_ref)
        self.add_vector_row(euler_q_ref)
        self.add_vector_row(v_ref)
        self.add_double(thrust_force_b_ref[2])  # Thrust force in z

        # Control
        self.add_vector_row(motor_angular_velocity_ref, False)

        # End line
        self.file.write('\n')

    def close(self) -> None:
        """Close the csv file."""
        self.file.close()
