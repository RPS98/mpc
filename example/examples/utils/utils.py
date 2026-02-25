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

from dataclasses import dataclass, field
import ctypes
import math
import os
import sys
import importlib.util

import numpy as np
import yaml


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convert Euler angles to a quaternion.

    :param roll: Roll angle in radians
    :type roll: float
    :param pitch: Pitch angle in radians
    :type pitch: float
    :param yaw: Yaw angle in radians
    :type yaw: float
    :return: Quaternion [w, x, y, z]
    :rtype: np.ndarray
    """
    roll_half = roll * 0.5
    pitch_half = pitch * 0.5
    yaw_half = yaw * 0.5

    sr = math.sin(roll_half)
    cr = math.cos(roll_half)
    sp = math.sin(pitch_half)
    cp = math.cos(pitch_half)
    sy = math.sin(yaw_half)
    cy = math.cos(yaw_half)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    q = np.array([w, x, y, z], dtype=float)
    return q / np.linalg.norm(q)


def quaternion_to_euler(q: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw).

    :param q: Quaternion [w, x, y, z]
    :type q: np.ndarray
    :return: Euler angles [roll, pitch, yaw] in radians
    :rtype: np.ndarray
    """
    w, x, y, z = q

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw], dtype=float)


def compute_path_facing(current_position: np.ndarray,
                        target_position: np.ndarray = None,
                        current_orientation: np.ndarray = None) -> np.ndarray:
    """
    Compute path-facing orientation.

    This function supports two modes:
    - C++-like mode: ``compute_path_facing(current_position, target_position, current_orientation)``
    - Legacy mode: ``compute_path_facing(velocity)``

    :param current_position: Current position or velocity (legacy mode)
    :type current_position: np.ndarray
    :param target_position: Target waypoint position
    :type target_position: np.ndarray
    :param current_orientation: Current orientation quaternion [w, x, y, z]
    :type current_orientation: np.ndarray
    :return: Quaternion [w, x, y, z]
    :rtype: np.ndarray
    """
    if target_position is None:
        velocity = np.asarray(current_position, dtype=float)
        yaw = math.atan2(velocity[1], velocity[0])
        return euler_to_quaternion(0.0, 0.0, yaw)

    current_position = np.asarray(current_position, dtype=float)
    target_position = np.asarray(target_position, dtype=float)

    x_diff = target_position[0] - current_position[0]
    y_diff = target_position[1] - current_position[1]
    diff = np.array([x_diff, y_diff], dtype=float)

    if np.linalg.norm(diff) < 0.1 and current_orientation is not None:
        return np.asarray(current_orientation, dtype=float).copy()

    yaw_target = math.atan2(y_diff, x_diff)
    return euler_to_quaternion(0.0, 0.0, yaw_target)


@dataclass
class YamlMPCData:
    """Container for MPC data loaded from yaml."""

    dt: float = 0.0
    Q: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    Qe: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    R: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    lbu: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    ubu: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    lbx: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    ubx: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    lsbx: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    usbx: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    Zl: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    Zu: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    zl: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    zu: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    Zl_e: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    Zu_e: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    zl_e: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    zu_e: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    p: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))


@dataclass
class YamlData:
    """Container for example yaml data."""

    sim_time: float = 0.0
    max_speed: float = 0.0
    waypoints: np.ndarray = field(default_factory=lambda: np.zeros((0, 3), dtype=float))
    path_facing: bool = False
    solver_definition_path: str = ''
    ocp_json_file_path: str = ''
    mpc_data: YamlMPCData = field(default_factory=YamlMPCData)


def _to_float_array(values, default=None) -> np.ndarray:
    """
    Convert list/scalar to float numpy array.

    :param values: Input value
    :type values: Any
    :param default: Fallback value if input is None
    :type default: Any
    :return: Array of float
    :rtype: np.ndarray
    """
    if values is None:
        values = default
    if values is None:
        return np.array([], dtype=float)
    if isinstance(values, (int, float)):
        return np.array([values], dtype=float)
    return np.asarray(values, dtype=float)


def read_yaml_params(file_path: str) -> YamlData:
    """
    Read yaml parameters in the same structure as C++ example utility.

    :param file_path: Path to yaml file
    :type file_path: str
    :return: Parsed yaml data
    :rtype: YamlData
    """
    if not os.path.isfile(file_path):
        absolute_simulation_config_path = os.path.abspath(file_path)
        raise ValueError(f'File {absolute_simulation_config_path} does not exist.')

    with open(file_path, 'r') as f:
        config = yaml.safe_load(f)

    sim_cfg = config.get('sim_config', {})
    controller_cfg = config.get('controller', {})
    mpc_cfg = controller_cfg.get('mpc', {})
    cost_cfg = mpc_cfg.get('cost', {})
    constraints_cfg = mpc_cfg.get('constraints', {})
    params_cfg = mpc_cfg.get('parameters', {})

    waypoints_list = sim_cfg.get('waypoints', [])
    waypoints = np.asarray(waypoints_list, dtype=float).reshape(-1, 3) if waypoints_list else np.zeros((0, 3), dtype=float)

    Q = _to_float_array(cost_cfg.get('Q', []))
    Qe = _to_float_array(cost_cfg.get('Qe', []))
    R = _to_float_array(cost_cfg.get('R', []))
    lbu = _to_float_array(constraints_cfg.get('lbu', []))
    ubu = _to_float_array(constraints_cfg.get('ubu', []))
    lbx = _to_float_array(constraints_cfg.get('lbx', []))
    ubx = _to_float_array(constraints_cfg.get('ubx', []))
    lsbx = _to_float_array(constraints_cfg.get('lsbx', np.zeros_like(lbx)))
    usbx = _to_float_array(constraints_cfg.get('usbx', np.zeros_like(ubx)))
    Zl = _to_float_array(constraints_cfg.get('Zl', np.zeros_like(lbx)))
    Zu = _to_float_array(constraints_cfg.get('Zu', np.zeros_like(ubx)))
    zl = _to_float_array(constraints_cfg.get('zl', np.zeros_like(lbx)))
    zu = _to_float_array(constraints_cfg.get('zu', np.zeros_like(ubx)))

    Zl_e = _to_float_array(constraints_cfg.get('Zl_e', Zl))
    Zu_e = _to_float_array(constraints_cfg.get('Zu_e', Zu))
    zl_e = _to_float_array(constraints_cfg.get('zl_e', zl))
    zu_e = _to_float_array(constraints_cfg.get('zu_e', zu))

    p = _to_float_array(params_cfg.get('mass', [1.0]))

    mpc_data = YamlMPCData(
        dt=float(sim_cfg.get('dt', 0.0)),
        Q=Q,
        Qe=Qe,
        R=R,
        lbu=lbu,
        ubu=ubu,
        lbx=lbx,
        ubx=ubx,
        lsbx=lsbx,
        usbx=usbx,
        Zl=Zl,
        Zu=Zu,
        zl=zl,
        zu=zu,
        Zl_e=Zl_e,
        Zu_e=Zu_e,
        zl_e=zl_e,
        zu_e=zu_e,
        p=p,
    )

    return YamlData(
        sim_time=float(sim_cfg.get('sim_time', 0.0)),
        max_speed=float(sim_cfg.get('max_speed', 0.0)),
        waypoints=waypoints,
        path_facing=bool(sim_cfg.get('path_facing', False)),
        solver_definition_path=controller_cfg.get('solver_definition_path', ''),
        ocp_json_file_path=controller_cfg.get('ocp_json_file_path', ''),
        mpc_data=mpc_data,
    )


def get_trajectory_generator(initial_position: np.ndarray, waypoints: list, speed: float):
    """
    Initialize and generate a dynamic trajectory.

    :param initial_position: Initial position of the vehicle
    :type initial_position: np.ndarray
    :param waypoints: List of waypoints
    :type waypoints: list
    :param speed: Speed of the trajectory generator
    :type speed: float
    :return: Generated trajectory
    :rtype: Any
    """
    current_dir = os.path.dirname(os.path.abspath(__file__))
    so_file_path = os.path.join(current_dir, 'trajectory_generator.so')

    if not os.path.isfile(so_file_path):
        raise FileNotFoundError(f'Shared library not found at path: {so_file_path}')

    dep_lib_path = os.path.join(current_dir, 'libdynamic_trajectory_generator.so')
    if os.path.isfile(dep_lib_path):
        ctypes.CDLL(dep_lib_path, mode=ctypes.RTLD_GLOBAL)

    mav_lib_path = os.path.join(current_dir, 'libmav_trajectory_generation.so')
    if os.path.isfile(mav_lib_path):
        ctypes.CDLL(mav_lib_path, mode=ctypes.RTLD_GLOBAL)

    if current_dir not in sys.path:
        sys.path.insert(0, current_dir)

    spec = importlib.util.spec_from_file_location('trajectory_generator', so_file_path)
    dtb = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(dtb)

    trajectory_generator = dtb.DynamicTrajectory()
    trajectory_generator.set_path_facing(False)
    trajectory_generator.generate_trajectory(initial_position, 0.0, waypoints, speed)

    max_time = trajectory_generator.get_max_time()
    print(f'Trajectory generated with max time: {max_time}')
    return trajectory_generator


class CsvLogger:
    """Log simulation data to a csv file."""

    def __init__(self, file_name: str) -> None:
        """
        Initialize csv logger.

        :param file_name: Name of the file to save the data
        :type file_name: str
        :return: None
        :rtype: None
        """
        self.file_name = file_name
        print(f'Saving to file: {self.file_name}')
        self.file = open(self.file_name, 'w')
        self.file.write(
            'time,'
            'x,y,z,qw,qx,qy,qz,roll,pitch,yaw,vx,vy,vz,'
            'x_ref,y_ref,z_ref,qw_ref,qx_ref,qy_ref,qz_ref,roll_ref,pitch_ref,yaw_ref,vx_ref,vy_ref,vz_ref,'
            'thrust,wx,wy,wz\n'
        )

    def add_double(self, data: float, add_final_comma: bool = True) -> None:
        """
        Add scalar data to the csv file.

        :param data: Scalar value
        :type data: float
        :param add_final_comma: Add trailing comma
        :type add_final_comma: bool
        :return: None
        :rtype: None
        """
        if data is None:
            raise ValueError('Data is None')
        self.file.write(f'{data}')
        if add_final_comma:
            self.file.write(',')

    def add_vector_row(self, data: np.ndarray, add_final_comma: bool = True) -> None:
        """
        Add vector data to the csv file.

        :param data: Vector data
        :type data: np.ndarray
        :param add_final_comma: Add trailing comma
        :type add_final_comma: bool
        :return: None
        :rtype: None
        """
        data = np.asarray(data).reshape(-1)
        for i in range(data.size):
            if data[i] is None:
                raise ValueError('Data is None')
            self.file.write(f'{data[i]}')
            if i < data.size - 1:
                self.file.write(',')
            elif add_final_comma:
                self.file.write(',')

    def save(self, time_value: float, mpc_data) -> None:
        """
        Save simulation data to the csv file.

        :param time_value: Current simulation time
        :type time_value: float
        :param mpc_data: MPCData object
        :type mpc_data: Any
        :return: None
        :rtype: None
        """
        self.add_double(time_value)

        x = np.asarray(mpc_data.state.vector).reshape(-1)
        x_pos = x[0:3]
        q = x[3:7]
        euler_q = quaternion_to_euler(q)
        v = x[7:10]

        self.add_vector_row(x_pos)
        self.add_vector_row(q)
        self.add_vector_row(euler_q)
        self.add_vector_row(v)

        y_pos = np.asarray(mpc_data.parameters.desired_position).reshape(-1)
        q_ref = np.asarray(mpc_data.parameters.desired_orientation).reshape(-1)
        euler_q_ref = quaternion_to_euler(q_ref)
        v_ref = np.zeros(3)

        self.add_vector_row(y_pos)
        self.add_vector_row(q_ref)
        self.add_vector_row(euler_q_ref)
        self.add_vector_row(v_ref)

        u = np.asarray(mpc_data.actuation.vector).reshape(-1)
        self.add_vector_row(u, add_final_comma=False)
        self.file.write('\n')

    def close(self) -> None:
        """Close the csv file."""
        self.file.close()
