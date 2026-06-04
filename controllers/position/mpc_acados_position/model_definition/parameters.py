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

from mpc_acados_core.utils.datatypes_utils import VectorBase
import casadi as ca
import numpy as np


class _ParametersDef:
    """Parameters definition for MAV."""

    _names: ClassVar[List[str]] = [
        'mass',
        'desired_position',
        'desired_velocity',
        'desired_orientation',
        'external_force',
        'Q',
        'Qe',
        'R',
        'p_anchor',
        'w_cte'
    ]
    _names_sx: ClassVar[List[str]] = [
        'sx_mass',
        'sx_desired_position',
        'sx_desired_velocity',
        'sx_desired_orientation',
        'sx_external_force',
        'sx_Q',
        'sx_Qe',
        'sx_R',
        'sx_p_anchor',
        'sx_w_cte'
    ]
    _sizes: ClassVar[List[int]] = [
        1,
        3,
        3,
        4,
        3,
        9,
        9,
        4,
        3,
        1
    ]

    mass: Union[ca.SX, ca.DM]
    desired_position: Union[ca.SX, ca.DM]
    desired_velocity: Union[ca.SX, ca.DM]
    desired_orientation: Union[ca.SX, ca.DM]
    external_force: Union[ca.SX, ca.DM]
    Q: Union[ca.SX, ca.DM]
    Qe: Union[ca.SX, ca.DM]
    R: Union[ca.SX, ca.DM]
    p_anchor: Union[ca.SX, ca.DM]
    w_cte: Union[ca.SX, ca.DM]


class CaParameters(_ParametersDef, VectorBase):
    """
    CasADi SX Parameters for the UAV.

    :param mass: Mass of the MAV (kg)
    :type mass: ca.SX
    :param desired_position: Desired position in world frame [x, y, z] (m)
    :type desired_position: ca.SX
    :param desired_velocity: Desired linear velocity in world frame [vx, vy, vz] (m/s)
    :type desired_velocity: ca.SX
    :param desired_orientation: Desired orientation as a quaternion [qw, qx, qy, qz]
    :type desired_orientation: ca.SX
    :param external_force: External force acting on the MAV in base frame [fx, fy, fz] (N)
    :type external_force: ca.SX
    :param Q: Stage gains for [x, y, z, roll, pitch, yaw, vx, vy, vz]
    :type Q: ca.SX
    :param Qe: Terminal gains for [x, y, z, roll, pitch, yaw, vx, vy, vz]
    :type Qe: ca.SX
    :param R: Stage gains for control inputs [thrust, wx, wy, wz]
    :type R: ca.SX
    :param p_anchor: CTE cost anchor: line origin between p_anchor and desired_position [x, y, z] (m)
    :type p_anchor: ca.SX
    :param w_cte: CTE cost weight; penalises lateral distance from the anchor->ref line (0 = disabled)
    :type w_cte: ca.SX
    """

    _type = 'ca.SX'


class Parameters(_ParametersDef, VectorBase):
    """Parameters for the UAV Model."""

    _type = 'np.array'

    def __init__(
            self,
            mass: np.array = np.array(1.0),
            desired_position: np.array = np.array([0.0, 0.0, 0.0]),
            desired_velocity: np.array = np.array([0.0, 0.0, 0.0]),
            desired_orientation: np.array = np.array([1.0, 0.0, 0.0, 0.0]),
            external_force: np.array = np.array([0.0, 0.0, 0.0]),
            Q: np.array = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            Qe: np.array = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            R: np.array = np.array([0.0, 0.0, 0.0, 0.0]),
            p_anchor: np.array = np.array([0.0, 0.0, 0.0]),
            w_cte: np.array = np.array(0.0)):
        """
        Parameters for the UAV Model.

        :param mass: Mass of the MAV (kg)
        :type mass: np.array
        :param desired_position: Desired position in world frame [x, y, z] (m)
        :type desired_position: np.array
        :param desired_velocity: Desired linear velocity in world frame [vx, vy, vz] (m/s)
        :type desired_velocity: np.array
        :param desired_orientation: Desired orientation as a quaternion [qw, qx, qy, qz]
        :type desired_orientation: np.array
        :param external_force: External force acting on the MAV in base frame [fx, fy, fz] (N)
        :type external_force: np.array
        :param Q: Stage gains for [x, y, z, roll, pitch, yaw, vx, vy, vz]
        :type Q: np.array
        :param Qe: Terminal gains for [x, y, z, roll, pitch, yaw, vx, vy, vz]
        :type Qe: np.array
        :param R: Stage gains for control inputs [thrust, wx, wy, wz]
        :type R: np.array
        :param p_anchor: CTE cost anchor: line origin between p_anchor and desired_position [x, y, z] (m)
        :type p_anchor: np.array
        :param w_cte: CTE cost weight; penalises lateral distance from the anchor->ref line (0 = disabled)
        :type w_cte: np.array
        """
        super().__init__()
        self.mass = mass
        self.desired_position = desired_position
        self.desired_velocity = desired_velocity
        self.desired_orientation = desired_orientation
        self.external_force = external_force
        self.Q = Q
        self.Qe = Qe
        self.R = R
        self.p_anchor = p_anchor
        self.w_cte = w_cte


class OnlineParameters:
    """Parameters for all MPC stages."""

    _names: ClassVar[List[str]] = _ParametersDef._names
    _names_sx: ClassVar[List[str]] = _ParametersDef._names_sx
    _sizes: ClassVar[List[int]] = _ParametersDef._sizes
    Np: ClassVar[int] = 40

    mass_offset: ClassVar[int] = 0
    mass_length: ClassVar[int] = 1
    desired_position_offset: ClassVar[int] = 1
    desired_position_length: ClassVar[int] = 3
    desired_velocity_offset: ClassVar[int] = 4
    desired_velocity_length: ClassVar[int] = 3
    desired_orientation_offset: ClassVar[int] = 7
    desired_orientation_length: ClassVar[int] = 4
    external_force_offset: ClassVar[int] = 11
    external_force_length: ClassVar[int] = 3
    Q_offset: ClassVar[int] = 14
    Q_length: ClassVar[int] = 9
    Qe_offset: ClassVar[int] = 23
    Qe_length: ClassVar[int] = 9
    R_offset: ClassVar[int] = 32
    R_length: ClassVar[int] = 4
    p_anchor_offset: ClassVar[int] = 36
    p_anchor_length: ClassVar[int] = 3
    w_cte_offset: ClassVar[int] = 39
    w_cte_length: ClassVar[int] = 1

    def __init__(
            self,
            num_stages: int = 1,
            mass: np.array = np.array(1.0),
            desired_position: np.array = np.array([0.0, 0.0, 0.0]),
            desired_velocity: np.array = np.array([0.0, 0.0, 0.0]),
            desired_orientation: np.array = np.array([1.0, 0.0, 0.0, 0.0]),
            external_force: np.array = np.array([0.0, 0.0, 0.0]),
            Q: np.array = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            Qe: np.array = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            R: np.array = np.array([0.0, 0.0, 0.0, 0.0]),
            p_anchor: np.array = np.array([0.0, 0.0, 0.0]),
            w_cte: np.array = np.array(0.0)):
        """
        Parameters for all MPC stages.

        :param num_stages: Number of stored stages.
        :type num_stages: int

        :param mass: Mass of the MAV (kg)
        :type mass: np.array
        :param desired_position: Desired position in world frame [x, y, z] (m)
        :type desired_position: np.array
        :param desired_velocity: Desired linear velocity in world frame [vx, vy, vz] (m/s)
        :type desired_velocity: np.array
        :param desired_orientation: Desired orientation as a quaternion [qw, qx, qy, qz]
        :type desired_orientation: np.array
        :param external_force: External force acting on the MAV in base frame [fx, fy, fz] (N)
        :type external_force: np.array
        :param Q: Stage gains for [x, y, z, roll, pitch, yaw, vx, vy, vz]
        :type Q: np.array
        :param Qe: Terminal gains for [x, y, z, roll, pitch, yaw, vx, vy, vz]
        :type Qe: np.array
        :param R: Stage gains for control inputs [thrust, wx, wy, wz]
        :type R: np.array
        :param p_anchor: CTE cost anchor: line origin between p_anchor and desired_position [x, y, z] (m)
        :type p_anchor: np.array
        :param w_cte: CTE cost weight; penalises lateral distance from the anchor->ref line (0 = disabled)
        :type w_cte: np.array
        """
        if num_stages < 1:
            raise ValueError('num_stages must be greater than zero')
        self.num_stages = int(num_stages)
        stage_parameters = Parameters(
            mass=mass,
            desired_position=desired_position,
            desired_velocity=desired_velocity,
            desired_orientation=desired_orientation,
            external_force=external_force,
            Q=Q,
            Qe=Qe,
            R=R,
            p_anchor=p_anchor,
            w_cte=w_cte
        )
        self.data = np.tile(stage_parameters.vector, (self.num_stages, 1))

    def _check_stage(self, stage: int) -> None:
        """Validate a stage index."""
        if stage < 0 or stage >= self.num_stages:
            raise IndexError(f'stage={stage} is out of range [0, {self.num_stages}).')

    @property
    def full_vector(self) -> np.ndarray:
        """Get all stage parameters flattened by stage."""
        return self.data.reshape(-1)

    @full_vector.setter
    def full_vector(self, value: np.ndarray) -> None:
        """Set all stage parameters from a flat or 2D array."""
        array = np.asarray(value, dtype=float)
        if array.shape == (self.Np,):
            self.set_parameters(array)
            return
        if array.shape == (self.num_stages * self.Np,):
            self.data[:, :] = array.reshape(self.num_stages, self.Np)
            return
        if array.shape == (self.num_stages, self.Np):
            self.data[:, :] = array
            return
        raise ValueError(
            f"Size mismatch: full_vector has shape {array.shape}, "
            f"but expected ({self.Np},), ({self.num_stages * self.Np},) "
            f"or ({self.num_stages}, {self.Np}).")

    @property
    def size(self) -> int:
        """Get the total number of stored scalar values."""
        return self.num_stages * self.Np

    def get_data(self, stage: int = 0) -> np.ndarray:
        """Get the raw parameter vector of one stage."""
        self._check_stage(stage)
        return self.data[stage, :]

    def get_parameters(self, stage: int = 0) -> Parameters:
        """Get one stage as a Parameters object."""
        self._check_stage(stage)
        parameters = Parameters()
        parameters.vector = self.data[stage, :].copy()
        return parameters

    def get_online_parameters(self) -> np.ndarray:
        """Get all stage parameters flattened by stage."""
        return self.full_vector.copy()

    def set_parameters(self, params, stage: int = -1) -> None:
        """Set one stage or broadcast parameters to all stages."""
        if isinstance(params, OnlineParameters):
            if params.num_stages != self.num_stages:
                raise ValueError(
                    f'num_stages mismatch: got {params.num_stages}, expected {self.num_stages}.')
            if stage == -1:
                self.data[:, :] = params.data
            else:
                self._check_stage(stage)
                self.data[stage, :] = params.data[stage, :]
            return

        if isinstance(params, Parameters):
            params = params.vector

        array = np.asarray(params, dtype=float)
        if array.shape == (self.Np,):
            if stage == -1:
                self.data[:, :] = array
            else:
                self._check_stage(stage)
                self.data[stage, :] = array
            return
        if array.shape == (self.num_stages * self.Np,):
            reshaped = array.reshape(self.num_stages, self.Np)
            if stage == -1:
                self.data[:, :] = reshaped
            else:
                self._check_stage(stage)
                self.data[stage, :] = reshaped[stage, :]
            return
        if array.shape == (self.num_stages, self.Np):
            if stage == -1:
                self.data[:, :] = array
            else:
                self._check_stage(stage)
                self.data[stage, :] = array[stage, :]
            return
        raise ValueError(
            f"Size mismatch: params has shape {array.shape}, "
            f"but expected ({self.Np},), ({self.num_stages * self.Np},) "
            f"or ({self.num_stages}, {self.Np}).")

    def set_mass(self, value: np.array, stage: int = -1) -> None:
        """Set mass for one stage or all stages."""
        scalar_value = float(np.asarray(value, dtype=float).reshape(()))
        if stage == -1:
            self.data[:, self.mass_offset] = scalar_value
            return
        self._check_stage(stage)
        self.data[stage, self.mass_offset] = scalar_value

    def get_mass(self, stage: int = 0):
        """Get mass for one stage."""
        self._check_stage(stage)
        return float(self.data[stage, self.mass_offset])

    def set_desired_position(self, value: np.array, stage: int = -1) -> None:
        """Set desired_position for one stage or all stages."""
        vector_value = np.atleast_1d(value).astype(float)
        if vector_value.shape != (self.desired_position_length,):
            raise ValueError(
                f"Size mismatch: desired_position has shape {vector_value.shape}, "
                f"but expected ({self.desired_position_length},).")
        start = self.desired_position_offset
        end = start + self.desired_position_length
        if stage == -1:
            self.data[:, start:end] = vector_value
            return
        self._check_stage(stage)
        self.data[stage, start:end] = vector_value

    def get_desired_position(self, stage: int = 0):
        """Get desired_position for one stage."""
        self._check_stage(stage)
        start = self.desired_position_offset
        end = start + self.desired_position_length
        return self.data[stage, start:end].copy()

    def set_desired_velocity(self, value: np.array, stage: int = -1) -> None:
        """Set desired_velocity for one stage or all stages."""
        vector_value = np.atleast_1d(value).astype(float)
        if vector_value.shape != (self.desired_velocity_length,):
            raise ValueError(
                f"Size mismatch: desired_velocity has shape {vector_value.shape}, "
                f"but expected ({self.desired_velocity_length},).")
        start = self.desired_velocity_offset
        end = start + self.desired_velocity_length
        if stage == -1:
            self.data[:, start:end] = vector_value
            return
        self._check_stage(stage)
        self.data[stage, start:end] = vector_value

    def get_desired_velocity(self, stage: int = 0):
        """Get desired_velocity for one stage."""
        self._check_stage(stage)
        start = self.desired_velocity_offset
        end = start + self.desired_velocity_length
        return self.data[stage, start:end].copy()

    def set_desired_orientation(self, value: np.array, stage: int = -1) -> None:
        """Set desired_orientation for one stage or all stages."""
        vector_value = np.atleast_1d(value).astype(float)
        if vector_value.shape != (self.desired_orientation_length,):
            raise ValueError(
                f"Size mismatch: desired_orientation has shape {vector_value.shape}, "
                f"but expected ({self.desired_orientation_length},).")
        start = self.desired_orientation_offset
        end = start + self.desired_orientation_length
        if stage == -1:
            self.data[:, start:end] = vector_value
            return
        self._check_stage(stage)
        self.data[stage, start:end] = vector_value

    def get_desired_orientation(self, stage: int = 0):
        """Get desired_orientation for one stage."""
        self._check_stage(stage)
        start = self.desired_orientation_offset
        end = start + self.desired_orientation_length
        return self.data[stage, start:end].copy()

    def set_external_force(self, value: np.array, stage: int = -1) -> None:
        """Set external_force for one stage or all stages."""
        vector_value = np.atleast_1d(value).astype(float)
        if vector_value.shape != (self.external_force_length,):
            raise ValueError(
                f"Size mismatch: external_force has shape {vector_value.shape}, "
                f"but expected ({self.external_force_length},).")
        start = self.external_force_offset
        end = start + self.external_force_length
        if stage == -1:
            self.data[:, start:end] = vector_value
            return
        self._check_stage(stage)
        self.data[stage, start:end] = vector_value

    def get_external_force(self, stage: int = 0):
        """Get external_force for one stage."""
        self._check_stage(stage)
        start = self.external_force_offset
        end = start + self.external_force_length
        return self.data[stage, start:end].copy()

    def set_Q(self, value: np.array, stage: int = -1) -> None:
        """Set Q for one stage or all stages."""
        vector_value = np.atleast_1d(value).astype(float)
        if vector_value.shape != (self.Q_length,):
            raise ValueError(
                f"Size mismatch: Q has shape {vector_value.shape}, "
                f"but expected ({self.Q_length},).")
        start = self.Q_offset
        end = start + self.Q_length
        if stage == -1:
            self.data[:, start:end] = vector_value
            return
        self._check_stage(stage)
        self.data[stage, start:end] = vector_value

    def get_Q(self, stage: int = 0):
        """Get Q for one stage."""
        self._check_stage(stage)
        start = self.Q_offset
        end = start + self.Q_length
        return self.data[stage, start:end].copy()

    def set_Qe(self, value: np.array, stage: int = -1) -> None:
        """Set Qe for one stage or all stages."""
        vector_value = np.atleast_1d(value).astype(float)
        if vector_value.shape != (self.Qe_length,):
            raise ValueError(
                f"Size mismatch: Qe has shape {vector_value.shape}, "
                f"but expected ({self.Qe_length},).")
        start = self.Qe_offset
        end = start + self.Qe_length
        if stage == -1:
            self.data[:, start:end] = vector_value
            return
        self._check_stage(stage)
        self.data[stage, start:end] = vector_value

    def get_Qe(self, stage: int = 0):
        """Get Qe for one stage."""
        self._check_stage(stage)
        start = self.Qe_offset
        end = start + self.Qe_length
        return self.data[stage, start:end].copy()

    def set_R(self, value: np.array, stage: int = -1) -> None:
        """Set R for one stage or all stages."""
        vector_value = np.atleast_1d(value).astype(float)
        if vector_value.shape != (self.R_length,):
            raise ValueError(
                f"Size mismatch: R has shape {vector_value.shape}, "
                f"but expected ({self.R_length},).")
        start = self.R_offset
        end = start + self.R_length
        if stage == -1:
            self.data[:, start:end] = vector_value
            return
        self._check_stage(stage)
        self.data[stage, start:end] = vector_value

    def get_R(self, stage: int = 0):
        """Get R for one stage."""
        self._check_stage(stage)
        start = self.R_offset
        end = start + self.R_length
        return self.data[stage, start:end].copy()

    def set_p_anchor(self, value: np.array, stage: int = -1) -> None:
        """Set p_anchor for one stage or all stages."""
        vector_value = np.atleast_1d(value).astype(float)
        if vector_value.shape != (self.p_anchor_length,):
            raise ValueError(
                f"Size mismatch: p_anchor has shape {vector_value.shape}, "
                f"but expected ({self.p_anchor_length},).")
        start = self.p_anchor_offset
        end = start + self.p_anchor_length
        if stage == -1:
            self.data[:, start:end] = vector_value
            return
        self._check_stage(stage)
        self.data[stage, start:end] = vector_value

    def get_p_anchor(self, stage: int = 0):
        """Get p_anchor for one stage."""
        self._check_stage(stage)
        start = self.p_anchor_offset
        end = start + self.p_anchor_length
        return self.data[stage, start:end].copy()

    def set_w_cte(self, value: np.array, stage: int = -1) -> None:
        """Set w_cte for one stage or all stages."""
        scalar_value = float(np.asarray(value, dtype=float).reshape(()))
        if stage == -1:
            self.data[:, self.w_cte_offset] = scalar_value
            return
        self._check_stage(stage)
        self.data[stage, self.w_cte_offset] = scalar_value

    def get_w_cte(self, stage: int = 0):
        """Get w_cte for one stage."""
        self._check_stage(stage)
        return float(self.data[stage, self.w_cte_offset])

if __name__ == '__main__':
    # Test the Parameters class
    parameters = Parameters()
    print('CasADi SX:', parameters._names)
    print('CasADi SX Sizes:', parameters._sizes)
    print('Vector:', parameters.vector)
    print('mass:', parameters.mass)
    print('desired_position:', parameters.desired_position)
    print('desired_velocity:', parameters.desired_velocity)
    print('desired_orientation:', parameters.desired_orientation)
    print('external_force:', parameters.external_force)
    print('Q:', parameters.Q)
    print('Qe:', parameters.Qe)
    print('R:', parameters.R)
    print('p_anchor:', parameters.p_anchor)
    print('w_cte:', parameters.w_cte)
    print(parameters)

    online_parameters = OnlineParameters(num_stages=2)
    print('Online Vector:', online_parameters.full_vector)
