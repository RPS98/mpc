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
        'external_force'
    ]
    _names_sx: ClassVar[List[str]] = [
        'sx_mass',
        'sx_desired_orientation',
        'sx_external_force'
    ]
    _sizes: ClassVar[List[int]] = [
        1,
        4,
        3
    ]

    mass: Union[ca.SX, ca.DM]
    desired_orientation: Union[ca.SX, ca.DM]
    external_force: Union[ca.SX, ca.DM]


class CaParameters(_ParametersDef, VectorBase):
    """
    CasADi SX Parameters for the UAV.

    :param mass: Mass of the MAV (kg)
    :type mass: ca.SX
    :param desired_orientation: Desired orientation as a quaternion [qw, qx, qy, qz]
    :type desired_orientation: ca.SX
    :param external_force: External force acting on the MAV in base frame [fx, fy, fz] (N)
    :type external_force: ca.SX
    """

    _type = 'ca.SX'


class Parameters(_ParametersDef, VectorBase):
    """Parameters for the UAV Model."""

    _type = 'np.array'

    def __init__(
            self,
            mass: np.array = np.array(1.0),
            desired_orientation: np.array = np.array([1.0, 0.0, 0.0, 0.0]),
            external_force: np.array = np.array([0.0, 0.0, 0.0])):
        """
        Parameters for the UAV Model.

        :param mass: Mass of the MAV (kg)
        :type mass: np.array
        :param desired_orientation: Desired orientation as a quaternion [qw, qx, qy, qz]
        :type desired_orientation: np.array
        :param external_force: External force acting on the MAV in base frame [fx, fy, fz] (N)
        :type external_force: np.array
        """
        super().__init__()
        self.mass = mass
        self.desired_orientation = desired_orientation
        self.external_force = external_force


class OnlineParameters:
    """Parameters for all MPC stages."""

    _names: ClassVar[List[str]] = _ParametersDef._names
    _names_sx: ClassVar[List[str]] = _ParametersDef._names_sx
    _sizes: ClassVar[List[int]] = _ParametersDef._sizes
    Np: ClassVar[int] = 8

    mass_offset: ClassVar[int] = 0
    mass_length: ClassVar[int] = 1
    desired_orientation_offset: ClassVar[int] = 1
    desired_orientation_length: ClassVar[int] = 4
    external_force_offset: ClassVar[int] = 5
    external_force_length: ClassVar[int] = 3

    def __init__(
            self,
            num_stages: int = 1,
            mass: np.array = np.array(1.0),
            desired_orientation: np.array = np.array([1.0, 0.0, 0.0, 0.0]),
            external_force: np.array = np.array([0.0, 0.0, 0.0])):
        """
        Parameters for all MPC stages.

        :param num_stages: Number of stored stages.
        :type num_stages: int

        :param mass: Mass of the MAV (kg)
        :type mass: np.array
        :param desired_orientation: Desired orientation as a quaternion [qw, qx, qy, qz]
        :type desired_orientation: np.array
        :param external_force: External force acting on the MAV in base frame [fx, fy, fz] (N)
        :type external_force: np.array
        """
        if num_stages < 1:
            raise ValueError('num_stages must be greater than zero')
        self.num_stages = int(num_stages)
        stage_parameters = Parameters(
            mass=mass,
            desired_orientation=desired_orientation,
            external_force=external_force
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

if __name__ == '__main__':
    # Test the Parameters class
    parameters = Parameters()
    print('CasADi SX:', parameters._names)
    print('CasADi SX Sizes:', parameters._sizes)
    print('Vector:', parameters.vector)
    print('mass:', parameters.mass)
    print('desired_orientation:', parameters.desired_orientation)
    print('external_force:', parameters.external_force)
    print(parameters)

    online_parameters = OnlineParameters(num_stages=2)
    print('Online Vector:', online_parameters.full_vector)
