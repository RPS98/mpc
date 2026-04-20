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
"""Common utilities for the drone model."""

__authors__ = 'Rafael Perez-Segui'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from typing import ClassVar, List, Union

import casadi as ca
import numpy as np


class VectorBase:
    """Vector representation of data."""

    _type: ClassVar[str]
    _names: ClassVar[List[str]]
    _sizes: ClassVar[List[int]]
    _names_sx: ClassVar[List[str]]

    def __init__(self):
        """Initialize the vector base class for manage data as a vector."""
        total_size = sum(self._sizes)

        if self._type == 'ca.SX':
            segments = []
            offset = 0
            for name, sx_name, size in zip(self._names, self._names_sx, self._sizes):
                sym = ca.SX.sym(sx_name, size)
                segments.append(sym)

                def getter(self, o=offset, s=size):
                    return self._vector[o:o + s]

                def setter(self, value, o=offset, s=size):
                    if not isinstance(value, ca.SX):
                        raise TypeError('Expected ca.SX')
                    if value.size() != (s, 1):
                        raise ValueError(f'Expected size {(s, 1)}, got {value.size()}')
                    # Reconstruct the vector with the new segment
                    new_segments = []
                    current_offset = 0
                    for i, (n, sz) in enumerate(zip(self._names, self._sizes)):
                        if current_offset == o:
                            new_segments.append(value)
                        else:
                            new_segments.append(self._vector[current_offset:current_offset + sz])
                        current_offset += sz
                    self._vector = ca.vertcat(*new_segments)

                setattr(self.__class__, name, property(getter, setter))
                offset += size

            self._vector = ca.vertcat(*segments)

        elif self._type == 'ca.DM':
            self._vector = ca.DM.zeros(total_size)
            offset = 0
            for name, size in zip(self._names, self._sizes):
                if size == 1:
                    def getter(self, o=offset):
                        return float(self._vector[o])

                    def setter(self, value, o=offset):
                        self._vector[o] = float(value)
                else:
                    def getter(self, o=offset, s=size):
                        return self._vector[o:o + s]

                    def setter(self, value, o=offset, s=size):
                        # value = ca.atleast_1d(value).astype(ca.DM.float64)
                        if value.shape != (s, 1):
                            raise ValueError(f'Expected shape {(s,1)}, got {value.shape}')
                        self._vector[o:o + s] = value

                setattr(self.__class__, name, property(getter, setter))
                offset += size
        elif self._type == 'np.array':
            self._vector = np.zeros(total_size)
            offset = 0
            for name, size in zip(self._names, self._sizes):
                if size == 1:
                    def getter(self, o=offset):
                        return self._vector[o]

                    def setter(self, value, o=offset):
                        self._vector[o] = value
                else:
                    def getter(self, o=offset, s=size):
                        return self._vector[o:o + s]

                    def setter(self, value, o=offset, s=size):
                        value = np.atleast_1d(value).astype(float)
                        if value.shape != (s,):
                            raise ValueError(f'Expected shape {(s,)}, got {value.shape}')
                        self._vector[o:o + s] = value

                setattr(self.__class__, name, property(getter, setter))
                offset += size

    @property
    def vector(self) -> Union[ca.SX, ca.DM]:
        """Get the vector representation of the state."""
        return self._vector

    @vector.setter
    def vector(self, value: Union[ca.SX, ca.DM, np.ndarray]):
        """Set the vector representation of the state."""
        expected_size = sum(self._sizes)
        if self._type == 'ca.SX':
            if not isinstance(value, ca.SX):
                raise TypeError('Expected ca.SX vector')
            if value.size() != expected_size:
                raise ValueError(f'Expected vector size {expected_size}, got {value.size()}')
        elif self._type == 'np.array':
            if not isinstance(value, np.ndarray):
                raise TypeError('Expected np.array vector')
            if value.shape != (expected_size,):
                raise ValueError(f'Expected vector shape {(expected_size,)}, got {value.shape}')
        else:
            if not isinstance(value, ca.DM):
                raise TypeError(f'Expected ca.DM vector, got {type(value)}')
            if value.shape[0] != expected_size:
                raise ValueError(f'Expected vector size {expected_size}, got {value.shape[0]}')
        self._vector[:] = value

    @property
    def size(self) -> int:
        """Get the total size of the vector."""
        return sum(self._sizes)


if __name__ == '__main__':
    class TestStateSpec:
        """Test state specification."""

        _names: ClassVar[List[str]] = [
            'position', 'orientation', 'linear_velocity', 'angular_velocity',
            'linear_acceleration', 'angular_acceleration', 'force', 'torque',
            'motor_angular_velocity', 'motor_angular_acceleration'
        ]
        _names_sx: ClassVar[List[str]] = [
            'x_p', 'x_q', 'x_v', 'x_w', 'x_a', 'x_alpha',
            'x_force', 'x_torque', 'x_u_w', 'x_u_alpha'
        ]
        _sizes: ClassVar[List[int]] = [3, 4, 3, 3, 3, 3, 3, 3, 4, 4]
        position: Union[ca.SX, ca.DM]
        orientation: Union[ca.SX, ca.DM]
        linear_velocity: Union[ca.SX, ca.DM]
        angular_velocity: Union[ca.SX, ca.DM]
        linear_acceleration: Union[ca.SX, ca.DM]
        angular_acceleration: Union[ca.SX, ca.DM]
        force: Union[ca.SX, ca.DM]
        torque: Union[ca.SX, ca.DM]
        motor_angular_velocity: Union[ca.SX, ca.DM]
        motor_angular_acceleration: Union[ca.SX, ca.DM]

    class TestCaState(TestStateSpec, VectorBase):
        """Test CasADi state."""

        _type = 'ca.SX'

    class TestState(TestStateSpec, VectorBase):
        """Test DM state."""

        _type = 'ca.DM'

        def __init__(
                self,
                position: ca.DM = ca.DM.zeros(3),
                orientation: ca.DM = ca.DM([1.0, 0.0, 0.0, 0.0]),
                linear_velocity: ca.DM = ca.DM.zeros(3),
                angular_velocity: ca.DM = ca.DM.zeros(3),
                linear_acceleration: ca.DM = ca.DM.zeros(3),
                angular_acceleration: ca.DM = ca.DM.zeros(3),
                force: ca.DM = ca.DM.zeros(3),
                torque: ca.DM = ca.DM.zeros(3),
                motor_angular_velocity: ca.DM = ca.DM.zeros(4),
                motor_angular_acceleration: ca.DM = ca.DM.zeros(4)):
            """Initialize the state test."""
            super().__init__()
            self.position = position
            self.orientation = orientation
            self.linear_velocity = linear_velocity
            self.angular_velocity = angular_velocity
            self.linear_acceleration = linear_acceleration
            self.angular_acceleration = angular_acceleration
            self.force = force
            self.torque = torque
            self.motor_angular_velocity = motor_angular_velocity
            self.motor_angular_acceleration = motor_angular_acceleration

    class TestStateNP(TestStateSpec, VectorBase):
        """Test NumPy state."""

        _type = 'np.array'

        def __init__(
                self,
                position: np.array = np.zeros(3),
                orientation: np.array = np.array([1.0, 0.0, 0.0, 0.0]),
                linear_velocity: np.array = np.zeros(3),
                angular_velocity: np.array = np.zeros(3),
                linear_acceleration: np.array = np.zeros(3),
                angular_acceleration: np.array = np.zeros(3),
                force: np.array = np.zeros(3),
                torque: np.array = np.zeros(3),
                motor_angular_velocity: np.array = np.zeros(4),
                motor_angular_acceleration: np.array = np.zeros(4)):
            """Initialize the state test."""
            super().__init__()
            self.position = position
            self.orientation = orientation
            self.linear_velocity = linear_velocity
            self.angular_velocity = angular_velocity
            self.linear_acceleration = linear_acceleration
            self.angular_acceleration = angular_acceleration
            self.force = force
            self.torque = torque
            self.motor_angular_velocity = motor_angular_velocity
            self.motor_angular_acceleration = motor_angular_acceleration

    print('Testing CasADi Base class')
    ca_state = TestCaState()
    print('Position:', ca_state.position)
    print('Orientation:', ca_state.orientation)
    print('Linear Velocity:', ca_state.linear_velocity)
    print('Angular Velocity:', ca_state.angular_velocity)
    print('Linear Acceleration:', ca_state.linear_acceleration)
    print('Angular Acceleration:', ca_state.angular_acceleration)
    print('Force:', ca_state.force)
    print('Torque:', ca_state.torque)
    print('Motor Angular Velocity:', ca_state.motor_angular_velocity)
    print('Motor Angular Acceleration:', ca_state.motor_angular_acceleration)
    print('Vector:', ca_state.vector)
    print('Vector Size:', ca_state.vector.size())
    print('Vector Shape:', ca_state.vector.shape)
    print('Vector Type:', type(ca_state.vector))

    ca_pos = ca.SX.sym('test_pos', 3)
    ca_state.position = ca_pos
    print('Updated Position:', ca_state.position)
    print('Vector:', ca_state.vector)
    print('Vector Size:', ca_state.vector.size())
    print('Vector Shape:', ca_state.vector.shape)
    print('Vector Type:', type(ca_state.vector))

    print('Testing NumPy Base class')
    dm_state = TestState(
        position=ca.DM([1.0, 2.0, 3.0]),
        orientation=ca.DM([0.707, 0.0, 0.707, 0.0]))

    print(dm_state.position[0])
    print(dm_state.vector[0])

    vector_copy = ca.DM(dm_state.vector)
    vector_copy[0] = 10.0  # Modify the copy
    dm_state.vector = vector_copy  # Set the modified vector
    print(dm_state.position[0])
    print(dm_state.vector[0])

    print('Position:', dm_state.position)
    print('Orientation:', dm_state.orientation)
    print('Linear Velocity:', dm_state.linear_velocity)
    print('Angular Velocity:', dm_state.angular_velocity)
    print('Linear Acceleration:', dm_state.linear_acceleration)
    print('Angular Acceleration:', dm_state.angular_acceleration)
    print('Force:', dm_state.force)
    print('Torque:', dm_state.torque)
    print('Motor Angular Velocity:', dm_state.motor_angular_velocity)
    print('Motor Angular Acceleration:', dm_state.motor_angular_acceleration)
    print('Vector:', dm_state.vector)
    print('Vector Size:', dm_state.vector.size)
    print('Vector Shape:', dm_state.vector.shape)

    np_state = TestStateNP(
        position=np.array([1.0, 2.0, 3.0]),
        orientation=np.array([0.707, 0.0, 0.707, 0.0]))

    print('Position:', np_state.position)
    print('Orientation:', np_state.orientation)
    print('Linear Velocity:', np_state.linear_velocity)
    print('Angular Velocity:', np_state.angular_velocity)
    print('Linear Acceleration:', np_state.linear_acceleration)
    print('Angular Acceleration:', np_state.angular_acceleration)
    print('Force:', np_state.force)
    print('Torque:', np_state.torque)
    print('Motor Angular Velocity:', np_state.motor_angular_velocity)
    print('Motor Angular Acceleration:', np_state.motor_angular_acceleration)
    print('Vector:', np_state.vector)
    print('Vector Size:', np_state.vector.size)
    print('Vector Shape:', np_state.vector.shape)
