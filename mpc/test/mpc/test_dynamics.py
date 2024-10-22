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

"""Test MPC Dynamics unittest."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'


import unittest

from mpc.mpc_controller_lib.drone_model import CaDynamics, CaState, CaControl
import numpy as np
from pyquaternion import Quaternion
import math

def Euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convert Euler angles to a quaternion.

    :param roll (float): The roll angle in radians.
    :param pitch (float): The pitch angle in radians.
    :param yaw (float): The yaw angle in radians.

    :return (np.ndarray): The resulting quaternion.
    """
    # Calculate half angles
    roll_half = roll * 0.5
    pitch_half = pitch * 0.5
    yaw_half = yaw * 0.5

    # Calculate the sine and cosine of the half angles
    sr = math.sin(roll_half)
    cr = math.cos(roll_half)
    sp = math.sin(pitch_half)
    cp = math.cos(pitch_half)
    sy = math.sin(yaw_half)
    cy = math.cos(yaw_half)

    # Calculate the quaternion components
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    # Create the np.array
    q = np.array([w, x, y, z])

    # Normalize the quaternion
    q = q / np.linalg.norm(q)

    return q


class TestCaDynamics(unittest.TestCase):

    def setUp(self):
        self.x = CaState()
        self.u = CaControl()
        self.dynamics = CaDynamics(
            self.x,
            self.u)

    def test_quaternion_derivate(self):
        # Test for each quaternion initial value
        for _ in range(20):  # You can increase/decrease the number of tests here
            # Generate random quaternion values (normalized)
            q = Quaternion(*np.random.rand(4))
            q = q.normalised

            q_np = np.array([q[0], q[1], q[2], q[3]])

            # Generate angular velocity values
            for _ in range(10):
                w = np.random.rand(3)

                # Get the derivative of the quaternion using pyquaternion
                q_dot = q.derivative(w)

                # Get the derivative of the quaternion using the function
                q_dot_func = self.dynamics.quaternion_derivate(q_np, w)

                # Check the results
                self.assertAlmostEqual(q_dot[0], q_dot_func[0])
                self.assertAlmostEqual(q_dot[1], q_dot_func[1])
                self.assertAlmostEqual(q_dot[2], q_dot_func[2])
                self.assertAlmostEqual(q_dot[3], q_dot_func[3])

    def test_quaternion_multiply(self):
        # Test for each quaternion initial value
        for _ in range(20):
            # Generate random quaternion values (normalized)
            q1 = Quaternion(*np.random.rand(4))
            q1 = q1.normalised
            q2 = Quaternion(*np.random.rand(4))
            q2 = q2.normalised

            q1_np = np.array([q1[0], q1[1], q1[2], q1[3]])
            q2_np = np.array([q2[0], q2[1], q2[2], q2[3]])

            # Get the multiplication of the quaternions using pyquaternion
            q_mult = q1 * q2

            # Get the multiplication of the quaternions using the function
            q_mult_func = self.dynamics.quaternion_multiply(q1_np, q2_np)

            # Check the results
            self.assertAlmostEqual(q_mult[0], q_mult_func[0])
            self.assertAlmostEqual(q_mult[1], q_mult_func[1])
            self.assertAlmostEqual(q_mult[2], q_mult_func[2])
            self.assertAlmostEqual(q_mult[3], q_mult_func[3])

    def test_apply_rotation(self):
        # Test for each quaternion initial value
        for _ in range(20):
            # Generate random quaternion values (normalized)
            q = Quaternion(*np.random.rand(4))
            q = q.normalised

            q_np = np.array([q[0], q[1], q[2], q[3]])

            # Generate random vector values
            for _ in range(10):
                v = np.random.rand(3)

                # Get the rotation of the vector using pyquaternion
                v_rot = q.rotate(v)

                # Get the rotation of the vector using the function
                v_rot_func = self.dynamics.apply_rotation(q_np, v)

                # Check the results
                self.assertAlmostEqual(v_rot[0], v_rot_func[0])
                self.assertAlmostEqual(v_rot[1], v_rot_func[1])
                self.assertAlmostEqual(v_rot[2], v_rot_func[2])

    def test_apply_inverse_rotation(self):
        # Test for each quaternion initial value
        for _ in range(20):
            # Generate random quaternion values (normalized)
            q = Quaternion(*np.random.rand(4))
            q = q.normalised

            q_np = np.array([q[0], q[1], q[2], q[3]])

            # Generate random vector values
            for _ in range(10):
                v = np.random.rand(3)

                # Get the inverse rotation of the vector using pyquaternion
                v_rot = q.inverse.rotate(v)

                # Get the inverse rotation of the vector using the function
                v_rot_func = self.dynamics.apply_inverse_rotation(q_np, v)

                # Check the results
                self.assertAlmostEqual(v_rot[0], v_rot_func[0])
                self.assertAlmostEqual(v_rot[1], v_rot_func[1])
                self.assertAlmostEqual(v_rot[2], v_rot_func[2])

    def test_quaternion_inverse(self):
        # Test for each quaternion initial value
        for _ in range(20):
            # Generate random quaternion values (normalized)
            q = Quaternion(*np.random.rand(4))
            q = q.normalised

            q_np = np.array([q[0], q[1], q[2], q[3]])

            # Get the inverse of the quaternion using pyquaternion
            q_inv = q.inverse

            # Get the inverse of the quaternion using the function
            q_inv_func = self.dynamics.quaternion_inverse(q_np)

            # Check the results
            self.assertAlmostEqual(q_inv[0], q_inv_func[0])
            self.assertAlmostEqual(q_inv[1], q_inv_func[1])
            self.assertAlmostEqual(q_inv[2], q_inv_func[2])
            self.assertAlmostEqual(q_inv[3], q_inv_func[3])

    def test_velocity_derivate(self):
        gravity = np.array([9.81])
        mass = np.array([1.0])

        # Hover
        thrust = np.array([mass * gravity])
        q = np.array([1.0, 0.0, 0.0, 0.0])
        v_dot = self.dynamics.velocity_derivate(q, thrust, gravity, mass)
        self.assertEqual(v_dot[0], 0.0)
        self.assertEqual(v_dot[1], 0.0)
        self.assertEqual(v_dot[2], 0.0)

        # Move up
        thrust = np.array([mass * gravity + 1.0])
        q = np.array([1.0, 0.0, 0.0, 0.0])
        v_dot = self.dynamics.velocity_derivate(q, thrust, gravity, mass)
        self.assertEqual(v_dot[0], 0.0)
        self.assertEqual(v_dot[1], 0.0)
        self.assertGreater(v_dot[2], 0.0)

        # Move down
        thrust = np.array([mass * gravity - 1.0])
        q = np.array([1.0, 0.0, 0.0, 0.0])
        v_dot = self.dynamics.velocity_derivate(q, thrust, gravity, mass)
        self.assertEqual(v_dot[0], 0.0)
        self.assertEqual(v_dot[1], 0.0)
        self.assertLess(v_dot[2], 0.0)

        # Move forward
        thrust = np.array([mass * gravity]) / math.sin(math.pi/4)
        q = Euler_to_quaternion(0.0, math.pi/4, 0.0)
        v_dot = self.dynamics.velocity_derivate(q, thrust, gravity, mass)
        self.assertAlmostEqual(v_dot[0], thrust * math.cos(math.pi/4) / mass)
        self.assertEqual(v_dot[1], 0.0)
        self.assertAlmostEqual(v_dot[2], 0.0)

        # Move backward
        thrust = np.array([mass * gravity]) / math.sin(math.pi/4)
        q = Euler_to_quaternion(0.0, -math.pi/4, 0.0)
        v_dot = self.dynamics.velocity_derivate(q, thrust, gravity, mass)
        self.assertAlmostEqual(v_dot[0], -thrust * math.cos(math.pi/4) / mass)
        self.assertEqual(v_dot[1], 0.0)
        self.assertAlmostEqual(v_dot[2], 0.0)

        # Move right
        thrust = np.array([mass * gravity]) / math.sin(math.pi/4)
        q = Euler_to_quaternion(-math.pi/4, 0.0, 0.0)
        v_dot = self.dynamics.velocity_derivate(q, thrust, gravity, mass)
        self.assertEqual(v_dot[0], 0.0)
        self.assertAlmostEqual(v_dot[1], thrust * math.cos(math.pi/4) / mass)
        self.assertAlmostEqual(v_dot[2], 0.0)

        # Move left
        thrust = np.array([mass * gravity]) / math.sin(math.pi/4)
        q = Euler_to_quaternion(math.pi/4, 0.0, 0.0)
        v_dot = self.dynamics.velocity_derivate(q, thrust, gravity, mass)
        self.assertEqual(v_dot[0], 0.0)
        self.assertAlmostEqual(v_dot[1], -thrust * math.cos(math.pi/4) / mass)
        self.assertAlmostEqual(v_dot[2], 0.0)


if __name__ == '__main__':
    unittest.main()
