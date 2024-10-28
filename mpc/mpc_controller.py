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

"""MPC Controller."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from dataclasses import dataclass, field

import numpy as np
import mpc.mpc_controller_lib.acados_solver as mpc_lib


class Trajectory:
    """Trajectory."""

    def __init__(self):
        self.trajectory = np.array([])
    
    def add_waypoint(
            self,
            state) -> None:
        """
        Add a waypoint to the trajectory.

        :param state(np.array): State [x, y, z, qw, qx, qy, qz, vx, vy, vz] (m, m/s).

        :return: None.
        """
        self.trajectory = np.vstack([self.trajectory, state]) if self.trajectory.size else state

    @property
    def reference(self) -> np.ndarray:
        """
        Get the reference trajectory.

        :return: Reference trajectory.
        """
        return self.trajectory


class MPC(mpc_lib.AcadosMPCSolver):
    """MPC."""

    def __init__(self,
                 prediction_steps: int,
                 prediction_horizon: float,
                 params: mpc_lib.AcadosMPCParams,
                 export_dir: str = 'mpc_generated_code') -> None:
        """
        Initialize the MPC.

        :param prediction_steps(int): Prediction steps.
        :param prediction_horizon(float): Prediction horizon (seconds).
        :param mpc_params(MPCParams): MPC parameters.
        :param export_dir(str): Export directory for the generated code.
        """
        # Initialize the AcadosMPCSolver
        super().__init__(prediction_steps, prediction_horizon, params, export_dir)

        # Internal variables
        self.thrust = 0.0  # Thrust (N)
        self.vehicle_angular_velocity = np.zeros(3)  # Angular velocity (rad/s)

    def trajectory_to_acro(
            self,
            state : np.ndarray,
            trajectory: Trajectory,
            ) -> np.ndarray:
        """
        Convert a desired trajectory into a desired attitude and thrust.

        It uses the concept of the differential flatness.

        :param state(np.array): Current state [x, y, z, qw, qx, qy, qz, vx, vy, vz] (m, m/s).
        :param trajectory(Trajectory): Desired trajectory.

        :return: Desired acro [thrust, wx, wy, wz] (N, rad/s).
        """
        # Compute the control
        u = self.compute_control_action(state, trajectory[:-1, :], trajectory[-1, :])
        return u

    # Getters and Setters
    def update_params(self, mpc_params: mpc_lib.AcadosMPCParams) -> None:
        """
        Update the MPC Parameters.

        :param mpc_params(mpc_lib.AcadosMPCParams): MPC Parameters.
        """
        self.update_mpc_params(mpc_params)


if __name__ == '__main__':
    mpc_params = mpc_lib.AcadosMPCParams(
        Q=mpc_lib.CaState.get_cost_matrix(
            position_weight=3000*np.ones(3),
            orientation_weight=1.0*np.ones(4),
            linear_velocity_weight=0.0*np.ones(3)
        ),
        Qe=mpc_lib.CaState.get_cost_matrix(
            position_weight=3000*np.ones(3),
            orientation_weight=1.0*np.ones(4),
            linear_velocity_weight=0.0*np.ones(3)
        ),
        R=mpc_lib.CaControl.get_cost_matrix(
            thrust_weight=np.array([1.0]),
            angular_velocity_weight=1.0*np.ones(3)
        ),
        lbu=np.array([0.5, -4*np.pi, -4*np.pi, -4*np.pi]),
        ubu=np.array([30.0, 4*np.pi, 4*np.pi, 4*np.pi]),
        p=np.array([1.0])
    )

    mpc = MPC(
        prediction_steps=100,
        prediction_horizon=0.5,
        params=mpc_params
    )
    Tf = mpc.prediction_horizon
    N = mpc.prediction_steps
    dt = Tf / N
    integrator = mpc.export_integrador(dt)
