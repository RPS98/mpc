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


@dataclass
class MPCParams:
    """
    MPC parameters.

    :param prediction_horizon (float): Prediction horizon in seconds.
    :param prediction_steps (int): Number of prediction steps.
    :param state_weight (AcadosStateWeight): State weight Q.
    :param control_weight (AcadosControlWeight): Control weight R.
    :param mass (float): Mass (kg).
    :param max_thrust (float): Maximum thrust (N).
    :param min_thrust (float): Minimum thrust (N).
    :param max_w_xy (float): Maximum angular velocity in xy (rad/s).
    :param min_w_xy (float): Minimum angular velocity in xy (rad/s).
    :param max_w_z (float): Maximum angular velocity in z (rad/s).
    :param min_w_z (float): Minimum angular velocity in z (rad/s).
    :param gravity (float): Gravity (m/s^2).
    """
    prediction_horizon: float
    prediction_steps: int
    Q: np.ndarray
    R: np.ndarray

    mass: float
    max_thrust: float
    min_thrust: float
    max_w_xy: float
    min_w_xy: float
    max_w_z: float
    min_w_z: float
    gravity: float = 9.81

    def __str__(self):
        return (f'prediction_horizon: {self.prediction_horizon}\n'
                f'prediction_steps: {self.prediction_steps}\n'
                f'Q: \n{self.Q}\n'
                f'R: \n{self.R}\n'
                f'mass: {self.mass}\n'
                f'max_thrust: {self.max_thrust}\n'
                f'min_thrust: {self.min_thrust}\n'
                f'max_w_xy: {self.max_w_xy}\n'
                f'min_w_xy: {self.min_w_xy}\n'
                f'max_w_z: {self.max_w_z}\n'
                f'min_w_z: {self.min_w_z}\n'
                f'gravity: {self.gravity}')


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
                 params: MPCParams) -> None:
        """
        Initialize the MPC.

        :param params(MPCParams): MPC Parameters.
        """
        acados_mpc_params, acados_model_params = self._process_params(params)

        # Initialize the AcadosMPCSolver
        super().__init__(acados_mpc_params, acados_model_params)

        # Set the parameters
        self.update_params(params)

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

    @staticmethod
    def _process_params(params: MPCParams) -> tuple:
        """
        Convert MPC params to AcadosMPCParams and AcadosModelParams objects.

        :param params(MPCParams): MPC Parameters.
        
        :return: AcadosMPCParams, AcadosModelParams.
        """
        # AcadosMPCParams
        acados_mpc_params = mpc_lib.AcadosMPCParams(
            prediction_horizon=params.prediction_horizon,
            prediction_steps=params.prediction_steps,
            Q=params.Q,
            R=params.R,
        )

        # AcadosModelParams
        acados_model_params = mpc_lib.AcadosModelParams(
            mass=params.mass,
            max_thrust=params.max_thrust,
            min_thrust=params.min_thrust,
            max_w_xy=params.max_w_xy,
            min_w_xy=params.min_w_xy,
            max_w_z=params.max_w_z,
            min_w_z=params.min_w_z,
            gravity=params.gravity
        )

        return acados_mpc_params, acados_model_params

    # Getters and Setters
    def update_params(self, params: MPCParams) -> None:
        """
        Update the MPC Parameters.

        :param params(MPCParams): MPC Parameters.
        """
        acados_mpc_params, acados_model_params = self._process_params(params)

        self.update_mpc_params(acados_mpc_params)
        self.update_model_params(acados_model_params)


if __name__ == '__main__':
    mpc_params = MPCParams(
        prediction_horizon=0.5,
        prediction_steps=100,
        Q=mpc_lib.CaState.get_cost_matrix(
            position_weight=3000*np.ones(3),
            orientation_weight=1.0*np.ones(4),
            linear_velocity_weight=0.0*np.ones(3)
        ),
        R=mpc_lib.CaControl.get_cost_matrix(
            thrust_weight=np.array([1.0]),
            angular_velocity_weight=1.0*np.ones(3)
        ),
        mass=1.0,
        max_thrust=30.0,
        min_thrust=1.0,
        max_w_xy=4*np.pi,
        min_w_xy=-4*np.pi,
        max_w_z=4*np.pi,
        min_w_z=-4*np.pi
    )

    mpc = MPC(mpc_params)
    Tf = mpc_params.prediction_horizon
    N = mpc_params.prediction_steps
    dt = Tf / N
    integrator = mpc.export_integrador(dt)

    mpc.update_mpc_params(mpc_params)
    # [0:196]
