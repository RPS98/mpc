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

import argparse

from dataclasses import dataclass
import json

from mpc.utils.yaml_to_dict import yaml_to_dict
from acados_template import AcadosOcpSolver
import numpy as np
import scipy.linalg


@dataclass
class MPCParameters():
    """
    MPC parameters.

    :param dt: Time step.
    :type dt: float
    :param Q: State weight matrix.
    :type Q: np.ndarray
    :param Qe: Terminal state weight matrix.
    :type Qe: np.ndarray
    :param R: Control weight matrix.
    :type R: np.ndarray
    :param p: Parameter vector.
    :type p: np.ndarray
    :param lbu: Lower bounds on control input.
    :type lbu: np.ndarray
    :param ubu: Upper bounds on control input.
    :type ubu: np.ndarray
    :param lbx: Lower bounds on state.
    :type lbx: np.ndarray
    :param ubx: Upper bounds on state.
    :type ubx: np.ndarray
    """
    dt: float
    Q: np.ndarray
    Qe: np.ndarray
    R: np.ndarray
    p: np.ndarray
    lbu: np.ndarray
    ubu: np.ndarray
    lbx: np.ndarray
    ubx: np.ndarray

    def __str__(self):
        return (
            f'dt: {self.dt}\n'
            f'Q: \n{self.Q}\n'
            f'Qe: \n{self.Qe}\n'
            f'R: \n{self.R}\n'
            f'p: {self.p}\n'
            f'lbu: {self.lbu}\n'
            f'ubu: {self.ubu}\n'
            f'lbx: {self.lbx}\n'
            f'ubx: {self.ubx}\n'
        )


class MPC():
    """MPC."""

    def __init__(self,
                 ocp_json_file: str,
                 ) -> None:
        """
        Initialize the Acados MPC controller.

        :param ocp_json_file: Path to the OCP JSON file
        :type ocp_json_file: str
        :return: None
        :rtype: None
        """
        # Initialize the AcadosOcpSolver
        self.acados_ocp_solver = AcadosOcpSolver(
            None,
            json_file=ocp_json_file,
            build=False,
            generate=False)

        # Get dimensions
        self._N = self.acados_ocp_solver.N

        # Get time parameters from JSON file (acados_ocp is None when loaded from JSON)
        with open(ocp_json_file, 'r') as f:
            ocp_json = json.load(f)
        self._tf = ocp_json['solver_options']['tf']
        self._dt = self._tf / self._N
        # Alternative: use time_steps directly if non-uniform
        # self._time_steps = np.array(ocp_json['solver_options']['time_steps'])

        self._w_size = self.acados_ocp_solver.cost_get(0, 'W').shape[0]
        self._we_size = self.acados_ocp_solver.cost_get(self._N, 'W').shape[0]
        self._x_size = self.acados_ocp_solver.get(0, 'x').shape[0]
        self._u_size = self.acados_ocp_solver.get(0, 'u').shape[0]
        self._p_size = self.acados_ocp_solver.get(0, 'p').shape[0]
        self._lbu_size = self.acados_ocp_solver.constraints_get(0, 'lbu').shape[0]
        self._ubu_size = self.acados_ocp_solver.constraints_get(0, 'ubu').shape[0]
        self._lbx_size = self.acados_ocp_solver.constraints_get(1, 'lbx').shape[0]
        self._ubx_size = self.acados_ocp_solver.constraints_get(1, 'ubx').shape[0]

        print('MPC parameters:')
        print(f'Horizon N={self.N}, tf={self.tf}, dt={self.dt}')
        print('Sizes:')
        print(f'  w_size: {self.w_size}')
        print(f'  we_size: {self.we_size}')
        print(f'  x_size: {self.x_size}')
        print(f'  u_size: {self.u_size}')
        print(f'  p_size: {self.p_size}')
        print(f'  lbu_size: {self.lbu_size}')
        print(f'  ubu_size: {self.ubu_size}')
        print(f'  lbx_size: {self.lbx_size}')
        print(f'  ubx_size: {self.ubx_size}')

        # Internal variables
        self.thrust = 0.0  # Thrust (N)
        self.vehicle_angular_velocity = np.zeros(3)  # Angular velocity (rad/s)
        self._u_ref = np.zeros(self.u_size)  # Reference control input

    # Read-only properties
    @property
    def N(self) -> int:
        """Get the number of prediction steps."""
        return self._N

    @property
    def tf(self) -> float:
        """Get the total prediction horizon time."""
        return self._tf

    @property
    def dt(self) -> float:
        """Get the time step between prediction stages."""
        return self._dt

    @property
    def w_size(self) -> int:
        """Get the size of the cost weighting matrix W."""
        return self._w_size

    @property
    def we_size(self) -> int:
        """Get the size of the terminal cost weighting matrix W_e."""
        return self._we_size

    @property
    def x_size(self) -> int:
        """Get the state vector size."""
        return self._x_size

    @property
    def u_size(self) -> int:
        """Get the control input vector size."""
        return self._u_size

    @property
    def p_size(self) -> int:
        """Get the parameter vector size."""
        return self._p_size

    @property
    def lbu_size(self) -> int:
        """Get the size of lower control bounds."""
        return self._lbu_size

    @property
    def ubu_size(self) -> int:
        """Get the size of upper control bounds."""
        return self._ubu_size

    @property
    def lbx_size(self) -> int:
        """Get the size of lower state bounds."""
        return self._lbx_size

    @property
    def ubx_size(self) -> int:
        """Get the size of upper state bounds."""
        return self._ubx_size

    @property
    def states(self):
        """
        Get the predicted state of the system.

        :return: The predicted state of the system.
        """
        states = np.zeros((
            self.N + 1, self.x_size))
        for i in range(self.N + 1):
            states[i, :] = self.get(i, 'x')
        return states

    @property
    def actuation(self):
        """
        Get the predicted actuation input of the system.

        :return: The predicted actuation input of the system.
        """
        actuation = np.zeros((
            self.N, self.u_size))
        for i in range(self.N):
            actuation[i, :] = self.acados_ocp_solver.get(i, 'u')
        return actuation
    
    @property
    def parameters(self):
        """
        Get the parameters of the system.

        :return: The parameters of the system.
        """
        parameters = np.zeros((
            self.N + 1, self.p_size))
        for i in range(self.N + 1):
            parameters[i, :] = self.acados_ocp_solver.get(i, 'p')
        return parameters

    def set_mpc_parameters(self, parameters: MPCParameters) -> None:
        """
        Set the MPC parameters.

        :param parameters: MPC parameters
        :type parameters: MPCParameters
        :return: None
        :rtype: None
        """
        self.set_gains(parameters.Q, parameters.R)
        self.set_gain_terminal_state(parameters.Qe)
        self.set_parameters(parameters.p)
        self.set_u_bounds(parameters.lbu, parameters.ubu)
        self.set_x_bounds(parameters.lbx, parameters.ubx)

    def set_gains(self, Q: np.ndarray, R: np.ndarray, stage: int = -1) -> None:
        """
        Set the state weighting matrix Q.

        :param Q: State weighting matrix
        :type Q: np.ndarray
        :param R: Actuation weighting matrix
        :type R: np.ndarray
        :param stage: Stage index (default: -1 for all stages)
        :type stage: int
        :return: None
        :rtype: None
        """
        # Check if Q and R are 2D quared matrices or 1D arrays
        if Q.ndim == 1:
            Q = np.diag(Q)
        if R.ndim == 1:
            R = np.diag(R)
        # Build W as a block-diagonal matrix so Q and R can have different sizes
        # Use scipy.linalg.block_diag which correctly handles matrices of
        # different dimensions (e.g., state and input weight sizes).
        w = scipy.linalg.block_diag(Q, R)

        # Check size of Q and R
        if w.shape != (self.w_size, self.w_size):
            raise ValueError(
                f"Size mismatch: Q and R result in W of shape {w.shape}, "
                f"but expected shape is ({self.w_size}, {self.w_size}).")

        if stage == -1:
            for state_i in range(self.N):
                self.acados_ocp_solver.cost_set(state_i, 'W', w)
        else:
            if stage >= self.N:
                raise ValueError(
                    f"Stage index {stage} out of bounds for N={self.N}.")
            self.acados_ocp_solver.cost_set(stage, 'W', w)

    def set_gain_terminal_state(self, Qe: np.ndarray) -> None:
        """
        Set the terminal state weighting matrix Qe.

        :param Qe: Terminal state weighting matrix
        :type Qe: np.ndarray
        :return: None
        :rtype: None
        """
        if Qe.ndim == 1:
            Qe = np.diag(Qe)

        # Check size of Qe
        if Qe.shape != (self.we_size, self.we_size):
            raise ValueError(
                f"Size mismatch: Qe has shape {Qe.shape}, "
                f"but expected shape is ({self.we_size}, {self.we_size}).")

        self.acados_ocp_solver.cost_set(self.N, 'W', Qe)

    def set_parameters(self, p: np.ndarray, stage: int = -1) -> None:
        """
        Set the parameters of the MPC problem.

        :param p: Parameters array
        :type p: np.ndarray
        :param stage: Stage index (default: -1 for all stages)
        :type stage: int
        :return: None
        :rtype: None
        """
        if p.shape == (self.N + 1, self.p_size):
            # If p is given for all stages, set it for each stage
            for stage_i in range(self.N + 1):
                self.acados_ocp_solver.set(stage_i, 'p', p[stage_i, :])
            return
        # Check size of p
        if p.shape[0] != self.p_size:
            raise ValueError(
                f"Size mismatch: p has shape {p.shape}, "
                f"but expected size is ({self.p_size},).")

        if stage == -1:
            for stage_i in range(self.N + 1):
                self.acados_ocp_solver.set(stage_i, 'p', p)
        else:
            self.acados_ocp_solver.set(stage, 'p', p)

    def set_u_bounds(self, u_min: np.ndarray, u_max: np.ndarray, stage: int = -1) -> None:
        """
        Set the control input bounds.

        :param u_min: Minimum control input
        :type u_min: np.ndarray
        :param u_max: Maximum control input
        :type u_max: np.ndarray
        :param stage: Stage index (default: -1 for all stages)
        :type stage: int
        :return: None
        :rtype: None
        """
        # Check size of u_min and u_max
        if u_min.shape[0] != self.lbu_size or u_max.shape[0] != self.ubu_size:
            raise ValueError(
                f"Size mismatch: u_min has shape {u_min.shape} and u_max has shape {u_max.shape}, "
                f"but expected sizes are ({self.lbu_size},) and ({self.ubu_size},).")

        # Set bounds
        if stage == -1:
            for stage_i in range(self.N):
                self.acados_ocp_solver.constraints_set(stage_i, 'lbu', u_min)
                self.acados_ocp_solver.constraints_set(stage_i, 'ubu', u_max)
        else:
            self.acados_ocp_solver.constraints_set(stage, 'lbu', u_min)
            self.acados_ocp_solver.constraints_set(stage, 'ubu', u_max)

    def set_x_bounds(self, x_min: np.ndarray, x_max: np.ndarray, stage: int = -1) -> None:
        """
        Set the state bounds.

        :param x_min: Minimum state
        :type x_min: np.ndarray
        :param x_max: Maximum state
        :type x_max: np.ndarray
        :param stage: Stage index (default: -1 for all stages)
        :type stage: int
        :return: None
        :rtype: None
        """
        # Check size of x_min and x_max
        if x_min.shape[0] != self.lbx_size or x_max.shape[0] != self.ubx_size:
            raise ValueError(
                f"Size mismatch: x_min has shape {x_min.shape} and x_max has shape {x_max.shape}, "
                f"but expected sizes are ({self.lbx_size},) and ({self.ubx_size},).")

        # Set bounds
        if stage == -1:
            for stage_i in range(self.N + 1):
                if stage_i == 0:
                    continue  # Skip initial state
                self.acados_ocp_solver.constraints_set(stage_i, 'lbx', x_min)
                self.acados_ocp_solver.constraints_set(stage_i, 'ubx', x_max)
        else:
            if stage == 0:
                return  # Skip initial state
            self.acados_ocp_solver.constraints_set(stage, 'lbx', x_min)
            self.acados_ocp_solver.constraints_set(stage, 'ubx', x_max)

    def set_state(self, x: np.ndarray) -> None:
        """
        Set the state of the MPC problem.

        :param x: State array
        :type x: np.ndarray
        :return: None
        :rtype: None
        """
        # Check size of x
        if x.shape[0] != self.x_size:
            raise ValueError(
                f"Size mismatch: x has shape {x.shape}, "
                f"but expected size is ({self.x_size},).")
        self.acados_ocp_solver.set(0, 'lbx', x)
        self.acados_ocp_solver.set(0, 'ubx', x)

    def set_u_ref(self, u_ref: np.ndarray) -> None:
        """
        Set the reference control input for yref.

        :param u_ref: Reference control input array
        :type u_ref: np.ndarray
        :return: None
        :rtype: None
        """
        # Check size of u_ref
        if u_ref.shape[0] != self.u_size:
            raise ValueError(
                f"Size mismatch: u_ref has shape {u_ref.shape}, "
                f"but expected size is ({self.u_size},).")
        self._u_ref = u_ref
        
    def get_u_ref(self) -> np.ndarray:
        """
        Get the reference control input for yref.

        :return: Reference control input array
        :rtype: np.ndarray
        """
        return self._u_ref

    def _expand_y_ref_with_u_ref(self, y_ref: np.ndarray) -> np.ndarray:
        """
        Expand y_ref by appending the reference control input u_ref.

        :param y_ref: Reference state array
        :type y_ref: np.ndarray
        :return: Expanded reference state array
        :rtype: np.ndarray
        """
        expanded_y_ref = np.zeros(self.w_size)
        expanded_y_ref[:self.we_size] = y_ref
        expanded_y_ref[self.we_size:] = self._u_ref
        return expanded_y_ref

    def set_y_ref_per_stage(self, y_ref: np.ndarray, stage: int) -> None:
        """
        Set the reference state of the MPC problem.

        :param y_ref: Reference state array
        :type y_ref: np.ndarray
        :param stage: Stage index
        :type stage: int
        :return: None
        :rtype: None
        """
        # Check size of y_ref
        if y_ref.shape[0] == self.we_size:
            y_ref = self._expand_y_ref_with_u_ref(y_ref)
        if y_ref.shape[0] != self.w_size:
            raise ValueError(
                f"Size mismatch: y_ref has shape {y_ref.shape}, "
                f"but expected size is ({self.w_size},).")
        if stage < 0 or stage >= self.N:
            raise ValueError(
                f"Stage index {stage} out of bounds for N={self.N}.")
        self.acados_ocp_solver.cost_set(stage, 'yref', y_ref)

    def set_y_ref(self, y_ref: np.ndarray) -> None:
        """
        Set the reference state of the MPC problem.

        :param y_ref: Reference state array
        :type y_ref: np.ndarray
        :return: None
        :rtype: None
        """
        if y_ref.shape[0] != self.N:
            raise ValueError(
                f"Size mismatch: y_ref has shape {y_ref.shape}, "
                f"but expected size is ({self.N},).")

        for stage_i in range(self.N):
            single_y_ref = y_ref[stage_i]
            if single_y_ref.shape[0] == self.we_size:
                single_y_ref = self._expand_y_ref_with_u_ref(single_y_ref)
            if single_y_ref.shape[0] != self.w_size:
                raise ValueError(
                    f"Size mismatch: y_ref at stage {stage_i} has shape {single_y_ref.shape}, "
                    f"but expected size is ({self.w_size},).")
            self.acados_ocp_solver.cost_set(stage_i, 'yref', single_y_ref)

    def set_y_ref_e(self, y_ref_e: np.ndarray) -> None:
        """
        Set the terminal state output of the MPC problem.

        :param y_ref_e: Terminal reference state array
        :type y_ref_e: np.ndarray
        :return: None
        :rtype: None
        """
        # Check size of y_ref_e
        if y_ref_e.shape[0] != self.we_size:
            raise ValueError(
                f"Size mismatch: y_ref_e has shape {y_ref_e.shape}, "
                f"but expected size is ({self.we_size},).")

        self.acados_ocp_solver.cost_set(self.N, 'yref', y_ref_e)

    def set_new_time_steps(self, dt: float) -> None:
        """
        Set new uniform time steps for the MPC problem.

        :param dt: New time step
        :type dt: float
        :return: None
        :rtype: None
        """
        time_steps = np.zeros(self.N)
        last_dt = 0.0
        for i in range(self.N):
            time_steps[i] = dt + last_dt
            last_dt = time_steps[i]
        self.acados_ocp_solver.set_new_time_steps(time_steps)
        self._dt = dt
        self._tf = dt * self.N

    def compute_control_action(
            self,
            state: np.ndarray,
            reference_trajectory_intermediate: np.ndarray,
            reference_trajectory_final: np.ndarray) -> np.ndarray:
        """
        Simulate the system using MPC with a given state and reference trajectory.

        Acados status:
            ACADOS_SUCCESS = 0
            ACADOS_NAN_DETECTED = 1
            ACADOS_MAXITER = 2
            ACADOS_MINSTEP = 3
            ACADOS_QP_FAILURE = 4
            ACADOS_READY = 5
            ACADOS_UNBOUNDED = 6

        :param state: The current state of the system.
        :param reference_trajectory_intermediate: The intermediate reference trajectory
        for the system (matrix of size [N, state_dim]).
        :param reference_trajectory_final: The final reference trajectory for the system
        (matrix of size [state_dim]).

        :return: The control action u0.
        """
        # Set the reference trajectory
        self.set_y_ref(reference_trajectory_intermediate)
        self.set_y_ref_e(reference_trajectory_final)

        # Set current state
        self.set_state(state)

        # Solve the MPC problem
        status = self.acados_ocp_solver.solve()
        if status != 0:
            # Provide additional diagnostics to help debugging QP failures
            print(f'MPC Solver failed with status {status}')
            try:
                stats = self.acados_ocp_solver.get_stats()
                print('Acados solver stats:')
                print(stats)
            except Exception as _e:
                print('Could not retrieve solver stats:', _e)
            try:
                diag = self.acados_ocp_solver.qp_diagnostics()
                print('QP diagnostics:')
                print(diag)
            except Exception as _e:
                print('Could not retrieve QP diagnostics:', _e)
            try:
                # Dump last QP to a JSON file for offline inspection
                self.acados_ocp_solver.dump_last_qp_to_json('last_qp.json')
                print('Dumped last QP to last_qp.json')
            except Exception as _e:
                print('Could not dump last QP to JSON:', _e)

            raise Exception(
                f'MPC solver returned status {status}. Exiting.')

        return self.actuation[0]

    def solve(
            self,
            state: np.ndarray = None,
            y_ref: np.ndarray = None,
            y_ref_e: np.ndarray = None,
            p: np.ndarray = None) -> np.ndarray:
        """
        Simulate the system using MPC with a given state and reference trajectory.

        Acados status:
            ACADOS_SUCCESS = 0
            ACADOS_NAN_DETECTED = 1
            ACADOS_MAXITER = 2
            ACADOS_MINSTEP = 3
            ACADOS_QP_FAILURE = 4
            ACADOS_READY = 5
            ACADOS_UNBOUNDED = 6

        :param state: The current state of the system.
        :param reference_trajectory_intermediate: The intermediate reference trajectory
        for the system (matrix of size [N, state_dim]).
        :param reference_trajectory_final: The final reference trajectory for the system
        (matrix of size [state_dim]).

        :return: The control action u0.
        """
        # Set the reference trajectory
        if y_ref is not None:
            self.set_y_ref(y_ref)
        if y_ref_e is not None:
            self.set_y_ref_e(y_ref_e)

        # Set current state
        if state is not None:
            self.set_state(state)
            
        # Set parameters
        if p is not None:
            self.set_parameters(p)

        # Solve the MPC problem
        status = self.acados_ocp_solver.solve()
        if status != 0:
            # Provide additional diagnostics to help debugging QP failures
            print(f'MPC Solver failed with status {status}')
            try:
                stats = self.acados_ocp_solver.get_stats()
                print('Acados solver stats:')
                print(stats)
            except Exception as _e:
                print('Could not retrieve solver stats:', _e)
            try:
                diag = self.acados_ocp_solver.qp_diagnostics()
                print('QP diagnostics:')
                print(diag)
            except Exception as _e:
                print('Could not retrieve QP diagnostics:', _e)
            try:
                # Dump last QP to a JSON file for offline inspection
                self.acados_ocp_solver.dump_last_qp_to_json('last_qp.json')
                print('Dumped last QP to last_qp.json')
            except Exception as _e:
                print('Could not dump last QP to JSON:', _e)

            raise Exception(
                f'MPC solver returned status {status}. Exiting.')

        return self.actuation[0]


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='MPC code exporter.')
    parser.add_argument(
        '--config_file',
        '-c',
        type=str,
        help='Configuration json file: ocp_json_file.json')

    args = parser.parse_args()
    mpc = MPC(
        ocp_json_file=args.config_file,
    )
