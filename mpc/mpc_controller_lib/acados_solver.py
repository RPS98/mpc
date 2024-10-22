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

"""Acados Solver definition."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from dataclasses import dataclass

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSim, AcadosSimSolver
import casadi as ca
import scipy.linalg
import numpy as np
from mpc.mpc_controller_lib.drone_model import CaControl, CaState, get_acados_model


CODE_EXPORT_DIR = 'mpc_generated_code'


@dataclass
class AcadosMPCParams:
    """
    MPC parameters.

    :param prediction_horizon (float): Prediction horizon in seconds.
    :param prediction_steps (int): Number of prediction steps.
    :param state_weight (AcadosStateWeight): State weight Q.
    :param control_weight (AcadosControlWeight): Control weight R.
    """
    prediction_horizon: float
    prediction_steps: int
    Q: np.ndarray
    R: np.ndarray


@dataclass
class AcadosModelParams:
    """
    Model parameters.

    :param mass (float): Mass (kg).
    :param max_thrust (float): Maximum thrust (N).
    :param min_thrust (float): Minimum thrust (N).
    :param max_w_xy (float): Maximum angular velocity in xy (rad/s).
    :param min_w_xy (float): Minimum angular velocity in xy (rad/s).
    :param max_w_z (float): Maximum angular velocity in z (rad/s).
    :param min_w_z (float): Minimum angular velocity in z (rad/s).
    :param gravity (float): Gravity (m/s^2).
    """
    mass: float
    max_thrust: float
    min_thrust: float
    max_w_xy: float
    min_w_xy: float
    max_w_z: float
    min_w_z: float
    gravity: float = 9.81


class AcadosMPCSolver:
    """
    Acados Solver for Model Predictive Controller.
    """

    def __init__(
            self,
            mpc_params: AcadosMPCParams,
            model_params: AcadosModelParams) -> None:
        """
        Initialize the Acados MPC controller.

        :param mpc_params(MPCParams): MPC parameters.
        :param model_params(ModelParams): Model parameters.
        """
        self.mpc_params = mpc_params
        self.model_params = model_params

        self.acados_integrator = None

        # Acados model
        self.acados_model = get_acados_model()

        # Define acados solver
        ocp = AcadosOcp()
        ocp.model = self.acados_model

        # initial values for parameter vector - can be updated stagewise
        ocp.parameter_values = np.array([self.model_params.mass])

        # Initial state and control
        x0 = CaState.get_state()
        u0 = CaControl.get_control(
            thrust=model_params.mass * model_params.gravity)

        # Cost
        cost = ocp.cost

        # weight matrix at intermediate shooting nodes (1 to N-1)
        cost.W = scipy.linalg.block_diag(self.mpc_params.Q, self.mpc_params.R)
        # weight matrix at terminal shooting node (N)
        cost.W_e = self.mpc_params.Q

        # reference at intermediate shooting nodes (1 to N-1)
        cost.yref = np.concatenate([x0, u0])
        # reference at terminal shooting node (N)
        cost.yref_e = x0

        # # For linear least squares cost
        # # Set up the cost type
        # cost.cost_type = 'LINEAR_LS'
        # cost.cost_type_e = 'LINEAR_LS'
        # # dimensions
        # nx = ocp.model.x.size()[0]
        # nu = ocp.model.u.size()[0]
        # ny = nx + nu
        # # x matrix coefficient at intermediate shooting nodes (1 to N-1)
        # cost.Vx = np.eye(ny, nx)
        # # u matrix coefficient at intermediate shooting nodes (1 to N-1)
        # cost.Vu = np.vstack((np.zeros((nx, nu)), np.eye(nu)))
        # # x matrix coefficient for cost at terminal shooting node (N)
        # cost.Vx_e = np.eye(nx, nx)

        # For nonlinear least squares cost
        # Set up the cost type
        cost.cost_type = 'NONLINEAR_LS'
        cost.cost_type_e = 'NONLINEAR_LS'
        # CasADi expression for nonlinear least squares
        ocp.model.cost_y_expr = ca.vertcat(ocp.model.x, ocp.model.u)
        ocp.model.cost_y_expr_e = ocp.model.x

        # Constraints
        constraints = ocp.constraints
        # initial state
        constraints.x0 = x0
        # lower bounds on u at shooting nodes (0 to N-1)
        constraints.lbu = np.array([
            self.model_params.min_thrust,
            self.model_params.min_w_xy,
            self.model_params.min_w_xy,
            self.model_params.min_w_z])
        # upper bounds on u at shooting nodes (0 to N-1)
        constraints.ubu = np.array([
            self.model_params.max_thrust,
            self.model_params.max_w_xy,
            self.model_params.max_w_xy,
            self.model_params.max_w_z])
        # matrix coefficient for bounds on u at shooting nodes
        constraints.Jbu = np.identity(4)

        # Solver options
        solver_options = ocp.solver_options
        # number of shooting intervals
        solver_options.N_horizon = self.mpc_params.prediction_steps
        # prediction horizon
        solver_options.tf = self.mpc_params.prediction_horizon
        # QP solver to be used in the NLP solver. String in (
        # ‘PARTIAL_CONDENSING_HPIPM’, ‘FULL_CONDENSING_QPOASES’, ‘FULL_CONDENSING_HPIPM’,
        # ‘PARTIAL_CONDENSING_QPDUNES’, ‘PARTIAL_CONDENSING_OSQP’, ‘FULL_CONDENSING_DAQP’).
        # Default: ‘PARTIAL_CONDENSING_HPIPM’.
        solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        # NLP solver. String in (‘SQP’, ‘SQP_RTI’, ‘DDP’). Default: ‘SQP_RTI’.
        solver_options.nlp_solver_type = 'SQP_RTI'
        # Hessian approximation. String in (‘GAUSS_NEWTON’, ‘EXACT’). Default: ‘GAUSS_NEWTON’.
        solver_options.hessian_approx = 'GAUSS_NEWTON'
        # Integrator type. String in (‘ERK’, ‘IRK’, ‘GNSF’, ‘DISCRETE’, ‘LIFTED_IRK’). Default: ‘ERK’.
        solver_options.integrator_type = 'ERK'

        # Create solver
        ocp.code_export_directory = CODE_EXPORT_DIR + '/mpc_generated_code'
        ocp.json_file = CODE_EXPORT_DIR + '/acados_ocp.json'

        self._solver = AcadosOcpSolver(
            ocp,
            # json_file = 'mpc_controller.json',
            generate=True,
            verbose=False)

        # Internal variables
        self.x_dim = x0.shape[0]
        self.u_dim = u0.shape[0]
        self.u0 = u0

    def update_mpc_params(self, mpc_params: AcadosMPCParams) -> None:
        """
        Update the MPC parameters: Q and R.

        :param mpc_params(MPCParams): MPC parameters.
        """
        # Update Solver cost:
        self.mpc_params = mpc_params

        # weight matrix at intermediate shooting nodes (1 to N-1)
        for node in range(1, self.N):
            self.solver.cost_set(node, 'W', scipy.linalg.block_diag(self.mpc_params.Q, self.mpc_params.R))

        # weight matrix at terminal shooting node (N)
        self.solver.cost_set(self.N, 'W', self.mpc_params.Q)

    def update_model_params(self, model_params: AcadosModelParams) -> None:
        """
        Update the model parameters.

        :param model_params(ModelParams): Model parameters.
        """
        # Update Solver constraints:
        self.model_params = model_params

        # lower bounds on u at shooting nodes (0 to N-1)
        lbu = np.array([
            self.model_params.min_thrust,
            self.model_params.min_w_xy,
            self.model_params.min_w_xy,
            self.model_params.min_w_z])
        # upper bounds on u at shooting nodes (0 to N-1)
        ubu = np.array([
            self.model_params.max_thrust,
            self.model_params.max_w_xy,
            self.model_params.max_w_xy,
            self.model_params.max_w_z])
        
        for node in range(self.N):
            self.solver.constraints_set(node, 'lbu', lbu)
            self.solver.constraints_set(node, 'ubu', ubu)

        
        # initial values for parameter vector - can be updated stagewise
        for i in range(self.N+1):
            self.solver.set(i, 'p', np.array([model_params.mass]))
        

    def export_integrador(self, simulation_time):
        # Acados Sim
        acados_sim = AcadosSim()
        acados_sim.model = self.acados_model
        acados_sim.parameter_values = np.array([self.model_params.mass])

        # Solver options
        # integrator type. String in (‘ERK’, ‘IRK’, ‘GNSF’, ‘DISCRETE’, ‘LIFTED_IRK’).
        acados_sim.solver_options.integrator_type = 'IRK'
        # number of stages in the integrator
        acados_sim.solver_options.num_stages = 4
        # number of steps in the integrator
        acados_sim.solver_options.num_steps = 3
        # time horizon
        acados_sim.solver_options.T = simulation_time

        # Create integrator
        acados_sim.code_export_directory = CODE_EXPORT_DIR + '/mpc_generated_code'
        json_file = CODE_EXPORT_DIR + '/acados_sim.json'
        self.acados_integrator = AcadosSimSolver(
            acados_sim,
            json_file=json_file,
            generate=True,
            verbose=True)

        return self.acados_integrator

    @property
    def states(self):
        """
        Get the predicted state of the system.

        :return: The predicted state of the system.
        """
        states = np.zeros((self.solver.acados_ocp.solver_options.N_horizon+1, self.solver.get(0, "x").shape[0]))
        for i in range(self.solver.acados_ocp.solver_options.N_horizon+1):
            states[i, :] = self.solver.get(i, "x")
        return states
    
    @property
    def controls(self):
        """
        Get the predicted control input of the system.

        :return: The predicted control input of the system.
        """
        controls = np.zeros((self.solver.acados_ocp.solver_options.N_horizon, self.solver.get(0, "u").shape[0]))
        for i in range(self.solver.acados_ocp.solver_options.N_horizon):
            controls[i, :] = self.solver.get(i, "u")
        return controls

    def _set_state(self, state: np.ndarray) -> None:
        """
        Set the current state of the system.

        :param state: The current state of the system.
        """
        self.solver.set(0, "lbx", state)
        self.solver.set(0, "ubx", state)

    def _set_references(self, yref: np.ndarray, yref_e: np.ndarray) -> None:
        """
        Set the reference trajectory for the system.

        :param yref: The intermediate reference trajectory for the system
        (matrix of size [N, state_dim + u_dim]).
        :param yref_e: The final reference trajectory for the system
        (matrix of size [state_dim]).
        """
        # Check the size of the reference and adjust as needed
        if yref.shape[1] != (self.x_dim + self.u_dim):
            # Add u0 to the reference trajectory: [yref, u0]
            yref = np.concatenate([yref, np.tile(self.u0, (yref.shape[0], 1))], axis=1)

        for i in range(self.N):
            self.solver.set(i, 'yref', yref[i, :])
        self.solver.set(self.N, 'yref', yref_e)
        
    def evaluate(
            self,
            state: np.ndarray,
            reference_trajectory_intermediate: np.ndarray,
            reference_trajectory_final: np.ndarray) -> np.ndarray:
        """
        Simulate the system using MPC with a given state and reference trajectory.

        :param state: The current state of the system.
        :param reference_trajectory_intermediate: The intermediate reference trajectory
        for the system (matrix of size [N, state_dim]).
        :param reference_trajectory_final: The final reference trajectory for the system
        (matrix of size [state_dim]).
        
        :return: The control action u0.
        """
        # Set the reference trajectory
        self._set_references(reference_trajectory_intermediate, reference_trajectory_final)

        # Set current state
        self._set_state(state)
        
        # Solve the MPC problem
        status = self.solver.solve()
        if status != 0:
            print(f"MPC Solver failed with status {status}")
            return None

        return self.controls[0]
    
    def compute_control_action(
            self,
            state: np.ndarray,
            reference_trajectory_intermediate: np.ndarray,
            reference_trajectory_final: np.ndarray) -> np.ndarray:
        """
        Simulate the system using MPC with a given state and reference trajectory.

        :param state: The current state of the system.
        :param reference_trajectory_intermediate: The intermediate reference trajectory
        for the system (matrix of size [N, state_dim]).
        :param reference_trajectory_final: The final reference trajectory for the system
        (matrix of size [state_dim]).
        
        :return: The control action u0.
        """
        # Set the reference trajectory
        self._set_references(reference_trajectory_intermediate, reference_trajectory_final)
        
        # Solve the MPC problem
        return self.solver.solve_for_x0(state)

    @property
    def solver(self):
        return self._solver
    
    @property
    def prediction_horizon(self):
        return self.solver.acados_ocp.solver_options.tf
    
    @property
    def prediction_steps(self):
        return self.solver.acados_ocp.solver_options.N_horizon
    
    @property
    def evaluation_time(self):
        return self.solver.acados_ocp.solver_options.shooting_nodes

    @property
    def N(self):
        """
        Index of maximum shooting node.
        """
        return self.solver.acados_ocp.solver_options.N_horizon

    def get_empty_reference(self):
        """
        Get an empty reference trajectory.

        :return: An empty reference trajectory.
        """
        return np.zeros((self.N, self.x_dim + self.u_dim))
    
    def get_empty_end_reference(self):
        """
        Get an empty final reference trajectory.

        :return: An empty final reference trajectory.
        """
        return np.zeros(self.x_dim)
    
    def get_empty_state(self):
        """
        Get an empty state.

        :return: An empty state.
        """
        return np.zeros(self.x_dim)


if __name__ == '__main__':
    mpc_params = AcadosMPCParams(
        prediction_horizon=0.5,
        prediction_steps=20,
        Q=CaState.get_cost_matrix(
            position_weight=1*np.ones(3),
            orientation_weight=0*np.ones(4),
            linear_velocity_weight=0.0*np.ones(3)
        ),
        R=CaControl.get_cost_matrix(
            thrust_weight=1e-2,
            angular_velocity_weight=1e-2*np.ones(3)
        )
    )

    model_params = AcadosModelParams(
        mass=1.0,
        gravity=9.81,
        max_thrust=30.0,
        min_thrust=0.2,
        max_w_xy=1.0,
        min_w_xy=-1.0,
        max_w_z=1.0,
        min_w_z=-1.0)
    
    mpc = AcadosMPCSolver(
        mpc_params=mpc_params,
        model_params=model_params)

    # Update params
    mpc_params = AcadosMPCParams(
        prediction_horizon=0.5,
        prediction_steps=20,
        Q=CaState.get_cost_matrix(
            position_weight=2*np.ones(3),
            orientation_weight=0*np.ones(4),
            linear_velocity_weight=0.0*np.ones(3)
        ),
        R=CaControl.get_cost_matrix(
            thrust_weight=1e-2,
            angular_velocity_weight=1e-2*np.ones(3)
        )
    )

    # Compute the control
    u = mpc.compute_control_action(
        mpc.get_empty_state(),
        mpc.get_empty_reference(),
        mpc.get_empty_end_reference())
