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
"""Acados Solver Parameters definition."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from typing import Callable, Type

import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSim, AcadosSimSolver

from mpc_acados_core.utils.solver_config import SolverDefinition


class AcadosMPCSolverBase:
    """
    Acados Solver base for Model Predictive Controller.

    Subclasses must bind the controller-specific types by overriding:
        - ``State``: Generated state vector class.
        - ``Actuation``: Generated actuation vector class.
        - ``Parameters``: Generated parameters vector class.
        - ``get_acados_model``: Callable returning ``(acados_model, drone_model)``.
    """

    State: Type = None
    Actuation: Type = None
    Parameters: Type = None
    get_acados_model: Callable = None

    @classmethod
    def _assert_bound(cls) -> None:
        """Ensure subclass injected required variant-specific classes."""
        missing = [
            attr for attr in ('State', 'Actuation', 'Parameters', 'get_acados_model')
            if getattr(cls, attr) is None
        ]
        if missing:
            raise TypeError(
                f"{cls.__name__} is unbound: subclass must define class attributes "
                f"{missing}."
            )

    def __init__(
            self,
            solver_definition_path: str,
            generate_acados_solver: bool = True,
            generate_acados_simulator: bool = True,
            generate_code: bool = True) -> None:
        """
        Initialize the Acados MPC controller.

        :param solver_definition_path: Path to the solver definition YAML file
        :type solver_definition_path: str
        :param generate_acados_solver: Whether to generate the Acados solver
        :type generate_acados_solver: bool
        :param generate_acados_simulator: Whether to generate the Acados simulator
        :type generate_acados_simulator: bool
        :param generate_code: Whether to generate C code
        :type generate_code: bool
        :return: None
        :rtype: None
        """
        self._assert_bound()

        # Load the solver definition from the YAML file
        solver_definition: SolverDefinition = SolverDefinition.from_yaml(solver_definition_path)

        # Acados model
        self.acados_model, self.drone_model = type(self).get_acados_model()

        # Acados solver and integrator
        if generate_acados_solver:
            self.solver = self.get_acados_solver(solver_definition, generate_code)
        if generate_acados_simulator:
            self.acados_integrator = self.get_acados_sim_solver(solver_definition, generate_code)

        # Generate C++ project if requested and solvers were generated
        if generate_acados_solver and generate_acados_simulator and generate_code:
            if solver_definition.cpp_module.generate:
                self.generate_cpp_project(solver_definition)

    def get_acados_solver(self, solver_definition: SolverDefinition, generate_code: bool = True) -> AcadosOcpSolver:
        """
        Get the Acados MPC Solver.

        :param solver_definition: Solver definition
        :type solver_definition: SolverDefinition
        :param generate_code: Whether to generate C code
        :type generate_code: bool
        :return: Acados MPC Solver
        :rtype: AcadosOcpSolver
        """
        # Acados solver
        ocp = AcadosOcp()
        ocp.model = self.acados_model
        state = self.State()
        actuation = self.Actuation()  # Hovering thrust reference

        # Parameters
        ocp.parameter_values = self.Parameters().vector

        # Cost
        cost = ocp.cost
        
        # Set depending on the cost type specified in the solver definition
        cost_type = solver_definition.solver.cost_type
        # Set up the cost type
        cost.cost_type = cost_type
        cost.cost_type_e = cost_type
        if cost_type == 'NONLINEAR_LS':
            # Nonlinear least squares cost
            # CasADi expression for nonlinear least squares
            # ocp.model.cost_y_expr = ca.vertcat(ocp.model.x, ocp.model.u)
            # ocp.model.cost_y_expr_e = ocp.model.x

            # Weight matrix at intermediate shooting nodes (1 to N-1)
            cost.W = np.diag(np.zeros(self.acados_model.cost_y_expr.shape[0]))
            # Weight matrix at terminal shooting node (N)
            cost.W_e = np.diag(np.zeros(self.acados_model.cost_y_expr_e.shape[0]))
            
            # Reference at intermediate shooting nodes (1 to N-1)
            cost.yref = np.concatenate([
                np.zeros(3),  # Position reference
                np.zeros(3),  # Attitude reference
                state.linear_velocity,  # Linear velocity reference
                actuation.vector # Control reference
            ])
            # Reference at terminal shooting node (N)
            cost.yref_e = np.concatenate([
                np.zeros(3),  # Position reference
                np.zeros(3),  # Attitude reference
                state.linear_velocity,  # Linear velocity reference
            ])
        
        elif cost_type == 'EXTERNAL':
            ocp.model.cost_expr_ext_cost = self.acados_model.cost_expr_ext_cost
            ocp.model.cost_expr_ext_cost_e = self.acados_model.cost_expr_ext_cost_e
            if solver_definition.solver.hessian_approx != 'EXACT':
                ocp.model.cost_expr_ext_cost_custom_hess = (
                    self.drone_model.cost_expr_ext_custom_hess
                )
                ocp.model.cost_expr_ext_cost_custom_hess_e = (
                    self.drone_model.cost_expr_ext_custom_hess_e
                )
        else:
            raise ValueError(
                f"Unsupported cost type: {solver_definition.solver.cost_type}. Supported cost types: {self._SUPPORTED_COST_TYPE}")
        
        # Constraints
        constraints = ocp.constraints
        constraints_def = solver_definition.mpc.constraints

        # Initial state
        constraints.x0 = state.vector

        # Hard constraints on inputs
        if constraints_def.idxbu.shape[0] > 0:
            constraints.idxbu = constraints_def.idxbu
            constraints.lbu = np.zeros(constraints_def.idxbu.shape[0])
            constraints.ubu = np.zeros(constraints_def.idxbu.shape[0])

        # Hard constraints on states
        if constraints_def.idxbx.shape[0] > 0:
            # At intermediate shooting nodes (1 to N-1)
            constraints.idxbx = constraints_def.idxbx
            constraints.lbx = np.zeros(constraints_def.idxbx.shape[0])
            constraints.ubx = np.zeros(constraints_def.idxbx.shape[0])
            
            # At terminal shooting node (N)
            constraints.idxbx_e = constraints_def.idxbx
            constraints.lbx_e = np.zeros(constraints_def.idxbx.shape[0])
            constraints.ubx_e = np.zeros(constraints_def.idxbx.shape[0])

        # Soft constraints on states
        if constraints_def.idxsbx.shape[0] > 0:
            # At intermediate shooting nodes (1 to N-1)
            constraints.idxsbx = constraints_def.idxsbx
            constraints.lsbx = constraints_def.lsbx
            constraints.usbx = constraints_def.usbx
            
            # At terminal shooting node (N)
            constraints.idxsbx_e = constraints_def.idxsbx
            constraints.lsbx_e = constraints_def.lsbx
            constraints.usbx_e = constraints_def.usbx

        # Nonlinear constraints within the nonlinear inequalities
        has_lh = constraints_def.lh.shape[0] > 0
        has_uh = constraints_def.uh.shape[0] > 0
        if has_lh != has_uh:
            raise ValueError(
                'Nonlinear constraints require both lh and uh to be set with the same size.')

        has_idxsh = constraints_def.idxsh.shape[0] > 0
        if has_lh and has_uh:
            if constraints_def.lh.shape[0] != constraints_def.uh.shape[0]:
                raise ValueError(
                    f'lh/uh size mismatch: {constraints_def.lh.shape[0]} != {constraints_def.uh.shape[0]}')
            constraints.lh = constraints_def.lh
            constraints.uh = constraints_def.uh
            constraints.lh_e = constraints_def.lh
            constraints.uh_e = constraints_def.uh

            # Soft nonlinear constraints within the indices of nonlinear constraints
            if has_idxsh:
                # Set the soft nonlinear constraint indices
                constraints.idxsh = constraints_def.idxsh
                constraints.idxsh_e = constraints_def.idxsh
                
                # lsh and ush set to 0.0 if not provided, but size must match idxsh
                if constraints_def.lsh.shape[0] == 0:
                    constraints_def.lsh = np.zeros(constraints_def.idxsh.shape[0])
                if constraints_def.ush.shape[0] == 0:
                    constraints_def.ush = np.zeros(constraints_def.idxsh.shape[0])

                # Check size consistency between lsh/ush and idxsh
                if constraints_def.lsh.shape[0] != constraints_def.idxsh.shape[0]:
                    raise ValueError(
                        f'lsh/idxsh size mismatch: {constraints_def.lsh.shape[0]} != {constraints_def.idxsh.shape[0]}')
                if constraints_def.ush.shape[0] != constraints_def.idxsh.shape[0]:
                    raise ValueError(
                        f'ush/idxsh size mismatch: {constraints_def.ush.shape[0]} != {constraints_def.idxsh.shape[0]}')
                
                # Set the soft nonlinear constraint bounds
                constraints.lsh = constraints_def.lsh
                constraints.ush = constraints_def.ush
                constraints.lsh_e = constraints_def.lsh
                constraints.ush_e = constraints_def.ush
        elif has_idxsh:
            raise ValueError(
                'idxsh was provided but nonlinear constraint bounds lh/uh are empty.')

        # Cost for slack constraints. Slack order: [sbx , sbu , sg , sh , sphi]
        if constraints_def.idxsbx.shape[0] > 0 or constraints_def.idxsh.shape[0] > 0:
            # At intermediate shooting nodes (1 to N-1)
            ns = constraints_def.idxsbx.shape[0] + constraints_def.idxsh.shape[0]
            cost.Zl = np.zeros(ns)
            cost.Zu = np.zeros(ns)
            cost.zl = np.zeros(ns)
            cost.zu = np.zeros(ns)

            # At terminal shooting node (N)
            cost.Zl_e = np.zeros(ns)
            cost.Zu_e = np.zeros(ns)
            cost.zl_e = np.zeros(ns)
            cost.zu_e = np.zeros(ns)

        # Solver options
        solver_options = ocp.solver_options
        # number of shooting intervals
        solver_options.N_horizon = solver_definition.mpc.N_horizon
        # prediction horizon
        solver_options.tf = solver_definition.mpc.tf
        # QP solver to be used in the NLP solver. String in (
        # ‘PARTIAL_CONDENSING_HPIPM’, ‘FULL_CONDENSING_QPOASES’, ‘FULL_CONDENSING_HPIPM’,
        # ‘PARTIAL_CONDENSING_QPDUNES’, ‘PARTIAL_CONDENSING_OSQP’, ‘FULL_CONDENSING_DAQP’).
        # Default: ‘PARTIAL_CONDENSING_HPIPM’.
        solver_options.qp_solver = solver_definition.solver.qp_solver
        # NLP solver. String in (‘SQP’, ‘SQP_RTI’, ‘DDP’). Default: ‘SQP_RTI’.
        solver_options.nlp_solver_type = solver_definition.solver.nlp_solver_type
        # Hessian approximation. String in (‘GAUSS_NEWTON’, ‘EXACT’). Default: ‘GAUSS_NEWTON’.
        solver_options.hessian_approx = solver_definition.solver.hessian_approx
        # Integrator type. String in (‘ERK’, ‘IRK’, ‘GNSF’, ‘DISCRETE’, ‘LIFTED_IRK’).
        # Default: ‘ERK’.
        solver_options.integrator_type = solver_definition.solver.integrator_type

        # Create solver
        base_export_dir = solver_definition.solver.export_dir + 'mpc_generated_code/'
        ocp.code_export_directory = base_export_dir + 'mpc_generated_code'
        ocp.json_file = base_export_dir + 'acados_ocp.json'

        self.solver = AcadosOcpSolver(
            ocp,
            generate=solver_definition.solver.generate_c_code and generate_code,
            verbose=solver_definition.solver.verbose,
        )

        return self.solver

    def get_acados_sim_solver(self, solver_definition: SolverDefinition, generate_code: bool = True) -> AcadosSimSolver:
        """
        Get the Acados Integrator Solver.

        :param solver_definition: Solver definition
        :type solver_definition: SolverDefinition
        :param generate_code: Whether to generate C code
        :type generate_code: bool
        :return: Acados Integrator Solver
        :rtype: AcadosSimSolver
        """
        # Create Integrator
        acados_sim = AcadosSim()
        acados_sim.model = self.acados_model
        acados_sim.model.name = self.acados_model.name
        acados_sim.parameter_values = self.Parameters().vector

        # Solver options
        # integrator type. String in (‘ERK’, ‘IRK’, ‘GNSF’, ‘DISCRETE’, ‘LIFTED_IRK’).
        acados_sim.solver_options.integrator_type = solver_definition.integrator.integrator_type
        # number of stages in the integrator
        acados_sim.solver_options.num_stages = solver_definition.integrator.num_stages
        # number of steps in the integrator
        acados_sim.solver_options.num_steps = solver_definition.integrator.num_steps
        # time horizon
        acados_sim.solver_options.T = solver_definition.mpc.tf / solver_definition.mpc.N_horizon

        base_export_dir = solver_definition.solver.export_dir + 'mpc_generated_code/'
        acados_sim.code_export_directory = base_export_dir + 'mpc_generated_code'
        json_file = acados_sim.code_export_directory + 'acados_sim.json'
        self.acados_integrator = AcadosSimSolver(
            acados_sim,
            json_file=json_file,
            generate=solver_definition.integrator.generate_c_code and generate_code,
            verbose=solver_definition.solver.verbose
        )
        return self.acados_integrator



if __name__ == '__main__':
    pass
