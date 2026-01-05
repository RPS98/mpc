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

import argparse
import os
import shutil

import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSim, AcadosSimSolver
from mpc.model_definition.state import State
from mpc.model_definition.actuation import Actuation
from mpc.model_definition.parameters import Parameters
from mpc.drone_model import get_acados_model
from mpc.utils.yaml_to_dict import yaml_to_dict


class AcadosMPCSolver:
    """Acados Solver for Model Predictive Controller."""

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
        # Load the solver definition from the YAML file
        solver_definition = yaml_to_dict(solver_definition_path)

        # Acados model
        self.acados_model = get_acados_model()

        # Acados solver and integrator
        if generate_acados_solver:
            self.solver = self.get_acados_solver(solver_definition, generate_code)
        if generate_acados_simulator:
            self.acados_integrator = self.get_acados_sim_solver(solver_definition, generate_code)

        # Generate C++ project if requested and solvers were generated
        if generate_acados_solver and generate_acados_simulator and generate_code:
            if solver_definition.cpp_module.generate:
                self.generate_cpp_project(solver_definition)

    def get_acados_solver(self, solver_definition: dict, generate_code: bool = True) -> AcadosOcpSolver:
        """
        Get the Acados MPC Solver.

        :param solver_definition: Solver definition dictionary
        :type solver_definition: dict
        :param generate_code: Whether to generate C code
        :type generate_code: bool
        :return: Acados MPC Solver
        :rtype: AcadosOcpSolver
        """
        # Acados solver
        ocp = AcadosOcp()
        ocp.model = self.acados_model
        state = State(
            motor_angular_velocity=np.ones(4) * 1100.0
        )
        actuation = Actuation(
            motor_angular_velocity=np.ones(4) * 0.5
        )

        # Parameters
        ocp.parameter_values = AcadosMPCSolver.get_parameters_vector(solver_definition.mpc)

        # Cost
        cost = ocp.cost

        # Weight matrix at intermediate shooting nodes (1 to N-1)
        cost.W = np.diag(np.zeros(
            self.acados_model.cost_y_expr.shape[0]
        ))
        # Weight matrix at terminal shooting node (N)
        cost.W_e = np.diag(np.zeros(
            self.acados_model.cost_y_expr_e.shape[0]
        ))

        # Reference at intermediate shooting nodes (1 to N-1)
        cost.yref = np.concatenate([
            state.position,  # Position reference
            np.zeros(3),  # Attitude reference
            state.linear_velocity,  # Linear velocity reference
            state.angular_velocity,  # Angular velocity reference
            state.motor_angular_velocity,  # Motor angular velocity reference
            actuation.vector  # Control reference
        ])
        # Reference at terminal shooting node (N)
        cost.yref_e = np.concatenate([
            state.position,  # Position reference
            np.zeros(3),  # Attitude reference
            state.linear_velocity,  # Linear velocity reference
            state.angular_velocity,  # Angular velocity reference
            state.motor_angular_velocity,  # Motor angular velocity reference
        ])

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
        cost.cost_type = solver_definition.solver.cost_type
        cost.cost_type_e = solver_definition.solver.cost_type
        # CasADi expression for nonlinear least squares
        # ocp.model.cost_y_expr = ca.vertcat(ocp.model.x, ocp.model.u)
        # ocp.model.cost_y_expr_e = ocp.model.x

        # Constraints
        constraints = ocp.constraints
        # Initial state
        constraints.x0 = state.vector

        # Hard constraints on inputs
        if hasattr(solver_definition.mpc, 'idxbu') and solver_definition.mpc.idxbu is not None:
            if solver_definition.mpc.idxbu.shape[0] > 0:
                # Indices of bounds on u at shooting nodes (0 to N-1)
                constraints.idxbu = solver_definition.mpc.idxbu
                # Lower bounds on u at shooting nodes (0 to N-1)
                constraints.lbu = solver_definition.mpc.lbu
                # Upper bounds on u at shooting nodes (0 to N-1)
                constraints.ubu = solver_definition.mpc.ubu

        # Hard constraints on states
        if hasattr(solver_definition.mpc, 'idxbx') and solver_definition.mpc.idxbx is not None:
            if solver_definition.mpc.idxbx.shape[0] > 0:
                # Indices of bounds on x at shooting nodes (1 to N)
                constraints.idxbx = solver_definition.mpc.idxbx
                # Lower bounds on x at shooting nodes (1 to N)
                constraints.lbx = solver_definition.mpc.lbx[constraints.idxbx]
                # Upper bounds on x at shooting nodes (1 to N)
                constraints.ubx = solver_definition.mpc.ubx[constraints.idxbx]
            if solver_definition.mpc.idxbx_e.shape[0] > 0:
                # Indices of bounds on x at terminal shooting node (N)
                constraints.idxbx_e = solver_definition.mpc.idxbx
                # Lower bounds on x at terminal shooting node (N)
                constraints.lbx_e = solver_definition.mpc.lbx[constraints.idxbx_e]
                # Upper bounds on x at terminal shooting node (N)
                constraints.ubx_e = solver_definition.mpc.ubx[constraints.idxbx_e]
        
        # Soft constraints on states
        if hasattr(solver_definition.mpc, 'idxsbx') and solver_definition.mpc.idxsbx is not None:
            if solver_definition.mpc.idxsbx.shape[0] > 0:
                # Indices of soft bounds on x within the indices of bounds on x at stages (1 to N-1)
                constraints.idxsbx = solver_definition.mpc.idxsbx
                # Lower bounds on slacks corresponding to soft lower bounds on x at stages (1 to N-1)
                constraints.lsbx = solver_definition.mpc.lsbx[constraints.idxsbx]
                # Upper bounds on slacks corresponding to soft upper bounds on x at stages (1 to N-1)
                constraints.usbx = solver_definition.mpc.usbx[constraints.idxsbx]
                
                # Cost for state bounds violation
                cost.Zl = solver_definition.mpc.Zl
                cost.Zu = solver_definition.mpc.Zu
                cost.zl = solver_definition.mpc.zl
                cost.zu = solver_definition.mpc.zu
        if hasattr(solver_definition.mpc, 'idxsbx_e') and solver_definition.mpc.idxsbx_e is not None:
            if solver_definition.mpc.idxsbx_e.shape[0] > 0:
                # Indices of soft bounds on x within the indices of bounds on x at terminal stage (N)
                constraints.idxsbx_e = solver_definition.mpc.idxsbx_e
                # Lower bounds on slacks corresponding to soft lower bounds on x at terminal stage (N)
                constraints.lsbx_e = solver_definition.mpc.lsbx[constraints.idxsbx_e]
                # Upper bounds on slacks corresponding to soft upper bounds on x at terminal stage (N)
                constraints.usbx_e = solver_definition.mpc.usbx[constraints.idxsbx_e]
                
                # Cost for state bounds violation at terminal stage
                cost.Zl_e = solver_definition.mpc.Zl_e
                cost.Zu_e = solver_definition.mpc.Zu_e
                cost.zl_e = solver_definition.mpc.zl_e
                cost.zu_e = solver_definition.mpc.zu_e

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


    @staticmethod
    def get_parameters_vector(solver_definition: dict) -> np.ndarray:
        """
        Get the parameters vector.

        :return: Parameters vector
        :rtype: np.ndarray
        """
        pv = solver_definition

        parameters: Parameters = Parameters(
            mass=pv.p_mass,
            desired_orientation=pv.p_desired_orientation,
            inertia=pv.p_inertia,
            motors_dx=pv.p_motors_dx,
            motors_dy=pv.p_motors_dy,
            motors_cf=pv.p_motors_cf,
            motors_ct=pv.p_motors_ct,
            motors_tau=pv.p_motors_tau,
            motors_direction=pv.p_motors_direction,
            motors_min_angular_velocity=pv.p_motors_min_angular_velocity,
            motors_max_angular_velocity=pv.p_motors_max_angular_velocity,
        )
        return parameters.vector

    def get_acados_sim_solver(self, solver_definition: dict, generate_code: bool = True) -> AcadosSimSolver:
        """
        Get the Acados Integrator Solver.

        :param solver_definition: Solver definition dictionary
        :type solver_definition: dict
        :param generate_code: Whether to generate C code
        :type generate_code: bool
        :return: Acados Integrator Solver
        :rtype: AcadosSimSolver
        """
        # Create Integrator
        acados_sim = AcadosSim()
        acados_sim.model = self.acados_model
        acados_sim.parameter_values = AcadosMPCSolver.get_parameters_vector(solver_definition.mpc)

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

    def generate_cpp_project(self, solver_definition: dict) -> None:
        """
        Generate C++ interface for the Acados MPC Solver.
        Copies the contents of cpp_interface directory into base_export_dir and updates the project name.

        :param solver_definition: Solver definition dictionary
        :type solver_definition: dict
        :return: None
        :rtype: None
        :raises FileNotFoundError: If base_export_dir does not exist
        :raises FileNotFoundError: If cpp_interface source directory does not exist
        """
        project_name = solver_definition.cpp_module.project_name

        # Get base export directory
        base_export_dir = solver_definition.solver.export_dir
        if not base_export_dir:
            raise ValueError("export_dir is not specified in solver configuration")

        # Convert to absolute path
        base_export_dir_abs = os.path.abspath(base_export_dir)

        # Verify that base_export_dir exists
        if not os.path.exists(base_export_dir_abs):
            raise FileNotFoundError(
                f"Export directory does not exist: {base_export_dir_abs}"
            )

        # Get cpp_interface source directory (inside the mpc module)
        mpc_module_dir = os.path.dirname(os.path.abspath(__file__))
        cpp_interface_src = os.path.join(mpc_module_dir, 'cpp_interface')

        # Verify that cpp_interface source exists
        if not os.path.exists(cpp_interface_src):
            raise FileNotFoundError(
                f"cpp_interface source directory does not exist: {cpp_interface_src}"
            )

        # Copy all contents from cpp_interface to base_export_dir
        for item in os.listdir(cpp_interface_src):
            src_item = os.path.join(cpp_interface_src, item)
            dst_item = os.path.join(base_export_dir_abs, item)
            
            # Remove existing item if it exists
            if os.path.exists(dst_item):
                if os.path.isdir(dst_item):
                    shutil.rmtree(dst_item)
                else:
                    os.remove(dst_item)
            
            # Copy the item
            if os.path.isdir(src_item):
                shutil.copytree(src_item, dst_item)
            else:
                shutil.copy2(src_item, dst_item)
        
        print(f"Copied contents from {cpp_interface_src} to {base_export_dir_abs}")

        # Update CMakeLists.txt with project_name
        cmake_file = os.path.join(base_export_dir_abs, 'CMakeLists.txt')
        if os.path.exists(cmake_file):
            with open(cmake_file, 'r') as f:
                content = f.read()

            # Replace the project name in CMakeLists.txt using regex
            import re
            updated_content = re.sub(
                r'set\(PROJECT_NAME\s+\w+\)',
                f'set(PROJECT_NAME {project_name})',
                content
            )

            with open(cmake_file, 'w') as f:
                f.write(updated_content)
            print(f"Updated project name to '{project_name}' in {cmake_file}")
        else:
            print(f"Warning: {cmake_file} does not exist")


def generate_acados_solver(config_path: str) -> AcadosMPCSolver:
    """
    Generate Acados MPC Solver from configuration file.

    :param config_path: Path to the solver definition YAML file
    :type config_path: str
    :return: Acados MPC Solver
    :rtype: AcadosMPCSolver
    """
    return AcadosMPCSolver(config_path)


def get_acados_sim_solver(config_path: str) -> AcadosSimSolver:
    """
    Generate Acados Integrator Solver from configuration file.

    :param config_path: Path to the solver definition YAML file
    :type config_path: str
    :return: Acados Integrator Solver
    :rtype: AcadosSimSolver
    """
    acados_mpc_solver = AcadosMPCSolver(
        solver_definition_path=config_path,
        generate_acados_solver=False,
        generate_acados_simulator=True,
        generate_code=False
    )
    return acados_mpc_solver.acados_integrator


def main():
    """Main entry point for the Acados MPC Solver generator."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Generate Acados MPC Solver from configuration file.')
    parser.add_argument(
        '-c', '--config_path',
        type=str,
        default='solver_definition.yaml',
        help='Path to the solver definition YAML file (default: solver_definition.yaml)'
    )
    args = parser.parse_args()

    # Generate Code
    acados_solver = generate_acados_solver(args.config_path)


if __name__ == '__main__':
    main()
