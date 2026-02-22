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
"""Solver configuration dataclasses."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from dataclasses import dataclass, field

import numpy as np
import yaml
from mpc.model_definition.parameters import Parameters


@dataclass
class ConstraintsDefinition:
    """
    Constraints definition for the MPC problem.
    
    Bounds on u (defines J_bu)
        idxbu: Indices of bounds on u at intermediate shooting nodes (0 to N-1)
    Bounds on x (defines J_bx and J_bx_e)
        idxbx: Indices of bounds on x at shooting nodes (1 to N)
    Soft bounds on x within the indices of bounds on x at stages
        idxsbx: Indices of bounds on x at shooting nodes (1 to N)
        Bounds on slacks corresponding to soft bounds on x at stages
            lsbx: Lower bounds on slacks at shooting nodes (1 to N)
            usbx: Upper bounds on slacks at shooting nodes (1 to N)
    Nonlinear constraints within the nonlinear inequalities
        lh: Lower bounds on nonlinear constraints at shooting nodes (1 to N)
        uh: Upper bounds on nonlinear constraints at shooting nodes (1 to N)
    Soft nonlinear constraints within the indices of nonlinear constraints
        idxsh: Indices of soft nonlinear constraints at shooting nodes (1 to N)
            
    """
    # Bounds on u (defines J_bu)
    idxbu: np.ndarray = np.array([])  # Indices of bounds on u at intermediate shooting nodes (0 to N-1)

    # Bounds on x (defines J_bx and J_bx_e)
    idxbx: np.ndarray = np.array([])  # Indices of bounds on x at shooting nodes (1 to N)

    # Soft bounds on x within the indices of bounds on x at stages
    idxsbx: np.ndarray = np.array([])  # Indices of soft bounds on x at shooting nodes (1 to N)
    #   Bounds on slacks corresponding to soft bounds on x
    lsbx: np.ndarray = np.array([])  # Lower bounds on slacks at shooting nodes (1 to N)
    usbx: np.ndarray = np.array([])  # Upper bounds on slacks at shooting nodes (1 to N)

    # Soft nonlinear constraints within the indices of nonlinear constraints
    idxsh: np.ndarray = np.array([])  # Indices of soft nonlinear constraints at shooting nodes (1 to N)
    
    @staticmethod
    def from_dict(constraints_dict: dict) -> 'ConstraintsDefinition':
        """
        Create ConstraintsDefinition from a dictionary.

        :param constraints_dict: Constraints definition dictionary
        :type constraints_dict: dict
        :return: ConstraintsDefinition instance
        :rtype: ConstraintsDefinition
        """
        return ConstraintsDefinition(
            idxbu=np.array(constraints_dict.get('idxbu', [])),
            idxbx=np.array(constraints_dict.get('idxbx', [])),
            idxsbx=np.array(constraints_dict.get('idxsbx', [])),
            lsbx=np.array(constraints_dict.get('lsbx', [])),
            usbx=np.array(constraints_dict.get('usbx', [])),
            idxsh=np.array(constraints_dict.get('idxsh', []))
        )


@dataclass
class MPCConfig:
    """MPC configuration parameters."""
    
    N_horizon: int = 0  # Prediction steps
    tf: float = 0.0  # Prediction horizon in seconds
    constraints: ConstraintsDefinition = field(default_factory=ConstraintsDefinition)
    
    @staticmethod
    def from_dict(data: dict) -> 'MPCConfig':
        """
        Create MPCConfig from dictionary.
        
        :param data: Dictionary with MPC configuration
        :type data: dict
        :return: MPCConfig instance
        :rtype: MPCConfig
        """
        return MPCConfig(
            N_horizon=data.get('N_horizon'),
            tf=data.get('tf'),
            constraints=ConstraintsDefinition.from_dict(data.get('constraints'))
        )


@dataclass
class SolverConfig:
    """Solver configuration parameters."""
    
    export_dir: str = ''
    generate_c_code: bool = True
    verbose: bool = True
    cost_type: str = 'NONLINEAR_LS'
    qp_solver: str = 'PARTIAL_CONDENSING_HPIPM'
    nlp_solver_type: str = 'SQP_RTI'
    hessian_approx: str = 'GAUSS_NEWTON'
    integrator_type: str = 'ERK'
    
    @staticmethod
    def from_dict(data: dict) -> 'SolverConfig':
        """
        Create SolverConfig from dictionary.
        
        :param data: Dictionary with solver configuration
        :type data: dict
        :return: SolverConfig instance
        :rtype: SolverConfig
        """
        return SolverConfig(
            export_dir=data.get('export_dir', ''),
            generate_c_code=data.get('generate_c_code', False),
            verbose=data.get('verbose', True),
            cost_type=data.get('cost_type', 'NONLINEAR_LS'),
            qp_solver=data.get('qp_solver', 'PARTIAL_CONDENSING_HPIPM'),
            nlp_solver_type=data.get('nlp_solver_type', 'SQP_RTI'),
            hessian_approx=data.get('hessian_approx', 'GAUSS_NEWTON'),
            integrator_type=data.get('integrator_type', 'ERK')
        )


@dataclass
class IntegratorConfig:
    """Integrator configuration parameters."""
    
    generate_c_code: bool = True
    integrator_type: str = 'ERK'
    num_stages: int = 4
    num_steps: int = 3
    
    @staticmethod
    def from_dict(data: dict) -> 'IntegratorConfig':
        """
        Create IntegratorConfig from dictionary.
        
        :param data: Dictionary with integrator configuration
        :type data: dict
        :return: IntegratorConfig instance
        :rtype: IntegratorConfig
        """
        return IntegratorConfig(
            generate_c_code=data.get('generate_c_code', False),
            integrator_type=data.get('integrator_type', 'ERK'),
            num_stages=data.get('num_stages', 4),
            num_steps=data.get('num_steps', 3)
        )


@dataclass
class CppModuleConfig:
    """C++ module configuration parameters."""
    
    generate: bool = False
    project_name: str = 'acados_mpc_solver'
    
    @staticmethod
    def from_dict(data: dict) -> 'CppModuleConfig':
        """
        Create CppModuleConfig from dictionary.
        
        :param data: Dictionary with C++ module configuration
        :type data: dict
        :return: CppModuleConfig instance
        :rtype: CppModuleConfig
        """
        return CppModuleConfig(
            generate=data.get('generate', False),
            project_name=data.get('project_name', 'acados_mpc_solver')
        )


@dataclass
class SolverDefinition:
    """Complete solver definition with all configuration sections."""
    
    mpc: MPCConfig
    solver: SolverConfig
    integrator: IntegratorConfig
    cpp_module: CppModuleConfig
    
    @staticmethod
    def from_dict(data: dict) -> 'SolverDefinition':
        """
        Create SolverDefinition from dictionary.
        
        :param data: Dictionary with full configuration
        :type data: dict
        :return: SolverDefinition instance
        :rtype: SolverDefinition
        """

        return SolverDefinition(
            mpc=MPCConfig.from_dict(data.get('mpc')),
            solver=SolverConfig.from_dict(data.get('solver')),
            integrator=IntegratorConfig.from_dict(data.get('integrator')),
            cpp_module=CppModuleConfig.from_dict(data.get('cpp_module'))
        )
    
    @staticmethod
    def from_yaml(yaml_path: str) -> 'SolverDefinition':
        """
        Load solver definition from YAML file.
        
        :param yaml_path: Path to YAML configuration file
        :type yaml_path: str
        :return: SolverDefinition instance
        :rtype: SolverDefinition
        """
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        
        return SolverDefinition.from_dict(data)


if __name__ == '__main__':
    # Example usage
    solver_definition = SolverDefinition.from_yaml('solver_definition.yaml')
    print(solver_definition)
