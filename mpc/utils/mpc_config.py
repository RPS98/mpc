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

from dataclasses import dataclass

import numpy as np
import yaml


@dataclass
class MPCConstraints:
    """
    Constraints for the MPC problem.
    
    Bounds on u (defines J_bu)
        lbu: Lower bounds on u at intermediate shooting nodes (0 to N-1)
        ubu: Upper bounds on u at intermediate shooting nodes (0 to N-1)
    Bounds on x (defines J_bx and J_bx_e)
        Bounds on x
            lbx: Lower bounds at shooting nodes (1 to N)
            ubx: Upper bounds at shooting nodes (1 to N)
    Soft bounds on x within the indices of bounds on x at stages
        Hessian and gradient for state bounds violation cost at stages
            zl: Gradient wrt lower slack at shooting nodes (1 to N)
            zu: Gradient wrt upper slack at shooting nodes (1 to N)
            Zl: Diagonal of Hessian wrt lower slack at shooting nodes (1 to N)
            Zu: Diagonal of Hessian wrt upper slack at shooting nodes (1 to N)
    Nonlinear constraints within the nonlinear inequalities
        lh: Lower bounds on nonlinear constraints at shooting nodes (1 to N)
        uh: Upper bounds on nonlinear constraints at shooting nodes (1 to N)
    Soft nonlinear constraints within the indices of nonlinear constraints
        Bounds on slacks corresponding to soft nonlinear constraints at shooting nodes (1 to N)
            lsh: Lower bounds on slacks at shooting nodes (1 to N)
            ush: Upper bounds on slacks at shooting nodes (1 to N)
            
    """
    # Bounds on u (defines J_bu)
    lbu: np.ndarray = np.array([])  # Lower bounds on u at intermediate shooting nodes (0 to N-1)
    ubu: np.ndarray = np.array([])  # Upper bounds on u at intermediate shooting nodes (0 to N-1)

    # Bounds on x (defines J_bx and J_bx_e)
    #   Bounds on x
    lbx: np.ndarray = np.array([])  # Lower bounds at shooting nodes (1 to N)
    ubx: np.ndarray = np.array([])  # Upper bounds at shooting nodes (1 to N)
    
    # Hessian wrt slack
    zl: np.ndarray = np.array([])  # Gradient wrt lower slack at shooting nodes (0 to N)
    zu: np.ndarray = np.array([])  # Gradient wrt upper slack at shooting nodes (0 to N)
    Zl: np.ndarray = np.array([])  # Diagonal of Hessian wrt lower slack at shooting nodes (0 to N)
    Zu: np.ndarray = np.array([])  # Diagonal of Hessian wrt upper slack at shooting nodes (0 to N)

    # Nonlinear constraints within the nonlinear inequalities
    lh: np.ndarray = np.array([])  # Lower bounds on nonlinear constraints at shooting nodes (1 to N)
    uh: np.ndarray = np.array([])  # Upper bounds on nonlinear constraints at shooting nodes (1 to N)

    # Soft nonlinear constraints within the indices of nonlinear constraints
    #   Bounds on slacks corresponding to soft nonlinear constraints
    lsh: np.ndarray = np.array([])  # Lower bounds on slacks at shooting nodes (1 to N)
    ush: np.ndarray = np.array([])  # Upper bounds on slacks at shooting nodes (1 to N)
    
    @staticmethod
    def from_dict(constraints_dict: dict) -> 'MPCConstraints':
        """
        Create MPCConstraints from a dictionary.

        :param constraints_dict: Constraints definition dictionary
        :type constraints_dict: dict
        :return: MPCConstraints instance
        :rtype: MPCConstraints
        """
        return MPCConstraints(
            lbu=np.array(constraints_dict.get('lbu', [])),
            ubu=np.array(constraints_dict.get('ubu', [])),
            lbx=np.array(constraints_dict.get('lbx', [])),
            ubx=np.array(constraints_dict.get('ubx', [])),
            zl=np.array(constraints_dict.get('zl', [])),
            zu=np.array(constraints_dict.get('zu', [])),
            Zl=np.array(constraints_dict.get('Zl', [])),
            Zu=np.array(constraints_dict.get('Zu', [])),
            lh=np.array(constraints_dict.get('lh', [])),
            uh=np.array(constraints_dict.get('uh', [])),
            lsh=np.array(constraints_dict.get('lsh', [])),
            ush=np.array(constraints_dict.get('ush', []))
        )


@dataclass
class MPCCost:
    """
    Cost for the MPC problem.
    
    Q: Weight for internal states (stage cost)
    Qe: Weight for end states (terminal cost)
    R: Weight for control inputs (stage cost)
    """
    Q: np.ndarray = np.array([])  # Weight for internal states
    Qe: np.ndarray = np.array([])  # Weight for end states
    R: np.ndarray = np.array([])  # Weight for control inputs

    @staticmethod
    def from_dict(cost_dict: dict) -> 'MPCCost':
        """
        Create MPCCost from a dictionary.

        :param cost_dict: Cost definition dictionary
        :type cost_dict: dict
        :return: MPCCost instance
        :rtype: MPCCost
        """
        return MPCCost(
            Q=np.array(cost_dict.get('Q', [])),
            Qe=np.array(cost_dict.get('Qe', [])),
            R=np.array(cost_dict.get('R', []))
        )


if __name__ == '__main__':
    import yaml
    # Example usage
    example_config = 'mpc_config.yaml'
    
    yaml_data = yaml.safe_load(open(example_config, 'r'))
    mpc_constraints = MPCConstraints.from_dict(yaml_data.get('constraints'))
    mpc_cost = MPCCost.from_dict(yaml_data.get('cost'))
