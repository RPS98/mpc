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
    :param lsbx: Lower soft bounds on state.
    :type lsbx: np.ndarray
    :param usbx: Upper soft bounds on state.
    :type usbx: np.ndarray
    """
    dt: float
    p: np.ndarray
    lbu: np.ndarray
    ubu: np.ndarray
    lbx: np.ndarray = None
    ubx: np.ndarray = None
    lsbx: np.ndarray = None
    usbx: np.ndarray = None
    Zl: np.ndarray = None
    Zu: np.ndarray = None
    zl: np.ndarray = None
    zu: np.ndarray = None
    Zl_e: np.ndarray = None
    Zu_e: np.ndarray = None
    zl_e: np.ndarray = None
    zu_e: np.ndarray = None

    def __str__(self):
        return (
            f'dt: {self.dt}\n'
            f'p: {self.p}\n'
            f'lbu: {self.lbu}\n'
            f'ubu: {self.ubu}\n'
            f'lbx: {self.lbx}\n'
            f'ubx: {self.ubx}\n'
            f'lsbx: {self.lsbx}\n'
            f'usbx: {self.usbx}\n'
            f'Zl: {self.Zl}\n'
            f'Zu: {self.Zu}\n'
            f'zl: {self.zl}\n'
            f'zu: {self.zu}\n'
            f'Zl_e: {self.Zl_e}\n'
            f'Zu_e: {self.Zu_e}\n'
            f'zl_e: {self.zl_e}\n'
            f'zu_e: {self.zu_e}\n'
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
        self._time_steps = np.array(ocp_json['solver_options']['time_steps'])

        self._x_size = self.acados_ocp_solver.get(0, 'x').shape[0]
        self._u_size = self.acados_ocp_solver.get(0, 'u').shape[0]
        self._p_size = self.acados_ocp_solver.get(0, 'p').shape[0]

        self._lbu_size  = len(ocp_json["constraints"]["lbu"])
        self._ubu_size  = len(ocp_json["constraints"]["ubu"])
        self._lbx_size  = len(ocp_json["constraints"]["lbx"])
        self._ubx_size  = len(ocp_json["constraints"]["ubx"])
        self._lsbx_size = len(ocp_json["constraints"]["lsbx"])
        self._usbx_size = len(ocp_json["constraints"]["usbx"])

        print('MPC parameters:')
        print(f'Horizon N={self.N}, tf={self.tf}, dt={self.dt}')
        print('Time steps:')
        print(self._time_steps)
        print('Sizes:')
        print(f'  x_size: {self.x_size}')
        print(f'  u_size: {self.u_size}')
        print(f'  p_size: {self.p_size}')
        print(f'  lbu_size: {self.lbu_size}')
        print(f'  ubu_size: {self.ubu_size}')
        print(f'  lbx_size: {self.lbx_size}')
        print(f'  ubx_size: {self.ubx_size}')
        print(f'  lsbx_size: {self.lsbx_size}')
        print(f'  usbx_size: {self.usbx_size}')

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
    def lsbx_size(self) -> int:
        """Get the size of lower soft state bounds."""
        return self._lsbx_size

    @property
    def usbx_size(self) -> int:
        """Get the size of upper soft state bounds."""
        return self._usbx_size

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
        self.set_parameters(parameters.p)
        self.set_u_bounds(parameters.lbu, parameters.ubu)
        self.set_x_bounds(parameters.lbx, parameters.ubx)

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
        if self.lbx_size == 0:
            return  # No state bounds to set
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

    def solve(
            self,
            state: np.ndarray = None,
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
        :type state: np.ndarray
        :param p: The parameters vector.
        :type p: np.ndarray

        :return: The control action u0.
        """
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
