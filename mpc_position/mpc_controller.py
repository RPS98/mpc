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

import json
from acados_template import AcadosOcpSolver
import numpy as np
from mpc_position.model_definition.actuation import Actuation
from mpc_position.model_definition.parameters import Parameters
from mpc_position.model_definition.state import State
from mpc_position.mpc_datatype import (
    Reference, ReferenceEnd, Gains, ActuationBounds, StateBounds,
    SoftStateBounds, SlackWeights, SlackWeightsEnd
)


class MPCData:
    """Data for MPC."""
    state: State = State()
    actuation: Actuation = Actuation()
    parameters: Parameters = Parameters()
    reference: Reference
    reference_end: ReferenceEnd

    def __init__(self, mpc_n: int, mpc_ny: int, mpc_nyn: int, ) -> None:
        """
        Initialize MPC data.
        
        :param mpc_n: Size of the prediction horizon
        :type mpc_n: int
        :param mpc_ny: Size of the reference state
        :type mpc_ny: int
        :param mpc_nyn: Size of the reference terminal state
        :type mpc_nyn: int
        :return: None
        :rtype: None
        """
        self.reference = Reference(mpc_n, mpc_ny)
        self.reference_end = ReferenceEnd(mpc_nyn)


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
        self.N = self.acados_ocp_solver.N

        # Get time parameters from JSON file (acados_ocp is None when loaded from JSON)
        with open(ocp_json_file, 'r') as f:
            ocp_json = json.load(f)
        self.tf = ocp_json['solver_options']['tf']
        self.dt = self.tf / self.N
        # Alternative: use time_steps directly if non-uniform
        self._time_steps = np.array(ocp_json['solver_options']['time_steps'])

        self.x_size = self.acados_ocp_solver.get(0, 'x').shape[0]
        self.u_size = self.acados_ocp_solver.get(0, 'u').shape[0]
        self.p_size = self.acados_ocp_solver.get(0, 'p').shape[0]

        self.idxbu_size = len(ocp_json["constraints"]["idxbu"])
        self.idxbx_size = len(ocp_json["constraints"]["idxbx"])
        self.idxsbx_size = len(ocp_json["constraints"]["idxsbx"])
        self.lh_size = len(ocp_json["constraints"]["lh"])
        self.idxsh_size = len(ocp_json["constraints"]["idxsh"])

        self.w_size = self.acados_ocp_solver.cost_get(0, 'W').shape[0]
        self.we_size = self.acados_ocp_solver.cost_get(self.N, 'W').shape[0]

        print('MPC parameters:')
        print(f'Horizon N={self.N}, tf={self.tf}, dt={self.dt}')
        print('Sizes:')
        print(f'  x_size: {self.x_size}')
        print(f'  u_size: {self.u_size}')
        print(f'  p_size: {self.p_size}')
        print(f'  w_size: {self.w_size}')
        print(f'  we_size: {self.we_size}')
        print(f'  idxbu_size: {self.idxbu_size}')
        print(f'  idxbx_size: {self.idxbx_size}')
        print(f'  idxsbx_size: {self.idxsbx_size}')
        print(f'  lh_size: {self.lh_size}')
        print(f'  idxsh_size: {self.idxsh_size}')  
        
        # Internal variables
        self._status: int = 0
        self._mpc_data: MPCData = MPCData(
            mpc_n=self.N,
            mpc_ny=self.w_size,
            mpc_nyn=self.we_size
        )
        
        self._gains = Gains(self.w_size, self.we_size)
        self._actuation_bounds = ActuationBounds(self.idxbu_size)
        self._state_bounds = StateBounds(self.idxbx_size)
        self._soft_state_bounds = SoftStateBounds(self.idxsbx_size)
        self._slack_weights = SlackWeights(self.idxsbx_size)
        self._slack_weights_end = SlackWeightsEnd(self.idxsbx_size)
        self._soft_state_bounds_warned = False

    # Private methods
    def _validate_status(self, status: int) -> None:
        """Validate the status of the MPC solver.

        Acados status:
            ACADOS_SUCCESS = 0
            ACADOS_NAN_DETECTED = 1
            ACADOS_MAXITER = 2
            ACADOS_MINSTEP = 3
            ACADOS_QP_FAILURE = 4
            ACADOS_READY = 5
            ACADOS_UNBOUNDED = 6
            
        :param status: The status code returned by the MPC solver.
        :type status: int
        :return: None
        :rtype: None
        """
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

    def _set_solver_state(self) -> None:
        """Set the state of the MPC solver."""
        x = self._mpc_data.state.vector
        # Check size of x
        if x.shape[0] != self.x_size:
            raise ValueError(
                f"Size mismatch: x has shape {x.shape}, "
                f"but expected size is ({self.x_size},).")
        self.acados_ocp_solver.set(0, 'lbx', x)
        self.acados_ocp_solver.set(0, 'ubx', x)
    
    def _set_solver_reference(self) -> None:
        """Set the reference of the MPC solver."""
        y_ref = self._mpc_data.reference
        # Check size of y_ref
        for stage_i in range(self.N):
            y_ref_i = y_ref.get_data(stage_i)
            if y_ref_i.shape[0] != self.w_size:
                raise ValueError(
                    f"Size mismatch: y_ref has shape {y_ref_i.shape}, "
                    f"but expected size is ({self.w_size},).")
            self.acados_ocp_solver.cost_set(stage_i, 'yref', y_ref_i)

    def _set_solver_reference_end(self) -> None:
        """Set the terminal reference of the MPC solver."""
        y_ref_e = self._mpc_data.reference_end.get_data()
        # Check size of y_ref_e
        if y_ref_e.shape[0] != self.we_size:
            raise ValueError(
                f"Size mismatch: y_ref_e has shape {y_ref_e.shape}, "
                f"but expected size is ({self.we_size},).")
        self.acados_ocp_solver.cost_set(self.N, 'yref', y_ref_e)

    def _set_solver_parameters(self) -> None:
        """Set the parameters of the MPC solver."""
        p = self._mpc_data.parameters.vector
        # Check size of p
        if p.shape[0] != self.p_size:
            raise ValueError(
                f"Size mismatch: p has shape {p.shape}, "
                f"but expected size is ({self.p_size},).")
        for stage_i in range(self.N + 1):
            self._status = self.acados_ocp_solver.set(stage_i, 'p', p)
    
    def solve(self) -> int:
        """Solve the MPC problem."""
        # Set solver state and reference
        self._set_solver_state()
        self._set_solver_reference()
        self._set_solver_reference_end()
        self._set_solver_parameters()
        
        # Solve OCP
        self._status = self.acados_ocp_solver.solve()
        self._validate_status(self._status)
        
        # Get solution
        u = self.acados_ocp_solver.get(0, 'u')
        self._mpc_data.actuation.vector = u
        
        return self._status

    # Getters
    def get_prediction_steps(self) -> int:
        """Get the number of prediction steps."""
        return self.N
    
    def get_prediction_horizon(self) -> float:
        """Get the total prediction horizon time."""
        return self.tf
    
    def get_prediction_time_step(self) -> float:
        """Get the time step between prediction stages."""
        return self.dt

    def get_data(self) -> MPCData:
        """Get the current MPC data."""
        return self._mpc_data

    def get_parameters(self) -> Parameters:
        """Get the current parameters."""
        return self._mpc_data.parameters
    
    def set_parameters(self, parameters: Parameters) -> None:
        """Set the parameters."""
        self._mpc_data.parameters = parameters
    
    def get_gains(self) -> Gains:
        """Get the current gains."""
        return self._gains
    
    def get_actuation_bounds(self) -> ActuationBounds:
        """Get the current actuation bounds."""
        return self._actuation_bounds
    
    def get_state_bounds(self) -> StateBounds:
        """Get the current state bounds."""
        return self._state_bounds
    
    def get_soft_state_bounds(self) -> SoftStateBounds:
        """Get the current soft state bounds."""
        return self._soft_state_bounds
    
    def get_slack_weights(self) -> SlackWeights:
        """Get the current slack weights."""
        return self._slack_weights
    
    def get_slack_weights_end(self) -> SlackWeightsEnd:
        """Get the current terminal slack weights."""
        return self._slack_weights_end
    
    def update_gains(self) -> None:
        """Update the gains in the MPC solver."""
        # weight matrix at intermediate shooting nodes (1 to N-1)
        for stage_i in range(self.N):
            w = self._gains.get_W()
            self.acados_ocp_solver.cost_set(stage_i, 'W', w)
        
        # weight matrix at terminal shooting node (N)
        self.acados_ocp_solver.cost_set(self.N, 'W', self._gains.get_We())
    
    def update_actuation_bounds(self) -> None:
        """Update the actuation bounds in the MPC solver."""
        # lower actuation_bounds on u at shooting nodes (0 to N-1)
        # upper actuation_bounds on u at shooting nodes (0 to N-1)
        for stage_i in range(self.N):
            u_min = self._actuation_bounds.get_lbu()
            u_max = self._actuation_bounds.get_ubu()
            self.acados_ocp_solver.constraints_set(stage_i, 'lbu', u_min)
            self.acados_ocp_solver.constraints_set(stage_i, 'ubu', u_max)

    def update_state_bounds(self) -> None:
        """Update the state bounds in the MPC solver."""
        # lower state_bounds on u at shooting nodes (0 to N-1)
        # upper state_bounds on u at shooting nodes (0 to N-1)
        for stage_i in range(self.N + 1):
            if stage_i == 0:
                continue  # Skip initial state
            x_min = self._state_bounds.get_lbx()
            x_max = self._state_bounds.get_ubx()
            self.acados_ocp_solver.constraints_set(stage_i, 'lbx', x_min)
            self.acados_ocp_solver.constraints_set(stage_i, 'ubx', x_max)

    def update_soft_state_bounds(self) -> None:
        """Update the soft state bounds in the MPC solver."""
        if not getattr(self, '_soft_state_bounds_warned', False):
            print('Update soft state bounds is not implemented in python API')
            self._soft_state_bounds_warned = True
        # # lower soft state bounds at shooting nodes (1 to N-1)
        # # upper soft state bounds at shooting nodes (1 to N-1)
        # for stage_i in range(self.N + 1):
        #     if stage_i == 0:
        #         continue  # Skip initial state
        #     lsbx = self._soft_state_bounds.get_lsbx()
        #     usbx = self._soft_state_bounds.get_usbx()
        #     self.acados_ocp_solver.constraints_set(stage_i, 'lsbx', lsbx)
        #     self.acados_ocp_solver.constraints_set(stage_i, 'usbx', usbx)
        # # lower soft state bounds at terminal shooting node (N)
        # # upper soft state bounds at terminal shooting node (N)
        # lsbx_e = self._soft_state_bounds.get_lsbx()
        # usbx_e = self._soft_state_bounds.get_usbx()
        # self.acados_ocp_solver.constraints_set(self.N, 'lsbx', lsbx_e)
        # self.acados_ocp_solver.constraints_set(self.N, 'usbx', usbx_e)
    
    def update_slack_weights(self) -> None:
        """Update the slack weights in the MPC solver."""
        for stage_i in range(self.N + 1):
            if stage_i == 0:
                continue  # Skip initial state
            zl = self._slack_weights.get_zl()
            zu = self._slack_weights.get_zu()
            Zl = self._slack_weights.get_Zl()
            Zu = self._slack_weights.get_Zu()
            self.acados_ocp_solver.cost_set(stage_i, 'zl', zl)
            self.acados_ocp_solver.cost_set(stage_i, 'zu', zu)
            self.acados_ocp_solver.cost_set(stage_i, 'Zl', Zl)
            self.acados_ocp_solver.cost_set(stage_i, 'Zu', Zu)

    def update_slack_weights_end(self) -> None:
        """Update the terminal slack weights in the MPC solver."""
        zl_e = self._slack_weights_end.get_zl_e()
        zu_e = self._slack_weights_end.get_zu_e()
        Zl_e = self._slack_weights_end.get_Zl_e()
        Zu_e = self._slack_weights_end.get_Zu_e()
        self.acados_ocp_solver.cost_set(self.N, 'zl', zl_e)
        self.acados_ocp_solver.cost_set(self.N, 'zu', zu_e)
        self.acados_ocp_solver.cost_set(self.N, 'Zl', Zl_e)
        self.acados_ocp_solver.cost_set(self.N, 'Zu', Zu_e)


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
