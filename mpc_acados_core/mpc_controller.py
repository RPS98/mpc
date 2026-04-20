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

"""MPC Controller base class.

Variant-agnostic controller. Concrete variants subclass :class:`MPCBase`
and bind controller-specific datatype classes (``State``, ``Actuation``,
``OnlineParameters``, ``Parameters``) as class attributes.
"""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import json
from typing import Type, Union

import numpy as np
from acados_template import AcadosOcpSolver

from mpc_acados_core.mpc_datatype import (
    ActuationBounds,
    Gains,
    NonlinearConstraintBounds,
    Reference,
    ReferenceEnd,
    SlackWeights,
    SlackWeightsEnd,
    SoftNonlinearConstraintBounds,
    SoftStateBounds,
    StateBounds,
)


class MPCDataBase:
    """Container for per-iteration MPC data.

    Holds the current state, last actuation, online parameters and
    references. Concrete variants may subclass or simply instantiate
    this class with the correct datatype classes.
    """

    def __init__(
            self,
            mpc_n: int,
            mpc_ny: int,
            mpc_nyn: int,
            x_size: int,
            u_size: int,
            *,
            State: Type,
            Actuation: Type,
            OnlineParameters: Type) -> None:
        """
        Initialize MPC data.

        :param mpc_n: Prediction horizon length (number of shooting intervals).
        :param mpc_ny: Size of the intermediate reference vector.
        :param mpc_nyn: Size of the terminal reference vector.
        :param x_size: Size of the state vector (used by :class:`Reference`).
        :param u_size: Size of the actuation vector (used by :class:`Reference`).
        :param State: Concrete state class provided by the variant.
        :param Actuation: Concrete actuation class provided by the variant.
        :param OnlineParameters: Concrete online-parameters class provided by the variant.
        """
        self.state = State()
        self.actuation = Actuation()
        self.parameters = OnlineParameters(num_stages=mpc_n + 1)
        self.reference = Reference(mpc_n, mpc_ny, x_size, u_size)
        self.reference_end = ReferenceEnd(mpc_nyn)


class MPCBase:
    """Variant-agnostic MPC controller.

    Concrete variants MUST subclass :class:`MPCBase` and set the following
    class attributes to the controller-specific datatype classes:

    - ``State``: State class (e.g. ``mpc_acados_position.state.State``).
    - ``Actuation``: Actuation class.
    - ``OnlineParameters``: OnlineParameters class.
    - ``Parameters``: Parameters class.

    Example::

        class MPC(MPCBase):
            State = _State
            Actuation = _Actuation
            OnlineParameters = _OnlineParameters
            Parameters = _Parameters
    """

    # Variant must override these in a subclass:
    State: Type = None
    Actuation: Type = None
    OnlineParameters: Type = None
    Parameters: Type = None

    # Optional override: concrete MPCData class if the variant wants one.
    MPCData: Type = MPCDataBase

    @classmethod
    def _assert_bound(cls) -> None:
        """Ensure the subclass set all required datatype classes."""
        missing = [
            name for name in ('State', 'Actuation', 'OnlineParameters', 'Parameters')
            if getattr(cls, name) is None
        ]
        if missing:
            raise TypeError(
                f"{cls.__name__} must set class attributes: {', '.join(missing)}"
            )

    def __init__(self, ocp_json_file: str) -> None:
        """
        Initialize the Acados MPC controller.

        :param ocp_json_file: Path to the OCP JSON file produced by Acados.
        """
        self._assert_bound()

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
        self.ns_size = self.idxsbx_size + self.idxsh_size

        self.w_size = int(ocp_json["dims"].get("ny", 0))
        self.we_size = int(ocp_json["dims"].get("ny_e", 0))

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
        print(f'  ns_size: {self.ns_size}')

        # Internal variables
        self._status: int = 0
        self._mpc_data = self.MPCData(
            mpc_n=self.N,
            mpc_ny=self.w_size,
            mpc_nyn=self.we_size,
            x_size=self.x_size,
            u_size=self.u_size,
            State=self.State,
            Actuation=self.Actuation,
            OnlineParameters=self.OnlineParameters,
        )

        self._gains = Gains(self.w_size, self.we_size)
        self._actuation_bounds = ActuationBounds(self.idxbu_size)
        self._state_bounds = StateBounds(self.idxbx_size)
        self._soft_state_bounds = SoftStateBounds(self.idxsbx_size)
        self._slack_weights = SlackWeights(self.ns_size)
        self._slack_weights_end = SlackWeightsEnd(self.ns_size)
        self._nonlinear_constraint_bounds = NonlinearConstraintBounds(self.lh_size)
        self._soft_nonlinear_constraint_bounds = SoftNonlinearConstraintBounds(self.idxsh_size)
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
        if x.shape[0] != self.x_size:
            raise ValueError(
                f"Size mismatch: x has shape {x.shape}, "
                f"but expected size is ({self.x_size},).")
        self.acados_ocp_solver.set(0, 'lbx', x)
        self.acados_ocp_solver.set(0, 'ubx', x)

    def _set_solver_reference(self) -> None:
        """Set the reference of the MPC solver."""
        if self.w_size == 0:
            return
        y_ref = self._mpc_data.reference
        for stage_i in range(self.N):
            y_ref_i = y_ref.get_data(stage_i)
            if y_ref_i.shape[0] != self.w_size:
                raise ValueError(
                    f"Size mismatch: y_ref has shape {y_ref_i.shape}, "
                    f"but expected size is ({self.w_size},).")
            self.acados_ocp_solver.cost_set(stage_i, 'yref', y_ref_i)

    def _set_solver_reference_end(self) -> None:
        """Set the terminal reference of the MPC solver."""
        if self.we_size == 0:
            return
        y_ref_e = self._mpc_data.reference_end.get_data()
        if y_ref_e.shape[0] != self.we_size:
            raise ValueError(
                f"Size mismatch: y_ref_e has shape {y_ref_e.shape}, "
                f"but expected size is ({self.we_size},).")
        self.acados_ocp_solver.cost_set(self.N, 'yref', y_ref_e)

    def _set_solver_parameters(self) -> None:
        """Set the parameters of the MPC solver."""
        for stage_i in range(self.N + 1):
            p = self._mpc_data.parameters.get_data(stage_i)
            if p.shape[0] != self.p_size:
                raise ValueError(
                    f"Size mismatch: p has shape {p.shape}, "
                    f"but expected size is ({self.p_size},).")
            self._status = self.acados_ocp_solver.set(stage_i, 'p', p)

    def solve(self) -> int:
        """Solve the MPC problem."""
        self._set_solver_state()
        self._set_solver_reference()
        self._set_solver_reference_end()
        self._set_solver_parameters()

        self._status = self.acados_ocp_solver.solve()
        self._validate_status(self._status)

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

    def get_data(self) -> MPCDataBase:
        """Get the current MPC data."""
        return self._mpc_data

    def get_parameters(self):
        """Get the current online parameters."""
        return self._mpc_data.parameters

    def set_parameters(
            self,
            parameters: Union['OnlineParameters', 'Parameters', np.ndarray],
            stage: int = -1) -> None:
        """Set online parameters for one stage or for the full horizon."""
        if isinstance(parameters, self.OnlineParameters):
            if stage != -1:
                raise ValueError('stage must be -1 when setting OnlineParameters.')
            if parameters.num_stages != self.N + 1:
                raise ValueError(
                    f'num_stages mismatch: got {parameters.num_stages}, expected {self.N + 1}.')
            self._mpc_data.parameters = parameters
            return

        self._mpc_data.parameters.set_parameters(parameters, stage=stage)

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

    def get_nonlinear_constraint_bounds(self) -> NonlinearConstraintBounds:
        """Get the current nonlinear constraint bounds."""
        return self._nonlinear_constraint_bounds

    def get_soft_nonlinear_constraint_bounds(self) -> SoftNonlinearConstraintBounds:
        """Get the current soft nonlinear constraint bounds."""
        return self._soft_nonlinear_constraint_bounds

    def update_gains(self) -> None:
        """Update the gains in the MPC solver."""
        if self.w_size == 0 and self.we_size == 0:
            return
        for stage_i in range(self.N):
            w = self._gains.get_W()
            self.acados_ocp_solver.cost_set(stage_i, 'W', w)

        self.acados_ocp_solver.cost_set(self.N, 'W', self._gains.get_We())

    def update_actuation_bounds(self) -> None:
        """Update the actuation bounds in the MPC solver."""
        for stage_i in range(self.N):
            u_min = self._actuation_bounds.get_lbu()
            u_max = self._actuation_bounds.get_ubu()
            self.acados_ocp_solver.constraints_set(stage_i, 'lbu', u_min)
            self.acados_ocp_solver.constraints_set(stage_i, 'ubu', u_max)

    def update_state_bounds(self) -> None:
        """Update the state bounds in the MPC solver."""
        if self.idxbx_size == 0:
            return
        for stage_i in range(self.N + 1):
            if stage_i == 0:
                continue  # Skip initial state
            x_min = self._state_bounds.get_lbx()
            x_max = self._state_bounds.get_ubx()
            self.acados_ocp_solver.constraints_set(stage_i, 'lbx', x_min)
            self.acados_ocp_solver.constraints_set(stage_i, 'ubx', x_max)

    def update_soft_state_bounds(self) -> None:
        """Update the soft state bounds in the MPC solver."""
        if self.idxsbx_size == 0:
            return
        if not getattr(self, '_soft_state_bounds_warned', False):
            print('Update soft state bounds is not implemented in python API')
            self._soft_state_bounds_warned = True

    def update_slack_weights(self) -> None:
        """Update the slack weights in the MPC solver."""
        if self.idxsbx_size + self.idxsh_size == 0:
            return
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
        if self.idxsbx_size + self.idxsh_size == 0:
            return
        zl_e = self._slack_weights_end.get_zl_e()
        zu_e = self._slack_weights_end.get_zu_e()
        Zl_e = self._slack_weights_end.get_Zl_e()
        Zu_e = self._slack_weights_end.get_Zu_e()
        self.acados_ocp_solver.cost_set(self.N, 'zl', zl_e)
        self.acados_ocp_solver.cost_set(self.N, 'zu', zu_e)
        self.acados_ocp_solver.cost_set(self.N, 'Zl', Zl_e)
        self.acados_ocp_solver.cost_set(self.N, 'Zu', Zu_e)

    def update_nonlinear_constraint_bounds(self) -> None:
        """Update the nonlinear constraint bounds lh/uh in the MPC solver."""
        if self.lh_size == 0:
            return
        lh = self._nonlinear_constraint_bounds.get_lh()
        uh = self._nonlinear_constraint_bounds.get_uh()
        for stage_i in range(1, self.N):
            self.acados_ocp_solver.constraints_set(stage_i, 'lh', lh)
            self.acados_ocp_solver.constraints_set(stage_i, 'uh', uh)

        self.acados_ocp_solver.constraints_set(self.N, 'lh', lh)
        self.acados_ocp_solver.constraints_set(self.N, 'uh', uh)

    def update_soft_nonlinear_constraint_bounds(self) -> None:
        """Update the soft nonlinear constraint bounds lsh/ush in the MPC solver."""
        if self.idxsh_size == 0:
            return
        if not getattr(self, '_soft_nonlinear_constraint_bounds_warned', False):
            print('Update soft nonlinear_constraint bounds is not implemented in python API')
            self._soft_nonlinear_constraint_bounds_warned = True
