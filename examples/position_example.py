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

"""MPC Test using Acados Integrator."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from functools import wraps

from mpc.mpc_controller import MPC, mpc_lib
from mpc.read_config import read_mpc_params
import numpy as np
import time
from tqdm import tqdm
from utils.position_utils import euler_to_quaternion, read_yaml_params, CsvLogger, YamlData

SIM_TIME = 60.0


def position_reference_to_mpc_reference(position_reference, ref_yaw: float = 0.0):
    """Convert trajectory point to MPC reference."""
    return mpc_lib.CaState.get_state(
        position=position_reference,
        orientation=euler_to_quaternion(0.0, 0.0, ref_yaw),
    )


def progress_bar(func):
    @wraps(func)
    def wrapper(mpc, simulator, trajectory_generator, logger, *args, **kwargs):
        sim_max_t = SIM_TIME

        pbar = tqdm(total=sim_max_t, desc=f'Progress {func.__name__}', unit='iter',
                    bar_format='{l_bar}{bar} | {n:.4f}/{total:.2f} '
                    '[{elapsed}<{remaining}, {rate_fmt}]')

        result = func(mpc, simulator, trajectory_generator, logger, pbar, *args, **kwargs)

        pbar.close()
        return result
    return wrapper


@progress_bar
def test_trajectory_controller(
        mpc: MPC,
        integrator: mpc_lib.AcadosSimSolver,
        simulation_data: YamlData,
        logger: CsvLogger,
        pbar):
    """Test trajectory controller."""
    x = mpc_lib.CaState.get_state(position=(14.0, 25.0, 1.23))
    u = mpc_lib.CaControl.get_control()
    y = mpc_lib.CaState.get_state()

    prediction_steps = mpc.prediction_steps
    prediction_horizon = mpc.prediction_horizon
    tf = prediction_horizon / prediction_steps

    t = 0.0
    max_time = SIM_TIME

    mpc_solve_times = np.zeros(0)
    state_history = np.zeros(7)
    reference_history = np.zeros(7)

    position_references = simulation_data.sim_params.waypoints
    pos_index = 0
    pos_ref = position_reference_to_mpc_reference(position_references[0])

    logger.save(t, x, y, u)
    flag_update_params = True
    while t < max_time:
        reference = np.zeros((prediction_steps + 1, mpc.x_dim))
        # print(reference_trajectory.shape)  # (101, 10)
        for i in range(prediction_steps + 1):
            reference[i, :] = position_reference_to_mpc_reference(
                position_references[pos_index])
        current_time = time.time()
        u = mpc.evaluate(x, reference[:-1], reference[-1][:mpc.x_dim])
        mpc_solve_times = np.append(mpc_solve_times, time.time() - current_time)

        integrator.set('x', x)
        integrator.set('u', u)
        status = integrator.solve()
        if status != 0:
            raise Exception(
                'acados integrator returned status {}. Exiting.'.format(status))
        x = integrator.get('x')

        t += tf

        # Update history
        state_history = np.vstack((state_history, x[:7]))
        reference_history = np.vstack((reference_history, reference[0][:7]))

        # Log data
        y = mpc_lib.CaState.get_state(
            position=reference[0][0:3],
            orientation=reference[0][3:7],
            linear_velocity=reference[0][7:10])
        logger.save(t, x, y, u)

        # Compute error between current state x and reference state reference[0][0:3]
        error = np.linalg.norm(x[:3] - reference[0][:3])
        if error < 0.1 and pos_index < len(position_references) - 1:
            pos_index += 1
            pos_ref = position_reference_to_mpc_reference(position_references[pos_index])
            print(f'Position reference updated to {pos_ref[:3]} at time {t:.2f}s')

        pbar.update(tf)
        if pos_ref[2] == 1.5 and flag_update_params:
            print(f"Updating MPC parameters at time {t:.2f}s")
            mpc.update_params(simulation_data.mpc_data)
            flag_update_params = False
    print(f'MPC solve time mean: {np.mean(mpc_solve_times)}')


if __name__ == '__main__':
    # Params
    simulation_yaml = read_yaml_params('examples/position_config.yaml')

    # MPC Params
    yaml_data = read_mpc_params('mpc_config.yaml')
    # Logger
    file_name = 'mpc_log.csv'
    logger = CsvLogger(file_name)

    mpc = MPC(
        prediction_steps=yaml_data.N_horizon,
        prediction_horizon=yaml_data.tf,
        params=yaml_data.mpc_params
    )

    Tf = mpc.prediction_horizon
    N = mpc.prediction_steps
    dt = Tf / N
    integrator = mpc.export_integrador(dt)

    test_trajectory_controller(
        mpc,
        integrator,
        simulation_yaml,
        logger)
