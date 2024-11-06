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
from utils.plot_results import plotSim3D
from utils.utils import euler_to_quaternion, get_trajectory_generator, read_yaml_params


def trajectory_point_to_mpc_reference(trajectory_point):
    """Convert trajectory point to MPC reference."""
    ref_position, ref_velocity, _, ref_yaw = \
        trajectory_point
    return mpc_lib.CaState.get_state(
        position=ref_position,
        orientation=euler_to_quaternion(0.0, 0.0, ref_yaw),
        linear_velocity=ref_velocity
    )


def progress_bar(func):
    @wraps(func)
    def wrapper(mpc, simulator, trajectory_generator, *args, **kwargs):
        sim_max_t = trajectory_generator.get_max_time()

        pbar = tqdm(total=sim_max_t, desc=f'Progress {func.__name__}', unit='iter',
                    bar_format='{l_bar}{bar} | {n:.4f}/{total:.2f} '
                    '[{elapsed}<{remaining}, {rate_fmt}]')

        result = func(mpc, simulator, trajectory_generator, pbar, *args, **kwargs)

        pbar.close()
        return result
    return wrapper


@progress_bar
def test_trajectory_controller(
        mpc: MPC,
        integrator: mpc_lib.AcadosSimSolver,
        trajectory_generator,
        pbar):
    """Test trajectory controller."""
    x = mpc_lib.CaState.get_state()
    u = mpc_lib.CaControl.get_control()

    prediction_steps = mpc.prediction_steps
    prediction_horizon = mpc.prediction_horizon
    tf = prediction_horizon / prediction_steps

    t = 0.0
    min_time = trajectory_generator.get_min_time()
    max_time = trajectory_generator.get_max_time()

    mpc_solve_times = np.zeros(0)
    state_history = np.zeros(7)
    reference_history = np.zeros(7)
    while t < max_time:
        t_eval = t
        reference_trajectory = np.zeros((prediction_steps+1, mpc.x_dim))
        # print(reference_trajectory.shape)  # (101, 10)
        for i in range(prediction_steps+1):
            if t_eval >= max_time:
                t_eval = max_time - tf
            elif t_eval <= min_time:
                t_eval = min_time
            trajectory_point = trajectory_generator.evaluate_trajectory(t_eval)
            reference_trajectory[i, :] = trajectory_point_to_mpc_reference(trajectory_point)
            t_eval += tf

        current_time = time.time()
        u = mpc.evaluate(x, reference_trajectory[:-1], reference_trajectory[-1][:mpc.x_dim])
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
        reference_history = np.vstack((reference_history, reference_trajectory[0][:7]))

        pbar.update(tf)
    print(f'MPC solve time mean: {np.mean(mpc_solve_times)}')
    plotSim3D(state_history, reference_history)


if __name__ == '__main__':
    # Params
    simulation_yaml = read_yaml_params('example/simulation_config.yaml')

    # MPC Params
    yaml_data = read_mpc_params('mpc_config.yaml')

    mpc = MPC(
        prediction_steps=yaml_data.N_horizon,
        prediction_horizon=yaml_data.tf,
        params=yaml_data.mpc_params
    )

    Tf = mpc.prediction_horizon
    N = mpc.prediction_steps
    dt = Tf / N
    integrator = mpc.export_integrador(dt)

    # Trajectory generator
    trajectory_generator = get_trajectory_generator(
        initial_position=np.zeros(3),
        waypoints=simulation_yaml.waypoints,
        speed=simulation_yaml.trajectory_generator_max_speed
    )
    trajectory_generator.set_path_facing(simulation_yaml.path_facing)

    test_trajectory_controller(
        mpc,
        integrator,
        trajectory_generator)
