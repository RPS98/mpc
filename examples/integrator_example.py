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

import math
import os
import examples.utils.example_utils as utils
from mpc.mpc_controller import MPC, MPCParams, mpc_lib
import numpy as np
from tqdm import tqdm


def trajectory_point_to_mpc_reference(trajectory_point):
    """Convert trajectory point to MPC reference."""
    ref_position, ref_velocity, _, ref_yaw = \
        trajectory_point
    return mpc_lib.CaState.get_state(
        position=ref_position,
        orientation=utils.euler_to_quaternion(0.0, 0.0, ref_yaw),
        linear_velocity=ref_velocity
    )


def progress_bar(func):
    @wraps(func)
    def wrapper(logger, mpc, simulator, trajectory_generator, yaml_data, *args, **kwargs):
        sim_max_t = yaml_data.sim_time

        pbar = tqdm(total=sim_max_t, desc=f'Progress {func.__name__}', unit='iter',
                    bar_format='{l_bar}{bar} | {n:.4f}/{total:.2f} '
                    '[{elapsed}<{remaining}, {rate_fmt}]')

        result = func(logger, mpc, simulator, trajectory_generator, yaml_data, pbar, *args, **kwargs)

        pbar.close()
        return result
    return wrapper


@progress_bar
def test_trajectory_controller(
        logger: utils.CsvLogger,
        mpc: MPC,
        simulator,
        trajectory_generator,
        yaml_data: utils.YamlData,
        pbar):
    """Test trajectory controller."""

    x = mpc_lib.CaState.get_state()
    y = mpc_lib.CaState.get_state()
    u = mpc_lib.CaControl.get_control()

    prediction_steps = mpc.prediction_steps
    prediction_horizon = mpc.prediction_horizon
    tf = prediction_horizon / prediction_steps

    t = 0.0
    min_time = trajectory_generator.get_min_time()
    tg_max_time = trajectory_generator.get_max_time()

    logger.save(t, x, y , u)
    while t < tg_max_time:
        t_eval = t
        reference_trajectory = np.zeros((prediction_steps+1, mpc.x_dim))
        # print(reference_trajectory.shape)  # (101, 10)
        for i in range(prediction_steps+1):
            if t_eval >= tg_max_time:
                t_eval = tg_max_time
            elif t_eval <= min_time:
                t_eval = min_time
            trajectory_point = trajectory_generator.evaluate_trajectory(t_eval)
            reference_trajectory[i, :] = trajectory_point_to_mpc_reference(trajectory_point)
            t_eval += tf
        # u = mpc.compute_control_action(x, reference_trajectory[:-1], reference_trajectory[-1][:mpc.x_dim])
        u = mpc.evaluate(x, reference_trajectory[:-1], reference_trajectory[-1][:mpc.x_dim])

        simulator.set('x', x)
        simulator.set('u', u)
        status = simulator.solve()
        if status != 0:
            raise Exception(
                'acados integrator returned status {}. Exiting.'.format(status))
        x = simulator.get("x")

        t += yaml_data.dt

        y = mpc_lib.CaState.get_state(
            position=reference_trajectory[0][0:3],
            orientation=reference_trajectory[0][3:7],
            linear_velocity=reference_trajectory[0][7:10])
        logger.save(t, x, y, u)

        pbar.update(yaml_data.dt)

    logger.close()


if __name__ == '__main__':
    # Logger
    file_name = 'mpc_log.csv'
    logger = utils.CsvLogger(file_name)
    
    # Params
    yaml_data = utils.read_yaml_params('/home/rafa/mpc/examples/simulation_config.yaml')

    # MPC
    mpc_params = MPCParams(
        prediction_horizon=0.5,
        prediction_steps=100,
        Q=mpc_lib.CaState.get_cost_matrix(
            position_weight=3000*np.ones(3),
            orientation_weight=1.0*np.ones(4),
            linear_velocity_weight=0.0*np.ones(3)
        ),
        R=mpc_lib.CaControl.get_cost_matrix(
            thrust_weight=np.array([1.0]),
            angular_velocity_weight=1.0*np.ones(3)
        ),
        mass=1.0,
        max_thrust=30.0,
        min_thrust=1.0,
        max_w_xy=4*np.pi,
        min_w_xy=-4*np.pi,
        max_w_z=4*np.pi,
        min_w_z=-4*np.pi
    )
    mpc = MPC(mpc_params)

    # Simulator
    Tf = mpc_params.prediction_horizon
    N = mpc_params.prediction_steps
    dt = Tf / N
    simulator = mpc.export_integrador(dt)

    # Trajectory generator
    trajectory_generator = utils.get_trajectory_generator(
        initial_position=np.zeros(3),
        waypoints=yaml_data.waypoints,
        speed=yaml_data.trajectory_generator_max_speed
    )
    
    test_trajectory_controller(
        logger,
        mpc,
        simulator,
        trajectory_generator,
        yaml_data)
