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

from acados_template import AcadosSimSolver
from mpc.mpc_controller import MPC, MPCParameters
from mpc.acados_solver import get_acados_sim_solver
from mpc.utils.yaml_to_dict import yaml_to_dict
from examples.utils.utils import euler_to_quaternion, get_trajectory_generator, CsvLogger
from mpc.model_definition.actuation import Actuation
from mpc.model_definition.state import State
from mpc.model_definition.parameters import Parameters
import numpy as np
import time
from tqdm import tqdm


def progress_bar(func):
    @wraps(func)
    def wrapper(mpc, simulator, yaml_data, logger, trajectory_generator, *args, **kwargs):
        sim_max_t = trajectory_generator.get_max_time() - mpc.dt

        pbar = tqdm(total=sim_max_t, desc=f'Progress {func.__name__}', unit='iter',
                    bar_format='{l_bar}{bar} | {n:.4f}/{total:.2f} '
                    '[{elapsed}<{remaining}, {rate_fmt}]')

        result = func(mpc, simulator, yaml_data, logger, trajectory_generator, pbar, *args, **kwargs)

        pbar.close()
        return result
    return wrapper


@progress_bar
def test_trajectory_controller(
        mpc: MPC,
        integrator: AcadosSimSolver,
        yaml_data: dict,
        logger: CsvLogger,
        trajectory_generator,
        pbar):
    """Test trajectory controller."""
    # MPC parameters
    mass = mpc.parameters[0][0]
    u_ref = mpc.get_u_ref()
    prediction_steps = mpc.N
    dt = mpc.dt
    
    x = State().vector
    u = Actuation().vector
    y_ref = np.zeros((prediction_steps, mpc.w_size))
    y_ref_e = np.zeros((1, mpc.we_size))
    
    p = mpc.parameters
    y_ref_0 = np.array([
        0.0, 0.0, 0.0,  # position
        0.0, 0.0, 0.0,  # orientation (Euler angles)
        0.0, 0.0, 0.0,   # linear velocity
        u_ref[0], u_ref[1], u_ref[2], u_ref[3]   # control inputs
    ])
    y_ref[0, :] = y_ref_0

    t = 0.0
    min_time = trajectory_generator.get_min_time()
    max_time = trajectory_generator.get_max_time()

    mpc_solve_times = np.zeros(0)
    reference_setpoint = np.zeros(10)

    logger.save(t, x, y_ref_0, u)
    while t <= max_time:
        t_eval = t
        for i in range(prediction_steps + 1):
            if t_eval >= max_time:
                t_eval = max_time - dt
            elif t_eval <= min_time:
                t_eval = min_time
            trajectory_point = trajectory_generator.evaluate_trajectory(t_eval)
            ref_position, ref_velocity, _, ref_yaw = \
                trajectory_point

            p[i, :] = Parameters(
                mass=mass,
                desired_orientation=euler_to_quaternion(0.0, 0.0, ref_yaw)
            ).vector
            
            if i < prediction_steps:
                y_ref[i, :] = np.array([
                    ref_position[0],
                    ref_position[1],
                    ref_position[2],
                    0.0,
                    0.0,
                    0.0,
                    ref_velocity[0],
                    ref_velocity[1],
                    ref_velocity[2],
                    u_ref[0],
                    u_ref[1],
                    u_ref[2],
                    u_ref[3]
                ])
            else:
                y_ref_e = np.array([
                    ref_position[0],
                    ref_position[1],
                    ref_position[2],
                    0.0,
                    0.0,
                    0.0,
                    ref_velocity[0],
                    ref_velocity[1],
                    ref_velocity[2]
                ])
            
            t_eval += dt

        current_time = time.time()
        u = mpc.solve(
            state=x,
            y_ref=y_ref,
            y_ref_e=y_ref_e,
            p=p
        )
        mpc_solve_times = np.append(mpc_solve_times, time.time() - current_time)

        integrator.set('x', x)
        integrator.set('u', u)
        status = integrator.solve()
        if status != 0:
            raise Exception(
                'acados integrator returned status {}. Exiting.'.format(status))
        x = integrator.get('x')

        # Update logger
        reference_setpoint = np.array([
            y_ref[0][0], y_ref[0][1], y_ref[0][2], # position
            p[0][1], p[0][2], p[0][3], p[0][4],    # orientation (quaternion)
            y_ref[0][6], y_ref[0][7], y_ref[0][8]  # velocity
        ])

        pbar.update(dt)
        t += dt
        logger.save(t, x, reference_setpoint, u)
    print(f'MPC solve time mean: {np.mean(mpc_solve_times)}')


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(
        description='Run MPC example with configurable simulation config and log file')
    parser.add_argument(
        '-c', '--config_path',
        type=str,
        default='examples/simulation_config.yaml',
        help='Path to the simulation configuration YAML file (default: examples/simulation_config.yaml)'
    )
    parser.add_argument(
        '-f', '--file_name',
        type=str,
        default='mpc_log.csv',
        help='CSV file name where logs will be saved (default: mpc_log.csv)'
    )
    args = parser.parse_args()

    # Params
    yaml_data = yaml_to_dict(args.config_path)

    # MPC
    mpc_params = yaml_data.controller.mpc
    mpc_params = MPCParameters(
        dt=mpc_params.dt,
        Q=mpc_params.Q,
        Qe=mpc_params.Qe,
        R=mpc_params.R,
        p=mpc_params.p,
        lbu=mpc_params.lbu,
        ubu=mpc_params.ubu,
        lbx=mpc_params.lbx,
        ubx=mpc_params.ubx
    )

    mpc = MPC(
        ocp_json_file=yaml_data.controller.ocp_json_file_path
    )
    print(mpc_params)
    mpc.set_mpc_parameters(mpc_params)
    mpc.set_u_ref(np.array([9.81, 0.0, 0.0, 0.0]))  # Hovering thrust

    # Integrator
    integrator = get_acados_sim_solver(
        config_path=yaml_data.controller.solver_definition_path
    )

    # Logger
    file_name = args.file_name
    logger = CsvLogger(file_name)

    # Trajectory generator
    trajectory_generator = get_trajectory_generator(
        initial_position=np.zeros(3),
        waypoints=yaml_data.sim_config.trajectory_generator_waypoints,
        speed=yaml_data.sim_config.trajectory_generator_max_speed
    )
    trajectory_generator.set_path_facing(yaml_data.sim_config.path_facing)

    test_trajectory_controller(
        mpc,
        integrator,
        yaml_data,
        logger,
        trajectory_generator
    )
