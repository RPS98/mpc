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
from mpc.mpc_controller import MPC, MPCCost, MPCConstraints
from mpc.acados_solver import get_acados_sim_solver
from mpc.utils.yaml_to_dict import yaml_to_dict
from utils.utils import euler_to_quaternion, CsvLogger
from mpc.model_definition.actuation import Actuation
from mpc.model_definition.state import State
from mpc.model_definition.parameters import Parameters
import numpy as np
import time
from tqdm import tqdm


def progress_bar(func):
    @wraps(func)
    def wrapper(mpc, simulator, yaml_data, logger, *args, **kwargs):
        sim_max_t = yaml_data.sim_config.sim_time

        pbar = tqdm(total=sim_max_t, desc=f'Progress {func.__name__}', unit='iter',
                    bar_format='{l_bar}{bar} | {n:.4f}/{total:.2f} '
                    '[{elapsed}<{remaining}, {rate_fmt}]')

        result = func(mpc, simulator, yaml_data, logger, pbar, *args, **kwargs)

        pbar.close()
        return result
    return wrapper


@progress_bar
def test_controller(
        mpc: MPC,
        integrator: AcadosSimSolver,
        simulation_data: dict,
        logger: CsvLogger,
        pbar):
    """Test trajectory controller."""
    # MPC parameters
    mass = simulation_data.controller.mpc.parameters.mass
    prediction_steps = mpc.N
    dt = mpc.dt
    
    x = State().vector
    u = Actuation().vector
    p = mpc.parameters
    y_ref_0 = np.array([
        0.0, 0.0, 0.0,  # position
        0.0, 0.0, 0.0,  # orientation (Euler angles)
        0.0, 0.0, 0.0,   # linear velocity
        9.8, 0.0, 0.0, 0.0   # actuation (thrust + torques)
    ])

    t = 0.0
    max_time = yaml_data.sim_config.sim_time

    position_references = simulation_data.sim_config.waypoints
    pos_index = 0

    mpc_solve_times = np.zeros(0)
    reference_setpoint = np.zeros(10)

    logger.save(t, x, y_ref_0, u)
    while t <= max_time:
        t_eval = t
        for i in range(prediction_steps + 1):
            ref_position = position_references[pos_index]
            ref_velocity = np.zeros(3)
            ref_yaw = 0.0
            if simulation_data.sim_config.path_facing:
                # Compute yaw to face the next waypoint from x position
                x_diff = ref_position[0] - x[0]
                y_diff = ref_position[1] - x[1]
                if np.linalg.norm(np.array([x_diff, y_diff])) > 0.1:
                    ref_yaw = np.arctan2(y_diff, x_diff)
            
            p[i, :] = Parameters(
                mass=mass,desired_position =ref_position[0:3],
                desired_orientation=euler_to_quaternion(0.0, 0.0, ref_yaw), desired_velocity=ref_velocity
            ).vector
            t_eval += dt

        current_time = time.time()
        u = mpc.solve(
            state=x,
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
            p[0][1], p[0][2], p[0][3], # position
            p[0][4], p[0][5], p[0][6], p[0][7],    # orientation (quaternion)
            p[0][8], p[0][9], p[0][10]  # velocity
        ])
        logger.save(t, x, reference_setpoint, u)

        # Compute error between current state x and reference state reference[0][0:3]
        error = np.linalg.norm(x[:3] - reference_setpoint[:3])
        if error < 0.01 and pos_index < len(position_references) - 1:
            pos_index += 1
            print(f'Position reference updated to {position_references[pos_index][:3]} at time {t:.2f}s')

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
    print(f'Using simulation config from: {args.config_path}')
    
    # Params
    yaml_data = yaml_to_dict(args.config_path)

    # MPC
    mpc = MPC(
        ocp_json_file=yaml_data.controller.ocp_json_file_path
    )
    
    # MPC cost
    mpc_cost = MPCCost.from_dict(yaml_data.controller.mpc.cost)
    mpc.set_gains(mpc_cost.Q, mpc_cost.R)
    mpc.set_gain_terminal_state(mpc_cost.Qe)

    # MPC constraints
    mpc_constraints = MPCConstraints.from_dict(yaml_data.controller.mpc.constraints)
    mpc.set_u_bounds(mpc_constraints.lbu, mpc_constraints.ubu)
    mpc.set_x_bounds(mpc_constraints.lbx, mpc_constraints.ubx)
    mpc.set_x_bounds_soft_gains(
        mpc_constraints.zl, mpc_constraints.zu,
        mpc_constraints.Zl, mpc_constraints.Zu
    )
    
    # MPC parameters
    mass = yaml_data.controller.mpc.parameters.mass
    parameters = Parameters(mass=mass)
    mpc.set_parameters(parameters.vector)

    # Integrator
    integrator = get_acados_sim_solver(
        config_path=yaml_data.controller.solver_definition_path
    )

    # Logger
    file_name = args.file_name
    logger = CsvLogger(file_name)

    test_controller(
        mpc,
        integrator,
        yaml_data,
        logger
    )
