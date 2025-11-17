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

__authors__ = 'Rafael Pérez Seguí, Carmen De Rojas Pita-Romero'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from functools import wraps

from acados_template import AcadosSimSolver
from mpc.mpc_controller import MPC, MPCParameters
from mpc.acados_solver import get_acados_sim_solver
from mpc.utils.yaml_to_dict import yaml_to_dict
from utils.utils import euler_to_quaternion, CsvLogger
from mpc.model_definition.actuation import Actuation
from mpc.model_definition.state import State
from mpc.model_definition.parameters import Parameters
from hermite_spline import HermiteSpline, compute_arc_length_reparametrization
from mpc.spline.spline_evaluation import evaluate_arc_length_spline
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
    prediction_steps = mpc.N
    dt = mpc.dt
    
    x = State(
        position=np.array([0.0, 0.0, 0.0]),
        theta=np.array([0.0]),
        linear_velocity=np.array([0.0,0.0,0.0])
    ).vector
    u = Actuation().vector

    position_references = simulation_data.sim_config.waypoints
    tg_references = simulation_data.sim_config.waypoints_tg

    mpc_solve_times = np.zeros(0)


    # Create spline
    spline = HermiteSpline(position_references, tg_references)
    spline_params = compute_arc_length_reparametrization(spline, n_samples=200, poly_degree=5)

    params_dict = simulation_data.controller.mpc
    p = Parameters(
        mass=params_dict.p_mass,
        desired_orientation=params_dict.p_desired_orientation,
        gains_contour_error=params_dict.p_gains_contour_error,
        gain_lag_error= params_dict.p_gain_lag_error,
        gains_orientation= params_dict.p_gains_orientation,
        gains_actuation= params_dict.p_gains_actuation,
        gains_actuation_theta_velocity= params_dict.p_gains_actuation_theta_velocity,
        gain_progress= params_dict.p_gain_progress,
        s1_p=spline_params['points'][0],
        s1_m=spline_params['tangents'][0],
        s2_p=spline_params['points'][1],
        s2_m=spline_params['tangents'][1],
        s3_p=spline_params['points'][2],
        s3_m=spline_params['tangents'][2],
        s_length=spline_params['total_length'],
        s_poly_coeffs=spline_params['poly_coeffs']
    )

    t = 0.0
    max_time = simulation_data.sim_config.sim_time
    logger.save(t, x, p, u)
    while t <= max_time:
        current_time = time.time()
        # print("Solving MPC at time:", t)
        u = mpc.solve(
            state=x,
            p=p.vector
        )
        # for k in range (prediction_steps):
        #     x_k= mpc.acados_ocp_solver.get(k,"x")
        #     u_k= mpc.acados_ocp_solver.get(k,"u")
        #     print(f"Predicted state {k}: {x_k}")
        #     print(f"Predicted control {k}: {u_k}")
        # print("MPC solve time:", time.time() - current_time)
        mpc_solve_times = np.append(mpc_solve_times, time.time() - current_time)

        integrator.set('x', x)
        integrator.set('u', u)
        status = integrator.solve()
        if status != 0:
            raise Exception(
                'acados integrator returned status {}. Exiting.'.format(status))
        x = integrator.get('x')

        # Theta update for next iteration
        x_1 = mpc.acados_ocp_solver.get(1, "x")
        theta_1 = x_1[10]  # theta is at index 10
        x[10] = theta_1

        # Update logger
        logger.save(t, x, p, u)
        pbar.update(dt)
        t += dt
    print(f'MPC solve time mean: {np.mean(mpc_solve_times)}')


def get_parameters_vector(solver_definition: dict) -> np.ndarray:
    """
    Get the parameters vector.

    :return: Parameters vector
    :rtype: np.ndarray
    """
    p_mass = solver_definition.p_mass
    p_desired_orientation = solver_definition.p_desired_orientation
    p_gains_contour_error = solver_definition.p_gains_contour_error
    p_gain_lag_error = solver_definition.p_gain_lag_error
    p_gains_orientation = solver_definition.p_gains_orientation
    p_gains_actuation = solver_definition.p_gains_actuation
    p_gains_actuation_theta_velocity = solver_definition.p_gains_actuation_theta_velocity
    p_gain_progress = solver_definition.p_gain_progress
    p_s1_p = solver_definition.p_s1_p
    p_s1_m = solver_definition.p_s1_m
    p_s2_p = solver_definition.p_s2_p
    p_s2_m = solver_definition.p_s2_m
    p_s3_p = solver_definition.p_s3_p
    p_s3_m = solver_definition.p_s3_m
    p_s_length = solver_definition.p_s_length
    p_s_poly_coeffs = solver_definition.p_s_poly_coeffs
    p_vector = np.concatenate([
        p_mass,
        p_desired_orientation,
        p_gains_contour_error,
        p_gain_lag_error,
        p_gains_orientation,
        p_gains_actuation,
        p_gains_actuation_theta_velocity,
        p_gain_progress,
        p_s1_p,
        p_s1_m,
        p_s2_p,
        p_s2_m,
        p_s3_p,
        p_s3_m,
        p_s_length,
        p_s_poly_coeffs
    ])
    return p_vector


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
        p=get_parameters_vector(mpc_params),
        lbu=mpc_params.lbu,
        ubu=mpc_params.ubu,
        lbx=mpc_params.lbx,
        ubx=mpc_params.ubx,
        lsbx=mpc_params.lsbx,
        usbx=mpc_params.usbx,
        Zl=mpc_params.Zl,
        Zu=mpc_params.Zu,
        zl=mpc_params.zl,
        zu=mpc_params.zu,
        Zl_e=mpc_params.Zl_e,
        Zu_e=mpc_params.Zu_e,
        zl_e=mpc_params.zl_e,
        zu_e=mpc_params.zu_e,
    )

    mpc = MPC(
        ocp_json_file=yaml_data.controller.ocp_json_file_path
    )
    print(mpc_params)
    mpc.set_mpc_parameters(mpc_params)

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
