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
    mass = mpc.parameters[0][0]
    u_ref = mpc.get_u_ref()
    prediction_steps = mpc.N
    dt = mpc.dt
    
    x = State(
        position=np.array([0.5, 0.0, 0.0]),
        theta=np.array([0.1])
    ).vector
    u = Actuation().vector
    y_ref = np.zeros((prediction_steps, mpc.w_size))
    y_ref_e = np.zeros((1, mpc.we_size))
    
    y_ref_0 = np.array([
        0.0, # error contouring
        0.0, # error lag
        0.0, 0.0, 0.0,  # orientation (Euler angles)
        0.0, # progress
        u_ref[0], u_ref[1], u_ref[2], u_ref[3] , u_ref[4] # control inputs
    ])
    for i in range (prediction_steps):
        mpc.set_y_ref_per_stage(y_ref_0, i)
    y_ref_e=np.zeros(5)
    mpc.set_y_ref_e(y_ref_e)

    t = 0.0
    max_time = yaml_data.sim_config.sim_time

    position_references = simulation_data.sim_config.waypoints
    tg_references = simulation_data.sim_config.waypoints_tg
    pos_index = 0

    mpc_solve_times = np.zeros(0)

    points = position_references
    tangents = tg_references
    # Create spline
    spline = HermiteSpline(points, tangents)
    params = compute_arc_length_reparametrization(spline, n_samples=200, poly_degree=5)
    print(spline.evaluate(0.9))
    print(spline.evaluate_derivative(0.9))
  

    # print(params)  
    # print(params['points'][0])
    p = Parameters(
        mass = np.array(1.0),
        desired_orientation= np.array([1.0, 0.0, 0.0, 0.0]),
        s1_p= params['points'][0],
        s1_m= params['tangents'][0],
        s2_p= params['points'][1],
        s2_m= params['tangents'][1],
        s3_p= params['points'][2],
        s3_m= params['tangents'][2],
        s_length= params['total_length'],
        s_poly_coeffs = params['poly_coeffs']
    )
    position_ref, tangent_vec = evaluate_arc_length_spline(
        np.array([0.1]),
        [p.s1_p, p.s2_p, p.s3_p],
        [p.s1_m, p.s2_m, p.s3_m],
        [0.0, 1.0, 2.0],
        p.s_length,
        p.s_poly_coeffs,
    )
    print("Position ref:", position_ref)
    print("Tangent vec:", tangent_vec)
    p = p.vector
    
    # for i in range(prediction_steps + 1):
    #     mpc.set_parameters(p.vector, i)
    print(f"parameters: {p}")
    logger.save(t, x, y_ref_0, u)
    
    while t <= max_time:
        t_eval = t

        current_time = time.time()
        print("Solving MPC at time:", t)
        mpc.actuation[0]= np.array([9.81, 0.0, 0.0, 0.0, 0.5])
        u = mpc.solve(
            state=x,
            y_ref=y_ref,
            y_ref_e=y_ref_e,
            p=p
        )
        for k in range (prediction_steps):
            x_k= mpc.acados_ocp_solver.get(k,"x")
            u_k= mpc.acados_ocp_solver.get(k,"u")
            print(f"Predicted state {k}: {x_k}")
            print(f"Predicted control {k}: {u_k}")
        exit(0)
        print("MPC solve time:", time.time() - current_time)
        mpc_solve_times = np.append(mpc_solve_times, time.time() - current_time)

        integrator.set('x', x)
        integrator.set('u', u)
        status = integrator.solve()
        if status != 0:
            raise Exception(
                'acados integrator returned status {}. Exiting.'.format(status))
        x = integrator.get('x')

        print(f'Time: {t:.2f}s, \nState: {x}, \nControl: {u}\n')

        # # Update logger
        # reference_setpoint = np.array([
        #     y_ref[0][0], y_ref[0][1], y_ref[0][2], # position
        #     p[0][1], p[0][2], p[0][3], p[0][4],    # orientation (quaternion)
        #     y_ref[0][6], y_ref[0][7], y_ref[0][8]  # velocity
        # ])
        # logger.save(t, x, reference_setpoint, u)

        # # Compute error between current state x and reference state reference[0][0:3]
        # error = np.linalg.norm(x[:3] - reference_setpoint[:3])
        # if error < 0.1 and pos_index < len(position_references) - 1:
        #     pos_index += 1
        #     print(f'Position reference updated to {position_references[pos_index][:3]} at time {t:.2f}s')

        pbar.update(dt)
        t += dt
        # logger.save(t, x, reference_setpoint, u)
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
    mpc.set_u_ref(np.array([9.81, 0.0, 0.0, 0.0, 1.0]))  # Hovering thrust

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
