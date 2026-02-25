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

"""MPC Test using Acados Integrator (C++-equivalent flow)."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
import sys
import time

import numpy as np
from acados_template import AcadosSimSolver

from mpc_position.acados_solver import get_acados_sim_solver
from mpc_position.mpc_controller import MPC, MPCData
from utils.utils import CsvLogger, YamlData, compute_path_facing, read_yaml_params


def set_mpc_reference_parameters(
        waypoint: np.ndarray,
        mpc: MPC,
        mpc_data: MPCData,
        index: int,
        path_facing: bool = False) -> None:
    """
    Set MPC references and online parameters for one stage.

    :param waypoint: Current waypoint [x, y, z]
    :type waypoint: np.ndarray
    :param mpc: MPC controller
    :type mpc: MPC
    :param mpc_data: MPC data object
    :type mpc_data: MPCData
    :param index: Prediction stage index
    :type index: int
    :param path_facing: Enable path-facing desired orientation
    :type path_facing: bool
    :return: None
    :rtype: None
    """
    waypoint = np.asarray(waypoint, dtype=float)

    current_position = np.asarray(mpc_data.state.position, dtype=float)
    current_orientation = np.asarray(mpc_data.state.orientation, dtype=float)
    desired_orientation = current_orientation.copy()

    if path_facing:
        desired_orientation = compute_path_facing(
            current_position=current_position,
            target_position=waypoint,
            current_orientation=current_orientation,
        )

    prediction_steps = mpc.get_prediction_steps()

    if index == prediction_steps:
        terminal_dim = min(mpc_data.reference_end.we_size, 9)
        for i in range(terminal_dim):
            mpc_data.reference_end.set_data(i, 0.0)
    elif index > prediction_steps:
        raise IndexError(f'Index {index} out of range for N={prediction_steps}.')
    else:
        stage_dim = min(mpc_data.reference.w_size, 9)
        for i in range(stage_dim):
            mpc_data.reference.set_data(index, i, 0.0)
        if mpc_data.reference.w_size > 9:
            mpc_data.reference.set_data(index, 9, float(mpc_data.parameters.mass) * 9.81)

    mpc_data.parameters.desired_position = waypoint[:3]
    mpc_data.parameters.desired_orientation = desired_orientation


def print_progress_bar(progress: float) -> None:
    """
    Print progress bar equivalent to C++ example.

    :param progress: Progress in [0, 1]
    :type progress: float
    :return: None
    :rtype: None
    """
    bar_width = 70
    progress = max(0.0, min(1.0, progress))
    pos = int(bar_width * progress)

    bar = []
    for i in range(bar_width):
        if i < pos:
            bar.append('=')
        elif i == pos:
            bar.append('>')
        else:
            bar.append(' ')

    sys.stdout.write(f"[{''.join(bar)}] {int(progress * 100.0)} %\r")
    sys.stdout.flush()


def _get_prediction_time_step(mpc: MPC) -> float:
    """
    Get prediction time step with compatibility fallbacks.

    :param mpc: MPC controller
    :type mpc: MPC
    :return: Prediction time step
    :rtype: float
    """
    if hasattr(mpc, 'get_prediction_time_step'):
        try:
            return float(mpc.get_prediction_time_step())
        except Exception:
            pass
    if hasattr(mpc, 'dt'):
        return float(getattr(mpc, 'dt'))
    if hasattr(mpc, '_dt'):
        return float(getattr(mpc, '_dt'))
    raise AttributeError('MPC object does not expose prediction time step.')


def _simulator_step(simulator: AcadosSimSolver, mpc_data) -> int:
    """
    Perform one integrator step from MPCData.

    :param simulator: Acados simulator
    :type simulator: AcadosSimSolver
    :param mpc_data: MPC data object
    :type mpc_data: Any
    :return: Simulator status
    :rtype: int
    """
    simulator.set('x', mpc_data.state.vector)
    simulator.set('u', mpc_data.actuation.vector)
    try:
        simulator.set('p', mpc_data.parameters.vector)
    except Exception:
        pass

    status = simulator.solve()
    if status == 0:
        mpc_data.state.vector = simulator.get('x')
    return status


def test_mpc_controller(logger: CsvLogger, mpc: MPC, simulator: AcadosSimSolver, yaml_data: YamlData) -> None:
    """
    Run MPC simulation loop.

    :param logger: CSV logger
    :type logger: CsvLogger
    :param mpc: MPC controller
    :type mpc: MPC
    :param simulator: Acados simulator
    :type simulator: AcadosSimSolver
    :param yaml_data: Simulation configuration
    :type yaml_data: YamlData
    :return: None
    :rtype: None
    """
    tg_max_time = yaml_data.sim_time

    mpc_data = mpc.get_data()
    prediction_steps = mpc.get_prediction_steps()
    dt = _get_prediction_time_step(mpc)

    logger.save(0.0, mpc_data)

    n_iterations = int((tg_max_time + 2.0) / dt) + 1
    mpc_times = np.zeros(n_iterations)
    sim_times = np.zeros(n_iterations)
    total_times = np.zeros(n_iterations)

    pos_index = 0
    hover_time = 2.0

    print('Starting MPC simulation...')
    print(f'Total time: {tg_max_time + hover_time} s')
    print(f'Time step: {dt} s')
    print(f'Prediction steps: {prediction_steps}')

    k = 0
    t = 0.0
    while t < tg_max_time + hover_time + 1e-9:
        print_progress_bar(t / (tg_max_time + hover_time))
        iter_start = time.perf_counter()

        for i in range(prediction_steps + 1):
            set_mpc_reference_parameters(
                waypoint=yaml_data.waypoints[pos_index],
                mpc=mpc,
                mpc_data=mpc_data,
                index=i,
                path_facing=yaml_data.path_facing,
            )

        mpc_start = time.perf_counter()
        mpc_status = mpc.solve()
        mpc_end = time.perf_counter()

        if mpc_status != 0:
            print(f'\nMPC solver failed with status {mpc_status} at time {t:.3f}s')

        sim_start = time.perf_counter()
        sim_status = _simulator_step(simulator, mpc_data)
        sim_end = time.perf_counter()

        if sim_status != 0:
            raise RuntimeError(f'acados integrator returned status {sim_status}. Exiting.')

        mpc_times[k] = mpc_end - mpc_start
        sim_times[k] = sim_end - sim_start
        total_times[k] = sim_end - iter_start

        logger.save(t, mpc_data)

        error = np.linalg.norm(mpc_data.state.position - yaml_data.waypoints[pos_index])
        if error < 0.1 and pos_index < len(yaml_data.waypoints) - 1:
            pos_index += 1
            print(f'\nWaypoint {pos_index} reached at time {t:.3f} s')
            print(f'Next target: {yaml_data.waypoints[pos_index]}')

        k += 1
        t += dt

    print('\n')
    logger.close()

    mpc_avg_time = np.mean(mpc_times[:k])
    sim_avg_time = np.mean(sim_times[:k])
    total_avg_time = np.mean(total_times[:k])

    print('\n=== Time Statistics ===')
    print(f'MPC average time: {mpc_avg_time * 1000.0:.3f} ms')
    print(f'Simulator average time: {sim_avg_time * 1000.0:.3f} ms')
    print(f'Total average time: {total_avg_time * 1000.0:.3f} ms')
    if total_avg_time > 0.0:
        print(f'Real-time factor: {dt / total_avg_time:.3f}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Run MPC example with configurable simulation config and log file')
    parser.add_argument(
        '-c', '--config_path',
        type=str,
        default='examples/simulation_config.yaml',
        help='Path to simulation configuration yaml (default: examples/simulation_config.yaml)'
    )
    parser.add_argument(
        '-f', '--file_name',
        type=str,
        default='mpc_log.csv',
        help='CSV file name where logs will be saved (default: mpc_log.csv)'
    )
    args = parser.parse_args()

    print(f'Using simulation config from: {args.config_path}')
    yaml_data = read_yaml_params(args.config_path)

    mpc = MPC(ocp_json_file=yaml_data.ocp_json_file_path)

    gains = mpc.get_gains()
    gains.set_Q(yaml_data.mpc_data.Q)
    gains.set_Qe(yaml_data.mpc_data.Qe)
    gains.set_R(yaml_data.mpc_data.R)

    mpc.get_actuation_bounds().set_lbu(yaml_data.mpc_data.lbu)
    mpc.get_actuation_bounds().set_ubu(yaml_data.mpc_data.ubu)

    mpc.get_state_bounds().set_lbx(yaml_data.mpc_data.lbx)
    mpc.get_state_bounds().set_ubx(yaml_data.mpc_data.ubx)

    mpc.get_soft_state_bounds().set_lsbx(yaml_data.mpc_data.lsbx)
    mpc.get_soft_state_bounds().set_usbx(yaml_data.mpc_data.usbx)

    mpc.get_slack_weights().set_Zl(yaml_data.mpc_data.Zl)
    mpc.get_slack_weights().set_Zu(yaml_data.mpc_data.Zu)
    mpc.get_slack_weights().set_zl(yaml_data.mpc_data.zl)
    mpc.get_slack_weights().set_zu(yaml_data.mpc_data.zu)

    mpc.get_slack_weights_end().set_Zl_e(yaml_data.mpc_data.Zl_e)
    mpc.get_slack_weights_end().set_Zu_e(yaml_data.mpc_data.Zu_e)
    mpc.get_slack_weights_end().set_zl_e(yaml_data.mpc_data.zl_e)
    mpc.get_slack_weights_end().set_zu_e(yaml_data.mpc_data.zu_e)

    mpc.update_actuation_bounds()
    mpc.update_state_bounds()
    mpc.update_soft_state_bounds()
    mpc.update_slack_weights()
    mpc.update_slack_weights_end()
    mpc.update_gains()

    mpc_data = mpc.get_data()
    if yaml_data.mpc_data.p.size > 0:
        mpc_data.parameters.mass = float(yaml_data.mpc_data.p[0])

    integrator = get_acados_sim_solver(config_path=yaml_data.solver_definition_path)

    logger = CsvLogger(args.file_name)
    test_mpc_controller(logger, mpc, integrator, yaml_data)
