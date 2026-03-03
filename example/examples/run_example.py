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

"""MPC example using the current object-based controller API."""

__authors__ = 'Rafael Perez-Segui'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
from functools import wraps
import time

import numpy as np
from acados_template import AcadosSimSolver

from mpc.acados_solver import get_acados_sim_solver
from mpc.mpc_controller import MPC, MPCData
from mpc.utils.yaml_to_dict import yaml_to_dict
from tqdm import tqdm

from utils.trajectory_generator_util import get_trajectory_generator
from utils.utils import CsvLogger, compute_path_facing


def _to_numpy_array(value, default=None) -> np.ndarray:
    """Convert config values to a flat numpy array."""
    if value is None:
        if default is None:
            return np.array([], dtype=float)
        value = default

    if isinstance(value, np.ndarray):
        return value.astype(float, copy=False)
    if isinstance(value, (list, tuple)):
        return np.asarray(value, dtype=float)
    return np.asarray([value], dtype=float)


def _get_cfg_section(config, section_name):
    """Return a nested config section, falling back to the parent object."""
    if hasattr(config, section_name):
        return getattr(config, section_name)
    return config


def _simulator_step(integrator: AcadosSimSolver, mpc_data: MPCData) -> int:
    """Perform one integrator step from MPCData."""
    integrator.set('x', mpc_data.state.vector)
    integrator.set('u', mpc_data.actuation.vector)
    try:
        integrator.set('p', mpc_data.parameters.get_data(0))
    except Exception:
        pass
    status = integrator.solve()
    if status != 0:
        raise RuntimeError(f'acados integrator returned status {status}. Exiting.')
    mpc_data.state.vector = integrator.get('x')
    return status


def create_trajectory_generator(yaml_data):
    """Create and configure the trajectory generator for the example."""
    desired_speed = yaml_data.sim_config.max_speed
    waypoints = yaml_data.sim_config.waypoints
    trajectory_generator = get_trajectory_generator(
        initial_position=np.zeros(3),
        waypoints=waypoints,
        speed=desired_speed
    )
    print(f'Created trajectory generator with {len(waypoints)} and '
          f'waypoints and desired speed {desired_speed} m/s.')
    trajectory_generator.set_path_facing(yaml_data.sim_config.path_facing)
    return trajectory_generator


def set_stage_reference(
        mpc_data: MPCData,
        desired_position: np.ndarray,
        desired_orientation: np.ndarray,
        desired_velocity: np.ndarray,
        hover_thrust: float,
        prediction_steps: int,
        stage: int) -> None:
    """Set the stage or terminal yref for one prediction index."""
    if stage < prediction_steps:
        mpc_data.reference.set_y_ref(np.array([
            desired_position[0],
            desired_position[1],
            desired_position[2],
            0.0,
            0.0,
            0.0,
            desired_velocity[0],
            desired_velocity[1],
            desired_velocity[2],
            hover_thrust,
            0.0,
            0.0,
            0.0
        ]), stage=stage)
        
        mpc_data.parameters.set_desired_orientation(desired_orientation, stage=stage)
        return

    mpc_data.reference_end.set_y_ref_e(np.array([
        desired_position[0],
        desired_position[1],
        desired_position[2],
        0.0,
        0.0,
        0.0,
        desired_velocity[0],
        desired_velocity[1],
        desired_velocity[2]
    ]))
    mpc_data.parameters.set_desired_orientation(desired_orientation, stage=stage)


def configure_mpc(mpc: MPC, yaml_data) -> None:
    """Apply gains, bounds and constant online parameters from YAML."""
    mpc_data = mpc.get_data()
    mpc_cfg = yaml_data.controller.mpc
    cost_cfg = _get_cfg_section(mpc_cfg, 'cost')
    constraints_cfg = _get_cfg_section(mpc_cfg, 'constraints')
    parameters_cfg = _get_cfg_section(mpc_cfg, 'parameters')

    # Parameters
    mass_values = _to_numpy_array(
        parameters_cfg.get('mass'),
        default=np.array([1.0], dtype=float))
    mpc_data.parameters.set_mass(mass_values)
    desired_orientation = _to_numpy_array(
        parameters_cfg.get('desired_orientation'),
        default=np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    )
    mpc_data.parameters.set_desired_orientation(desired_orientation)
    external_force = _to_numpy_array(
        parameters_cfg.get('external_force'),
        default=np.array([0.0, 0.0, 0.0], dtype=float)
    )
    mpc_data.parameters.set_external_force(external_force)

    # Gains
    gains = mpc.get_gains()
    gains.set_Q(cost_cfg.Q)
    gains.set_Qe(cost_cfg.Qe)
    gains.set_R(cost_cfg.R)
    
    # Bounds
    lbu = _to_numpy_array(constraints_cfg.get('lbu'))
    ubu = _to_numpy_array(constraints_cfg.get('ubu'))
    mpc.get_actuation_bounds().set_lbu(lbu)
    mpc.get_actuation_bounds().set_ubu(ubu)

    lbx = _to_numpy_array(constraints_cfg.get('lbx'))
    ubx = _to_numpy_array(constraints_cfg.get('ubx'))
    mpc.get_state_bounds().set_lbx(lbx)
    mpc.get_state_bounds().set_ubx(ubx)

    lbx = _to_numpy_array(constraints_cfg.get('lsbx'))
    ubx = _to_numpy_array(constraints_cfg.get('usbx'))
    mpc.get_soft_state_bounds().set_lsbx(lbx)
    mpc.get_soft_state_bounds().set_usbx(ubx)

    Zl = _to_numpy_array(constraints_cfg.get('Zl'))
    Zu = _to_numpy_array(constraints_cfg.get('Zu'))
    zl = _to_numpy_array(constraints_cfg.get('zl'))
    zu = _to_numpy_array(constraints_cfg.get('zu'))
    mpc.get_slack_weights().set_Zl(Zl)
    mpc.get_slack_weights().set_Zu(Zu)
    mpc.get_slack_weights().set_zl(zl)
    mpc.get_slack_weights().set_zu(zu)
    
    Zl_e = _to_numpy_array(constraints_cfg.get('Zl_e', default=Zl))
    Zu_e = _to_numpy_array(constraints_cfg.get('Zu_e', default=Zu))
    zl_e = _to_numpy_array(constraints_cfg.get('zl_e', default=zl))
    zu_e = _to_numpy_array(constraints_cfg.get('zu_e', default=zu))
    mpc.get_slack_weights_end().set_Zl_e(Zl_e)
    mpc.get_slack_weights_end().set_Zu_e(Zu_e)
    mpc.get_slack_weights_end().set_zl_e(zl_e)
    mpc.get_slack_weights_end().set_zu_e(zu_e)

    # Update MPC with the new values
    mpc.update_actuation_bounds()
    mpc.update_state_bounds()
    mpc.update_soft_state_bounds()
    mpc.update_slack_weights()
    mpc.update_slack_weights_end()
    mpc.update_gains()


def progress_bar(func):
    @wraps(func)
    def wrapper(mpc, simulator, yaml_data, logger, trajectory_generator, *args, **kwargs):
        sim_max_t = trajectory_generator.get_max_time()

        pbar = tqdm(total=sim_max_t, desc=f'Progress {func.__name__}', unit='iter',
                    bar_format='{l_bar}{bar} | {n:.4f}/{total:.2f} '
                    '[{elapsed}<{remaining}, {rate_fmt}]')

        result = func(mpc, simulator, yaml_data, logger, trajectory_generator, pbar, *args, **kwargs)

        pbar.close()
        return result
    return wrapper


@progress_bar
def test_mpc_controller(
        mpc: MPC,
        simulator: AcadosSimSolver,
        yaml_data: dict,
        logger: CsvLogger,
        trajectory_generator,
        pbar: tqdm) -> None:
    """Test trajectory controller."""
    mpc_data = mpc.get_data()
    prediction_steps = mpc.get_prediction_steps()
    dt = mpc.get_prediction_time_step()
    hover_thrust = float(mpc_data.parameters.get_mass()) * 9.81

    # Sim time
    min_time = trajectory_generator.get_min_time()
    max_time = trajectory_generator.get_max_time()

    # First log
    logger.save(
        0.0,
        np.zeros(3),
        np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
        np.zeros(3),
        np.zeros(3),
        np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
        np.zeros(3),
        0.0,
        np.zeros(3)
    )

    # Arrays to store timing data
    mpc_times = np.array([])
    sim_times = np.array([])
    total_times = np.array([])

    print('Starting MPC simulation...')
    print(f'Total time: {max_time} s')
    print(f'Time step: {dt} s')
    print(f'Prediction steps: {prediction_steps}')

    t = 0.0
    while t <= max_time:
        t += dt
        iter_start = time.perf_counter()

        # Set references
        desired_orientation = mpc_data.state.orientation.copy()
        t_eval = t
        for i in range(prediction_steps + 1):
            if t_eval >= max_time:
                t_eval = max_time - dt
            elif t_eval <= min_time:
                t_eval = min_time
            desired_position, desired_velocity, _, _ = trajectory_generator.evaluate_trajectory(t_eval)

            if yaml_data.sim_config.path_facing:
                desired_orientation = compute_path_facing(desired_velocity)
            
            set_stage_reference(
                mpc_data,
                desired_position,
                desired_orientation,
                desired_velocity,
                hover_thrust,
                prediction_steps,
                i,
            )
            t_eval += dt

        # Solve MPC
        mpc_start = time.perf_counter()
        mpc_status = mpc.solve()
        mpc_end = time.perf_counter()
        if mpc_status != 0:
            print(f'\nMPC solver failed with status {mpc_status} at time {t:.3f}s')

        # Simulate
        sim_start = time.perf_counter()
        sim_status = _simulator_step(simulator, mpc_data)
        sim_end = time.perf_counter()
        if sim_status != 0:
            raise RuntimeError(f'acados integrator returned status {sim_status}. Exiting.')
        
        # Log times
        mpc_times = np.append(mpc_times, mpc_end - mpc_start)
        sim_times = np.append(sim_times, sim_end - sim_start)
        total_times = np.append(total_times, sim_end - iter_start)

        # Log data
        logger.save(
            t,
            mpc_data.state.position,
            mpc_data.state.orientation,
            mpc_data.state.linear_velocity,
            mpc_data.reference.get_data(0)[0:3], # Position
            mpc_data.parameters.get_desired_orientation(0),
            mpc_data.reference.get_data(0)[6:9], # Linear velocity
            mpc_data.actuation.thrust,
            mpc_data.actuation.angular_velocity
        )

        # Update progress bar
        pbar.update(dt)

    print('\n')
    logger.close()

    mpc_avg_time = np.mean(mpc_times)
    sim_avg_time = np.mean(sim_times)
    total_avg_time = np.mean(total_times)

    print('\n=== Time Statistics ===')
    print(f'MPC average time: {mpc_avg_time * 1000.0:.3f} ms')
    print(f'Simulator average time: {sim_avg_time * 1000.0:.3f} ms')
    print(f'Total average time: {total_avg_time * 1000.0:.3f} ms')
    if total_avg_time > 0.0:
        print(f'Real-time factor: {dt / total_avg_time:.3f}')


def main() -> None:
    """Run the trajectory-tracking MPC example."""
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

    mpc = MPC(
        ocp_json_file=yaml_data.controller.ocp_json_file_path
    )
    configure_mpc(mpc, yaml_data)

    # Integrator
    integrator = get_acados_sim_solver(
        config_path=yaml_data.controller.solver_definition_path
    )

    # Logger
    file_name = args.file_name
    logger = CsvLogger(file_name)

    trajectory_generator = create_trajectory_generator(yaml_data)

    test_mpc_controller(
        mpc,
        integrator,
        yaml_data,
        logger,
        trajectory_generator
    )


if __name__ == '__main__':
    main()
