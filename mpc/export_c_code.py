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

"""MPC Controller C Code exporter."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'


import argparse
import numpy as np
from mpc.mpc_controller import MPC, MPCParams, mpc_lib


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='MPC code exporter.')
    parser.add_argument('--export_dir', '-d', type=str, default='mpc_generated_code',
                        help='Export directory. Default: mpc_generated_code')

    args = parser.parse_args()
    export_dir = args.export_dir

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

    mpc = MPC(mpc_params, export_dir)
    Tf = mpc_params.prediction_horizon
    N = mpc_params.prediction_steps
    dt = Tf / N
    integrator = mpc.export_integrador(dt)