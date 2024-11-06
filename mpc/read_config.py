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

"""MPC Yaml config reader."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from dataclasses import dataclass
import os

from mpc.mpc_controller_lib.acados_solver import AcadosMPCParams
import numpy as np
import yaml


@dataclass
class YamlMPCParams:
    N_horizon: int = 0  # Prediction steps
    tf: float = 0.0  # Prediction horizon
    mpc_params: AcadosMPCParams = AcadosMPCParams()  # MPC parameters


def read_mpc_params(file_path: str) -> YamlMPCParams:
    """
    Read YAML configuration file and populate YamlData object.

    :param file_path: Path to the YAML file.
    """
    if not os.path.exists(file_path):
        absolute_simulation_config_path = os.path.abspath(file_path)
        print(f'File {absolute_simulation_config_path} does not exist.')
        raise ValueError('File does not exist')

    with open(file_path, 'r') as f:
        config = yaml.safe_load(f)

    data = YamlMPCParams()

    data.N_horizon = config['mpc']['N_horizon']
    data.tf = config['mpc']['tf']
    data.mpc_params.Q = np.diag(np.array(config['mpc']['Q'], dtype=np.float64))
    data.mpc_params.Qe = np.diag(np.array(config['mpc']['Qe'], dtype=np.float64))
    data.mpc_params.R = np.diag(np.array(config['mpc']['R'], dtype=np.float64))
    data.mpc_params.lbu = np.array(config['mpc']['lbu'], dtype=np.float64)
    data.mpc_params.ubu = np.array(config['mpc']['ubu'], dtype=np.float64)
    data.mpc_params.p = np.array(config['mpc']['p'], dtype=np.float64)

    return data


if __name__ == '__main__':
    file_path = 'mpc_config.yaml'
    data = read_mpc_params(file_path)
    print(data)
