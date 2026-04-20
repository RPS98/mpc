#!/usr/bin/env python3

# Copyright 2025 Universidad Politécnica de Madrid
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

"""YAML-based configuration utilities for the MPC library."""

__authors__ = 'Rafael Perez-Segui'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

# THIS CODE HAS BEEN AUTOMATICALLY GENERATED USING templ_acados_mpc_yaml.py.j2

from pathlib import Path
from typing import Any, Callable, Dict, Optional

import numpy as np
import yaml

from mpc_acados_position.model_definition.parameters import OnlineParameters


class MpcYamlConfig:
    """MPC configuration loaded from YAML.

    Optional entries are stored as ``None`` when the key is absent in YAML.
    During configuration, only non-``None`` values are applied to MPC.
    """

    def __init__(self) -> None:
        self.mass: Optional[np.ndarray] = None
        self.desired_position: Optional[np.ndarray] = None
        self.desired_orientation: Optional[np.ndarray] = None
        self.external_force: Optional[np.ndarray] = None
        self.Q: Optional[np.ndarray] = None
        self.Qe: Optional[np.ndarray] = None
        self.R: Optional[np.ndarray] = None

        self.lbu: Optional[np.ndarray] = None
        self.ubu: Optional[np.ndarray] = None

        self.lbx: Optional[np.ndarray] = None
        self.ubx: Optional[np.ndarray] = None
        self.lsbx: Optional[np.ndarray] = None
        self.usbx: Optional[np.ndarray] = None

        self.lh: Optional[np.ndarray] = None
        self.uh: Optional[np.ndarray] = None
        self.lsh: Optional[np.ndarray] = None
        self.ush: Optional[np.ndarray] = None

        self.Zl: Optional[np.ndarray] = None
        self.Zu: Optional[np.ndarray] = None
        self.zl: Optional[np.ndarray] = None
        self.zu: Optional[np.ndarray] = None

        self.Zl_e: Optional[np.ndarray] = None
        self.Zu_e: Optional[np.ndarray] = None
        self.zl_e: Optional[np.ndarray] = None
        self.zu_e: Optional[np.ndarray] = None


def _ensure_dict(node: Any, path: str) -> Dict[str, Any]:
    """Return a dict for YAML mappings, or empty dict for missing nodes."""
    if node is None:
        return {}
    if isinstance(node, dict):
        return node
    raise ValueError(f'{path} must be a mapping in YAML.')


def _to_float_array(value: Any, path: str) -> np.ndarray:
    """Convert a YAML sequence to a 1D float numpy array."""
    if isinstance(value, np.ndarray):
        array = value.astype(float, copy=False)
    elif isinstance(value, (list, tuple)):
        array = np.asarray(value, dtype=float)
    else:
        raise ValueError(f'{path} must be a YAML sequence of numbers.')

    return np.asarray(array, dtype=float).reshape(-1)


def _validate_vector_size(vector: np.ndarray, expected_size: int, path: str) -> None:
    """Validate vector shape against expected solver/model size."""
    if vector.shape != (expected_size,):
        raise ValueError(
            f'{path} must have {expected_size} elements, got {vector.shape[0]}.')


def _yaml_array_required(section: Dict[str, Any], key: str, path: str) -> np.ndarray:
    """Read required YAML array."""
    if key not in section or section[key] is None:
        raise ValueError(f'Missing required YAML entry: {path}')
    return _to_float_array(section[key], path)


def _yaml_array_optional(section: Dict[str, Any], key: str, path: str) -> Optional[np.ndarray]:
    """Read optional YAML array without imposing a size at parse time."""
    if key not in section or section[key] is None:
        return None
    return _to_float_array(section[key], path)


def _yaml_array_optional_sized(
        section: Dict[str, Any],
        key: str,
        expected_size: int,
        path: str) -> Optional[np.ndarray]:
    """Read optional YAML array with fixed-size validation."""
    if key not in section or section[key] is None:
        return None
    array = _to_float_array(section[key], path)
    _validate_vector_size(array, expected_size, path)
    return array


def _apply_optional_vector(
        vector: Optional[np.ndarray],
        expected_size: int,
        path: str,
        setter: Callable[[np.ndarray], None]) -> None:
    """Validate and apply optional vectors to MPC datatypes."""
    if vector is None:
        return

    _validate_vector_size(vector, expected_size, path)
    setter(vector)


def read_mpc_yaml(file_path: str) -> MpcYamlConfig:
    """Read MPC configuration from YAML.

    Expected YAML structure:

    mpc:
      parameters:
        <param_name>: [...]   # optional
      constraints:
        lbu: [...]            # required
        ubu: [...]            # required
        lbx: [...]            # optional
        ubx: [...]            # optional
        ...
    """
    path = Path(file_path)
    if not path.is_file():
        raise ValueError(f'File does not exist: {path.resolve()}')

    with path.open('r', encoding='utf-8') as yaml_file:
        root = yaml.safe_load(yaml_file) or {}

    root_cfg = _ensure_dict(root, 'root')
    mpc_cfg = _ensure_dict(root_cfg.get('mpc'), 'mpc')
    parameters_cfg = _ensure_dict(mpc_cfg.get('parameters'), 'mpc.parameters')
    constraints_cfg = _ensure_dict(mpc_cfg.get('constraints'), 'mpc.constraints')

    config = MpcYamlConfig()
    config.mass = _yaml_array_optional_sized(
        parameters_cfg,
        'mass',
        OnlineParameters.mass_length,
        'mpc.parameters.mass')
    config.desired_position = _yaml_array_optional_sized(
        parameters_cfg,
        'desired_position',
        OnlineParameters.desired_position_length,
        'mpc.parameters.desired_position')
    config.desired_orientation = _yaml_array_optional_sized(
        parameters_cfg,
        'desired_orientation',
        OnlineParameters.desired_orientation_length,
        'mpc.parameters.desired_orientation')
    config.external_force = _yaml_array_optional_sized(
        parameters_cfg,
        'external_force',
        OnlineParameters.external_force_length,
        'mpc.parameters.external_force')
    config.Q = _yaml_array_optional_sized(
        parameters_cfg,
        'Q',
        OnlineParameters.Q_length,
        'mpc.parameters.Q')
    config.Qe = _yaml_array_optional_sized(
        parameters_cfg,
        'Qe',
        OnlineParameters.Qe_length,
        'mpc.parameters.Qe')
    config.R = _yaml_array_optional_sized(
        parameters_cfg,
        'R',
        OnlineParameters.R_length,
        'mpc.parameters.R')

    config.lbu = _yaml_array_required(constraints_cfg, 'lbu', 'mpc.constraints.lbu')
    config.ubu = _yaml_array_required(constraints_cfg, 'ubu', 'mpc.constraints.ubu')

    config.lbx = _yaml_array_optional(constraints_cfg, 'lbx', 'mpc.constraints.lbx')
    config.ubx = _yaml_array_optional(constraints_cfg, 'ubx', 'mpc.constraints.ubx')
    config.lsbx = _yaml_array_optional(constraints_cfg, 'lsbx', 'mpc.constraints.lsbx')
    config.usbx = _yaml_array_optional(constraints_cfg, 'usbx', 'mpc.constraints.usbx')

    config.lh = _yaml_array_optional(constraints_cfg, 'lh', 'mpc.constraints.lh')
    config.uh = _yaml_array_optional(constraints_cfg, 'uh', 'mpc.constraints.uh')
    config.lsh = _yaml_array_optional(constraints_cfg, 'lsh', 'mpc.constraints.lsh')
    config.ush = _yaml_array_optional(constraints_cfg, 'ush', 'mpc.constraints.ush')

    config.Zl = _yaml_array_optional(constraints_cfg, 'Zl', 'mpc.constraints.Zl')
    config.Zu = _yaml_array_optional(constraints_cfg, 'Zu', 'mpc.constraints.Zu')
    config.zl = _yaml_array_optional(constraints_cfg, 'zl', 'mpc.constraints.zl')
    config.zu = _yaml_array_optional(constraints_cfg, 'zu', 'mpc.constraints.zu')

    config.Zl_e = _yaml_array_optional(constraints_cfg, 'Zl_e', 'mpc.constraints.Zl_e')
    config.Zu_e = _yaml_array_optional(constraints_cfg, 'Zu_e', 'mpc.constraints.Zu_e')
    config.zl_e = _yaml_array_optional(constraints_cfg, 'zl_e', 'mpc.constraints.zl_e')
    config.zu_e = _yaml_array_optional(constraints_cfg, 'zu_e', 'mpc.constraints.zu_e')

    return config


def configure_mpc_from_yaml(mpc, file_path: str) -> None:
    """Configure an initialized MPC controller from a YAML file.

    This helper centralizes all runtime MPC configuration that comes from
    ``mpc_config.yaml``:

    1. Parses the YAML file through :func:`read_mpc_yaml`.
    2. Applies online parameters to ``mpc.get_data().parameters``.
    3. Applies configured bounds and slack weights.
    4. Calls all required ``update_*`` methods so values are pushed to acados.

    Required YAML fields:
      - ``mpc.constraints.lbu``
      - ``mpc.constraints.ubu``

    Optional YAML fields:
      - Online parameters:
        ``mass``, ``desired_position``, ``desired_orientation``,
        ``external_force``, ``Q``, ``Qe``, ``R``
      - Constraint/slack entries:
        ``lbx``, ``ubx``, ``lsbx``, ``usbx``, ``lh``, ``uh``,
        ``lsh``, ``ush``, ``Zl``, ``Zu``, ``zl``, ``zu``,
        ``Zl_e``, ``Zu_e``, ``zl_e``, ``zu_e``

    Behavior for optional entries:
      - If omitted, current values already stored inside ``mpc`` are preserved.
      - If provided, vector sizes must match the active solver dimensions.

    Args:
        mpc: Initialized ``mpc_acados_position.MPC`` instance.
        file_path: Path to the YAML configuration file.

    Raises:
        ValueError: If the YAML file does not exist, a required field is missing,
            or any provided vector has an invalid size.

    Note:
        Dynamic references such as desired position and orientation are applied as
        initial values and can be overwritten later by the control loop.
    """
    config = read_mpc_yaml(file_path)

    # Online parameters
    parameters = mpc.get_data().parameters
    if config.mass is not None:
        parameters.set_mass(config.mass)
    if config.desired_position is not None:
        parameters.set_desired_position(config.desired_position)
    if config.desired_orientation is not None:
        parameters.set_desired_orientation(config.desired_orientation)
    if config.external_force is not None:
        parameters.set_external_force(config.external_force)
    if config.Q is not None:
        parameters.set_Q(config.Q)
    if config.Qe is not None:
        parameters.set_Qe(config.Qe)
    if config.R is not None:
        parameters.set_R(config.R)

    # Actuation bounds (required)
    if config.lbu is None or config.ubu is None:
        raise ValueError('Missing required YAML entries: mpc.constraints.lbu and/or mpc.constraints.ubu')

    _validate_vector_size(config.lbu, mpc.idxbu_size, 'mpc.constraints.lbu')
    _validate_vector_size(config.ubu, mpc.idxbu_size, 'mpc.constraints.ubu')
    mpc.get_actuation_bounds().set_lbu(config.lbu)
    mpc.get_actuation_bounds().set_ubu(config.ubu)

    # State bounds
    state_bounds = mpc.get_state_bounds()
    _apply_optional_vector(config.lbx, mpc.idxbx_size, 'mpc.constraints.lbx', state_bounds.set_lbx)
    _apply_optional_vector(config.ubx, mpc.idxbx_size, 'mpc.constraints.ubx', state_bounds.set_ubx)

    # Soft state bounds
    soft_state_bounds = mpc.get_soft_state_bounds()
    _apply_optional_vector(
        config.lsbx,
        mpc.idxsbx_size,
        'mpc.constraints.lsbx',
        soft_state_bounds.set_lsbx)
    _apply_optional_vector(
        config.usbx,
        mpc.idxsbx_size,
        'mpc.constraints.usbx',
        soft_state_bounds.set_usbx)

    # Nonlinear constraint bounds
    nonlinear_bounds = mpc.get_nonlinear_constraint_bounds()
    _apply_optional_vector(config.lh, mpc.lh_size, 'mpc.constraints.lh', nonlinear_bounds.set_lh)
    _apply_optional_vector(config.uh, mpc.lh_size, 'mpc.constraints.uh', nonlinear_bounds.set_uh)

    # Soft nonlinear constraint bounds
    soft_nonlinear_bounds = mpc.get_soft_nonlinear_constraint_bounds()
    _apply_optional_vector(config.lsh, mpc.idxsh_size, 'mpc.constraints.lsh', soft_nonlinear_bounds.set_lsh)
    _apply_optional_vector(config.ush, mpc.idxsh_size, 'mpc.constraints.ush', soft_nonlinear_bounds.set_ush)

    # Slack weights
    slack_weights = mpc.get_slack_weights()
    _apply_optional_vector(config.Zl, mpc.ns_size, 'mpc.constraints.Zl', slack_weights.set_Zl)
    _apply_optional_vector(config.Zu, mpc.ns_size, 'mpc.constraints.Zu', slack_weights.set_Zu)
    _apply_optional_vector(config.zl, mpc.ns_size, 'mpc.constraints.zl', slack_weights.set_zl)
    _apply_optional_vector(config.zu, mpc.ns_size, 'mpc.constraints.zu', slack_weights.set_zu)

    # Terminal slack weights
    slack_weights_end = mpc.get_slack_weights_end()
    _apply_optional_vector(config.Zl_e, mpc.ns_size, 'mpc.constraints.Zl_e', slack_weights_end.set_Zl_e)
    _apply_optional_vector(config.Zu_e, mpc.ns_size, 'mpc.constraints.Zu_e', slack_weights_end.set_Zu_e)
    _apply_optional_vector(config.zl_e, mpc.ns_size, 'mpc.constraints.zl_e', slack_weights_end.set_zl_e)
    _apply_optional_vector(config.zu_e, mpc.ns_size, 'mpc.constraints.zu_e', slack_weights_end.set_zu_e)

    # Push bound and slack changes to the solver
    mpc.update_actuation_bounds()
    if config.lbx is not None or config.ubx is not None:
        mpc.update_state_bounds()
    if config.lsbx is not None or config.usbx is not None:
        mpc.update_soft_state_bounds()
    if config.lh is not None or config.uh is not None:
        mpc.update_nonlinear_constraint_bounds()
    if config.lsh is not None or config.ush is not None:
        mpc.update_soft_nonlinear_constraint_bounds()
    if config.Zl is not None or config.Zu is not None or config.zl is not None or config.zu is not None:
        mpc.update_slack_weights()
    if config.Zl_e is not None or config.Zu_e is not None or config.zl_e is not None or config.zu_e is not None:
        mpc.update_slack_weights_end()


__all__ = ['MpcYamlConfig', 'read_mpc_yaml', 'configure_mpc_from_yaml']

