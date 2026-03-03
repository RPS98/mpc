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

"""Trajectory-generator specific utils for the MPC example."""

__authors__ = 'Rafael Perez-Segui'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import ctypes
import importlib.util
from pathlib import Path
import sys

import numpy as np


def _first_existing_path(candidates: list[Path]) -> str | None:
    """Return the first existing path from a list of candidates."""
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate.resolve())
    return None


def _resolve_shared_library(current_dir: Path, library_name: str) -> str | None:
    """Resolve a shared library from the example tree or its build outputs."""
    build_dir = current_dir.parents[1] / 'build'

    known_locations = {
        'libnlopt.so.0': [
            current_dir / library_name,
            build_dir / '_deps' / 'nlopt-build' / library_name,
        ],
        'libmav_trajectory_generation.so': [
            current_dir / library_name,
            build_dir / '_deps' / 'dynamic_trajectory_generator-build' /
            'subpackages' / 'mav_trajectory_generation' / library_name,
        ],
        'libdynamic_trajectory_generator.so': [
            current_dir / library_name,
            build_dir / '_deps' / 'dynamic_trajectory_generator-build' / library_name,
        ],
    }

    resolved_path = _first_existing_path(known_locations.get(library_name, []))
    if resolved_path is not None:
        return resolved_path

    if build_dir.is_dir():
        matches = sorted(build_dir.rglob(library_name))
        if matches:
            return str(matches[0].resolve())

    return None


def _preload_library(current_dir: Path, library_name: str) -> None:
    """Load a shared library globally so Python extensions can resolve it."""
    library_path = _resolve_shared_library(current_dir, library_name)
    if library_path is None:
        raise FileNotFoundError(
            f"Required shared library '{library_name}' was not found next to the "
            f"example or under '{current_dir.parents[1] / 'build'}'."
        )
    ctypes.CDLL(library_path, mode=ctypes.RTLD_GLOBAL)


def get_trajectory_generator(initial_position: np.ndarray, waypoints: list, speed: float):
    """
    Initialize and generate a dynamic trajectory.

    :param initial_position: Initial position of the vehicle.
    :param waypoints: List of waypoints.
    :param speed: Speed of the trajectory generator.
    :return: Generated trajectory.
    """
    current_dir = Path(__file__).resolve().parent
    so_file_path = current_dir / 'trajectory_generator.so'

    if not so_file_path.is_file():
        raise FileNotFoundError(f"Shared library not found at path: {so_file_path}")

    for library_name in (
        'libnlopt.so.0',
        'libmav_trajectory_generation.so',
        'libdynamic_trajectory_generator.so',
    ):
        _preload_library(current_dir, library_name)

    current_dir_str = str(current_dir)
    if current_dir_str not in sys.path:
        sys.path.insert(0, current_dir_str)

    spec = importlib.util.spec_from_file_location("trajectory_generator", so_file_path)
    dtb = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(dtb)

    trajectory_generator = dtb.DynamicTrajectory()
    trajectory_generator.set_path_facing(False)
    trajectory_generator.generate_trajectory(
        initial_position,
        0.0,
        waypoints,
        speed)

    max_time = trajectory_generator.get_max_time()
    print(f"Trajectory generated with max time: {max_time}")
    return trajectory_generator
