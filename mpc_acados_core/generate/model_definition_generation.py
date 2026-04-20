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
"""Generator for controller-specific Python and C++ MPC datatypes.

Reads a ``model_definition.yaml`` and produces:

- Python datatype modules (``state.py``, ``actuation.py``, ``parameters.py``,
  ``dynamics.py``) under ``<output_root>/<package_name>/``.
- C++ datatype header + source and gtest under
  ``<output_root>/include/<package_name>/``,
  ``<output_root>/src/`` and ``<output_root>/tests/``.
- C++ YAML loader header under ``<output_root>/include/<package_name>/``.
- Python YAML loader module under ``<output_root>/<package_name>/utils/``.
- Optional runtime ``mpc_config.yaml`` under ``<output_root>/config/``.

The ``model_definition.yaml`` must contain a top-level ``package_name`` field
that names the Python/C++ package to generate (e.g. ``mpc_acados_position``).

Usage
-----
Run from the controller root (``--output-root`` defaults to the directory two
levels above the config file, i.e. the controller root when the YAML lives in
``generate_model_definition/model_definition.yaml``)::

    python3 -m mpc_acados_core.generate.model_definition_generation \\
        --config generate_model_definition/model_definition.yaml

From an arbitrary location::

    python3 -m mpc_acados_core.generate.model_definition_generation \\
        --config /path/to/model_definition.yaml \\
        --output-root /path/to/controller_root
"""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

import yaml

from mpc_acados_core.generate.utils.read_config import YamlConfig, generate_file


CORE_PACKAGE = 'mpc_acados_core'
DEFAULT_CPP_NAMESPACE = 'acados_mpc'
DEFAULT_MODEL_NAME = 'mpc'
TEMPLATES_DIR = Path(__file__).resolve().parent / 'templates'

PY_TEMPLATE = 'templ_model_definition.py.j2'
CPP_HPP_TEMPLATE = 'templ_acados_mpc_datatype.hpp.j2'
CPP_CPP_TEMPLATE = 'templ_acados_mpc_datatype.cpp.j2'
CPP_GTEST_TEMPLATE = 'templ_acados_mpc_gtest.cpp.j2'
MPC_YAML_HPP_TEMPLATE = 'templ_acados_mpc_yaml.hpp.j2'
MPC_YAML_PY_TEMPLATE = 'templ_acados_mpc_yaml.py.j2'
MPC_CONFIG_YAML_TEMPLATE = 'templ_mpc_config.yaml.j2'
ACADOS_MPC_HPP_TEMPLATE = 'templ_acados_mpc.hpp.j2'
ACADOS_MPC_CPP_TEMPLATE = 'templ_acados_mpc.cpp.j2'
ACADOS_SIM_HPP_TEMPLATE = 'templ_acados_sim_solver.hpp.j2'
ACADOS_SIM_CPP_TEMPLATE = 'templ_acados_sim_solver.cpp.j2'


def _sanitize_cpp_comment(comment: str) -> str:
    """Sanitize YAML comments for safe inclusion in Doxygen comments."""
    return ' '.join(str(comment).replace('*/', '* /').split())


def _template_path(name: str) -> str:
    return str(TEMPLATES_DIR / name)


def generate_python_datatypes(
        config_data: YamlConfig,
        output_dir: str,
        controller_package: str,
        core_package: str = CORE_PACKAGE) -> None:
    """Generate Python datatypes from model definition sections."""
    os.makedirs(output_dir, exist_ok=True)

    for section_name, section in config_data.sections.items():
        context = dict(
            section_name=section_name,
            class_name=section_name.capitalize(),
            size=section.size,
            num_parameters=section.num_parameters,
            parameters=section.as_list(),
            controller_package=controller_package,
            core_package=core_package,
        )
        generate_file(
            template_path=_template_path(PY_TEMPLATE),
            output_path=os.path.join(output_dir, f'{section_name.lower()}.py'),
            context=context,
        )


def _get_cpp_parameters(config_data: YamlConfig, section_name: str):
    """Get parameters for a required YAML section used by C++ generation."""
    if section_name not in config_data.sections:
        raise ValueError(
            f'Missing required section {section_name!r} in {config_data.filepath!r}.')

    section = config_data.sections[section_name]
    parameters = []
    for param in section.as_list():
        param['cpp_description'] = _sanitize_cpp_comment(param['description'])
        parameters.append(param)
    return parameters


def _collect_cpp_like_files(root_dir: str):
    """Collect C/C++ sources and headers recursively from root_dir."""
    extensions = ('.cpp', '.hpp', '.c', '.h')
    files = []
    for dirpath, _, filenames in os.walk(root_dir):
        for filename in filenames:
            if filename.endswith(extensions):
                files.append(os.path.join(dirpath, filename))
    return files


def _clang_format_files(file_paths):
    """Run clang-format over file_paths when clang-format is available."""
    clang_format = shutil.which('clang-format')
    if clang_format is None:
        print('Warning: clang-format not found. Skipping C/C++ auto-format.')
        return
    if not file_paths:
        return
    chunk_size = 200
    for idx in range(0, len(file_paths), chunk_size):
        chunk = file_paths[idx:idx + chunk_size]
        subprocess.run([clang_format, '-i', '-style=file', *chunk], check=True)


def generate_cpp_datatypes(
        config_data: YamlConfig,
        controller_root: str,
        controller_package: str,
        core_package: str = CORE_PACKAGE) -> None:
    """Generate C++ interface files into controller/include, src, tests."""
    context = dict(
        state_parameters=_get_cpp_parameters(config_data, 'state'),
        actuation_parameters=_get_cpp_parameters(config_data, 'actuation'),
        online_parameters=_get_cpp_parameters(config_data, 'parameters'),
        controller_package=controller_package,
        core_package=core_package,
    )

    include_dir = os.path.join(controller_root, 'include', controller_package)
    src_dir = os.path.join(controller_root, 'src')
    tests_dir = os.path.join(controller_root, 'tests')

    generate_file(
        template_path=_template_path(CPP_HPP_TEMPLATE),
        output_path=os.path.join(include_dir, 'acados_mpc_datatype.hpp'),
        context=context,
        trailing_blank_line=True,
    )
    generate_file(
        template_path=_template_path(CPP_CPP_TEMPLATE),
        output_path=os.path.join(src_dir, 'acados_mpc_datatype.cpp'),
        context=context,
        trailing_blank_line=True,
    )
    generate_file(
        template_path=_template_path(CPP_GTEST_TEMPLATE),
        output_path=os.path.join(tests_dir, 'acados_mpc_gtest.cpp'),
        context=context,
        trailing_blank_line=True,
    )


def generate_cpp_wrappers(
        controller_root: str,
        controller_package: str,
        cpp_namespace: str = DEFAULT_CPP_NAMESPACE,
        model_name: str = DEFAULT_MODEL_NAME,
        core_package: str = CORE_PACKAGE) -> None:
    """Generate the variant-specific C++ wrappers around the acados solver."""
    context = dict(
        controller_package=controller_package,
        cpp_namespace=cpp_namespace,
        model_name=model_name,
        core_package=core_package,
    )

    include_dir = os.path.join(controller_root, 'include', controller_package)
    src_dir = os.path.join(controller_root, 'src')

    generate_file(
        template_path=_template_path(ACADOS_MPC_HPP_TEMPLATE),
        output_path=os.path.join(include_dir, 'acados_mpc.hpp'),
        context=context,
        trailing_blank_line=True,
    )
    generate_file(
        template_path=_template_path(ACADOS_MPC_CPP_TEMPLATE),
        output_path=os.path.join(src_dir, 'acados_mpc.cpp'),
        context=context,
        trailing_blank_line=True,
    )
    generate_file(
        template_path=_template_path(ACADOS_SIM_HPP_TEMPLATE),
        output_path=os.path.join(include_dir, 'acados_sim_solver.hpp'),
        context=context,
        trailing_blank_line=True,
    )
    generate_file(
        template_path=_template_path(ACADOS_SIM_CPP_TEMPLATE),
        output_path=os.path.join(src_dir, 'acados_sim_solver.cpp'),
        context=context,
        trailing_blank_line=True,
    )


def generate_cpp_yaml_header(
        config_data: YamlConfig,
        controller_root: str,
        controller_package: str,
        core_package: str = CORE_PACKAGE) -> None:
    """Generate ``<controller_include>/acados_mpc_yaml.hpp``."""
    context = dict(
        actuation_parameters=_get_cpp_parameters(config_data, 'actuation'),
        online_parameters=_get_cpp_parameters(config_data, 'parameters'),
        controller_package=controller_package,
        core_package=core_package,
    )

    include_dir = os.path.join(controller_root, 'include', controller_package)
    generate_file(
        template_path=_template_path(MPC_YAML_HPP_TEMPLATE),
        output_path=os.path.join(include_dir, 'acados_mpc_yaml.hpp'),
        context=context,
        trailing_blank_line=True,
    )


def generate_python_yaml_module(
        config_data: YamlConfig,
        controller_root: str,
        controller_package: str,
        core_package: str = CORE_PACKAGE) -> None:
    """Generate ``<controller_package>/utils/mpc_yaml.py``."""
    context = dict(
        online_parameters=_get_cpp_parameters(config_data, 'parameters'),
        controller_package=controller_package,
        core_package=core_package,
    )

    utils_dir = os.path.join(controller_root, controller_package, 'utils')
    os.makedirs(utils_dir, exist_ok=True)
    init_path = os.path.join(utils_dir, '__init__.py')
    if not os.path.exists(init_path):
        with open(init_path, 'w') as f:
            f.write(f'"""Utility modules for {controller_package}."""\n')

    generate_file(
        template_path=_template_path(MPC_YAML_PY_TEMPLATE),
        output_path=os.path.join(utils_dir, 'mpc_yaml.py'),
        context=context,
        trailing_blank_line=True,
    )


def generate_mpc_config_yaml(
        config_data: YamlConfig,
        output_path: str,
        controller_package: str,
        core_package: str = CORE_PACKAGE) -> None:
    """Generate a starter ``mpc_config.yaml`` with correct array sizes."""
    actuation_parameters = _get_cpp_parameters(config_data, 'actuation')
    online_parameters = _get_cpp_parameters(config_data, 'parameters')
    state_parameters = _get_cpp_parameters(config_data, 'state')

    total_nu = sum(p['size'] for p in actuation_parameters)
    total_nx = sum(p['size'] for p in state_parameters)
    actuation_names = ', '.join(p['name'] for p in actuation_parameters)

    context = dict(
        online_parameters=online_parameters,
        actuation_parameters=actuation_parameters,
        total_nu=total_nu,
        total_nx=total_nx,
        actuation_names=actuation_names,
        controller_package=controller_package,
        core_package=core_package,
    )
    generate_file(
        template_path=_template_path(MPC_CONFIG_YAML_TEMPLATE),
        output_path=output_path,
        context=context,
    )


def generate_controller(
        controller_root: Path,
        package_name: str,
        config_file: Path,
        generate_mpc_config: bool = True) -> None:
    """Generate every artifact for a controller.

    Parameters
    ----------
    controller_root:
        Root directory of the controller (output is written here).
    package_name:
        Python/C++ package name, e.g. ``mpc_acados_position``.
    config_file:
        Path to the ``model_definition.yaml``.
    generate_mpc_config:
        Whether to generate ``config/mpc_config_template.yaml``.
    """
    controller_root = Path(controller_root).resolve()
    config_file = Path(config_file).resolve()

    if not config_file.is_file():
        raise FileNotFoundError(f'Config not found: {config_file}')

    config_data = YamlConfig(str(config_file))

    output_py_dir = controller_root / package_name
    generate_python_datatypes(
        config_data,
        str(output_py_dir),
        controller_package=package_name,
    )
    generate_cpp_datatypes(
        config_data,
        str(controller_root),
        controller_package=package_name,
    )
    generate_cpp_yaml_header(
        config_data,
        str(controller_root),
        controller_package=package_name,
    )
    generate_cpp_wrappers(
        str(controller_root),
        controller_package=package_name,
    )
    generate_python_yaml_module(
        config_data,
        str(controller_root),
        controller_package=package_name,
    )
    if generate_mpc_config:
        mpc_config_output = controller_root / 'config' / 'mpc_config_template.yaml'
        os.makedirs(mpc_config_output.parent, exist_ok=True)
        generate_mpc_config_yaml(
            config_data,
            str(mpc_config_output),
            controller_package=package_name,
        )

    cpp_files = _collect_cpp_like_files(str(controller_root / 'include'))
    cpp_files += _collect_cpp_like_files(str(controller_root / 'src'))
    cpp_files += _collect_cpp_like_files(str(controller_root / 'tests'))
    _clang_format_files(cpp_files)

    print(f'[{package_name}] Python datatypes -> {output_py_dir}')
    print(f'[{package_name}] C++ include      -> {controller_root / "include" / package_name}')
    print(f'[{package_name}] C++ src          -> {controller_root / "src"}')
    print(f'[{package_name}] C++ tests        -> {controller_root / "tests"}')
    if generate_mpc_config:
        print(f'[{package_name}] MPC config       -> {controller_root / "config" / "mpc_config_template.yaml"}')


def _read_package_name(config_file: Path) -> str:
    """Read ``package_name`` from the model_definition YAML."""
    with open(config_file, 'r') as f:
        data = yaml.safe_load(f)
    name = data.get('package_name', '').strip()
    if not name:
        raise ValueError(
            f"'package_name' field is missing or empty in {config_file}. "
            "Add 'package_name: <your_package>' at the top of the file.")
    return name


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Generate controller-specific Python and C++ MPC datatypes.')
    parser.add_argument(
        '-c', '--config',
        required=True,
        metavar='PATH',
        help='Path to model_definition.yaml (must contain a package_name field).',
    )
    parser.add_argument(
        '-o', '--output-root',
        default=None,
        metavar='PATH',
        help='Controller root directory where output is written. '
             'Defaults to the directory two levels above --config '
             '(i.e. <controller_root>/generate_model_definition/model_definition.yaml '
             '-> <controller_root>).',
    )
    parser.add_argument(
        '--no-mpc-config',
        action='store_true',
        help='Skip generation of config/mpc_config_template.yaml.',
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(argv)
    config_file = Path(args.config).resolve()
    package_name = _read_package_name(config_file)

    if args.output_root is not None:
        controller_root = Path(args.output_root).resolve()
    else:
        # Convention: config lives at <controller_root>/generate_model_definition/model_definition.yaml
        controller_root = config_file.parent.parent

    generate_controller(
        controller_root=controller_root,
        package_name=package_name,
        config_file=config_file,
        generate_mpc_config=not args.no_mpc_config,
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
