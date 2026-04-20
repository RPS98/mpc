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

Reads a ``model_definition.yaml`` living in a controller package
(``controllers/<name>/generate_model_definition/model_definition.yaml``) and
produces:

- Python datatype modules (``state.py``, ``actuation.py``, ``parameters.py``,
  ``dynamics.py``) under ``controllers/<name>/mpc_acados_<name>/``.
- C++ datatype header + source and gtest under
  ``controllers/<name>/include/mpc_acados_<name>/``,
  ``controllers/<name>/src/`` and ``controllers/<name>/tests/``.
- C++ YAML loader header under ``controllers/<name>/include/mpc_acados_<name>/``.
- Python YAML loader module under
  ``controllers/<name>/mpc_acados_<name>/utils/``.
- Optional runtime ``mpc_config.yaml`` under
  ``controllers/<name>/config/``.

All templates receive ``core_package`` and ``controller_package`` in their
Jinja context so generated code imports from the right packages.
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

from mpc_acados_core.generate.utils.read_config import YamlConfig, generate_file


CORE_PACKAGE = 'mpc_acados_core'
TEMPLATES_DIR = Path(__file__).resolve().parent / 'templates'

PY_TEMPLATE = 'templ_model_definition.py.j2'
CPP_HPP_TEMPLATE = 'templ_acados_mpc_datatype.hpp.j2'
CPP_CPP_TEMPLATE = 'templ_acados_mpc_datatype.cpp.j2'
CPP_GTEST_TEMPLATE = 'templ_acados_mpc_gtest.cpp.j2'
MPC_YAML_HPP_TEMPLATE = 'templ_acados_mpc_yaml.hpp.j2'
MPC_YAML_PY_TEMPLATE = 'templ_acados_mpc_yaml.py.j2'
MPC_CONFIG_YAML_TEMPLATE = 'templ_mpc_config.yaml.j2'


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


def _resolve_controller_root(controller: str, repo_root: Path) -> Path:
    """Locate ``controllers/<controller>`` relative to the repo root."""
    candidate = repo_root / 'controllers' / controller
    if not candidate.is_dir():
        raise FileNotFoundError(
            f'Controller directory not found: {candidate}. Expected layout '
            f'controllers/<name>/generate_model_definition/model_definition.yaml.')
    return candidate


def generate_controller(
        controller: str,
        repo_root: Path = None,
        generate_mpc_config: bool = True) -> None:
    """Generate every artifact for ``controllers/<controller>``."""
    if repo_root is None:
        # Assumes this file lives at repo_root/mpc_acados_core/generate/
        repo_root = Path(__file__).resolve().parents[2]

    controller_root = _resolve_controller_root(controller, repo_root)
    controller_package = f'mpc_acados_{controller}'
    config_file = controller_root / 'generate_model_definition' / 'model_definition.yaml'
    if not config_file.is_file():
        raise FileNotFoundError(f'Missing YAML: {config_file}')

    config_data = YamlConfig(str(config_file))

    output_py_dir = controller_root / controller_package
    generate_python_datatypes(
        config_data,
        str(output_py_dir),
        controller_package=controller_package,
    )
    generate_cpp_datatypes(
        config_data,
        str(controller_root),
        controller_package=controller_package,
    )
    generate_cpp_yaml_header(
        config_data,
        str(controller_root),
        controller_package=controller_package,
    )
    generate_python_yaml_module(
        config_data,
        str(controller_root),
        controller_package=controller_package,
    )
    if generate_mpc_config:
        mpc_config_output = controller_root / 'config' / 'mpc_config_template.yaml'
        os.makedirs(mpc_config_output.parent, exist_ok=True)
        generate_mpc_config_yaml(
            config_data,
            str(mpc_config_output),
            controller_package=controller_package,
        )

    cpp_files = _collect_cpp_like_files(str(controller_root / 'include'))
    cpp_files += _collect_cpp_like_files(str(controller_root / 'src'))
    cpp_files += _collect_cpp_like_files(str(controller_root / 'tests'))
    _clang_format_files(cpp_files)

    print(f"[{controller}] Python datatypes -> {output_py_dir}")
    print(f"[{controller}] C++ include      -> {controller_root / 'include' / controller_package}")
    print(f"[{controller}] C++ src          -> {controller_root / 'src'}")
    print(f"[{controller}] C++ tests        -> {controller_root / 'tests'}")
    if generate_mpc_config:
        print(f"[{controller}] MPC config       -> {mpc_config_output}")


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Generate controller-specific Python and C++ MPC datatypes.')
    parser.add_argument('-C', '--controller', required=True,
                        help='Controller name, e.g. "position" or "trajectory". '
                             'Resolves to controllers/<name>/.')
    parser.add_argument('--repo-root', default=None,
                        help='Override repository root. Defaults to the repo that '
                             'contains this mpc_acados_core package.')
    parser.add_argument('--no-mpc-config', action='store_true',
                        help='Skip generation of config/mpc_config_template.yaml.')
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(argv)
    repo_root = Path(args.repo_root).resolve() if args.repo_root else None
    generate_controller(
        args.controller,
        repo_root=repo_root,
        generate_mpc_config=not args.no_mpc_config,
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
