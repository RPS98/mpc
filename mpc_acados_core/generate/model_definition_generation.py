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

Reads a ``model_definition.yaml`` and produces the library template for a
controller package. Output layout (for ``package_name = mpc_acados_position``)::

    <controller_root>/mpc_acados_position/
    ├── model_definition/
    │   ├── __init__.py            (re-exports every generated datatype)
    │   ├── state.py
    │   ├── actuation.py
    │   ├── parameters.py
    │   └── dynamics.py
    ├── utils/
    │   ├── __init__.py
    │   └── mpc_yaml.py
    └── cpp_interface/
        ├── CMakeLists.txt         (expects sibling mpc_generated_code/)
        ├── include/mpc_acados_position/
        │   ├── acados_mpc.hpp
        │   ├── acados_mpc_datatype.hpp
        │   ├── acados_mpc_yaml.hpp
        │   └── acados_sim_solver.hpp
        ├── src/
        │   ├── acados_mpc.cpp
        │   ├── acados_mpc_datatype.cpp
        │   └── acados_sim_solver.cpp
        └── tests/
            └── acados_mpc_gtest.cpp

The ``cpp_interface/`` directory is a template: it is *not* buildable in
isolation. A consumer project is expected to copy it next to the
acados-generated C code (as ``mpc_interface/mpc_generated_code/``) and do
``add_subdirectory(mpc_interface)``.

The ``model_definition.yaml`` must contain a top-level ``package_name`` field
that names the Python/C++ package to generate (e.g. ``mpc_acados_position``).

Usage
-----
Run from the controller root (``--output-root`` defaults to the directory
containing the config file, i.e. the controller root when the YAML lives at
``<controller_root>/model_definition.yaml``)::

    python3 -m mpc_acados_core.generate.model_definition_generation \\
        --config model_definition.yaml

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
ACADOS_MPC_HPP_TEMPLATE = 'templ_acados_mpc.hpp.j2'
ACADOS_MPC_CPP_TEMPLATE = 'templ_acados_mpc.cpp.j2'
ACADOS_SIM_HPP_TEMPLATE = 'templ_acados_sim_solver.hpp.j2'
ACADOS_SIM_CPP_TEMPLATE = 'templ_acados_sim_solver.cpp.j2'
CPP_INTERFACE_CMAKE_TEMPLATE = 'templ_cpp_interface_cmakelists.txt.j2'

# Datatypes generated under <pkg>/model_definition/. The __init__.py re-exports
# each (class, CaClass) pair plus OnlineParameters, so user code can import
# from <pkg>.model_definition or (via the package __init__) from <pkg> directly.
_MODEL_DEFINITION_EXPORTS = {
    'state': ('State', 'CaState'),
    'actuation': ('Actuation', 'CaActuation'),
    'parameters': ('Parameters', 'CaParameters', 'OnlineParameters'),
    'dynamics': ('Dynamics', 'CaDynamics'),
}


def _sanitize_cpp_comment(comment: str) -> str:
    """Sanitize YAML comments for safe inclusion in Doxygen comments."""
    return ' '.join(str(comment).replace('*/', '* /').split())


def _template_path(name: str) -> str:
    return str(TEMPLATES_DIR / name)


def _cpp_interface_dir(controller_root: str, controller_package: str) -> str:
    return os.path.join(controller_root, controller_package, 'cpp_interface')


def _cpp_include_dir(controller_root: str, controller_package: str) -> str:
    return os.path.join(
        _cpp_interface_dir(controller_root, controller_package),
        'include', controller_package)


def _cpp_src_dir(controller_root: str, controller_package: str) -> str:
    return os.path.join(
        _cpp_interface_dir(controller_root, controller_package), 'src')


def _cpp_tests_dir(controller_root: str, controller_package: str) -> str:
    return os.path.join(
        _cpp_interface_dir(controller_root, controller_package), 'tests')


def generate_python_datatypes(
        config_data: YamlConfig,
        output_dir: str,
        controller_package: str,
        core_package: str = CORE_PACKAGE) -> None:
    """Generate Python datatype modules and an ``__init__.py`` that re-exports them.

    Writes one module per section under ``output_dir`` plus an
    ``__init__.py`` that re-exports every public symbol listed in
    ``_MODEL_DEFINITION_EXPORTS``.
    """
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

    _write_model_definition_init(output_dir, controller_package)


def _write_model_definition_init(output_dir: str, controller_package: str) -> None:
    """Write ``model_definition/__init__.py`` with re-exports of every section."""
    lines = [
        '"""Generated datatypes for {pkg}.'.format(pkg=controller_package),
        '',
        'This sub-package is produced by',
        '``mpc_acados_core.generate.model_definition_generation`` from the',
        'controller\'s ``model_definition.yaml``. Do not edit by hand.',
        '"""',
        '',
        '# THIS FILE HAS BEEN AUTOMATICALLY GENERATED.',
        '',
    ]
    all_symbols = []
    for section_name, symbols in _MODEL_DEFINITION_EXPORTS.items():
        import_list = ', '.join(symbols)
        lines.append(
            f'from {controller_package}.model_definition.{section_name} '
            f'import {import_list}'
        )
        all_symbols.extend(symbols)
    lines.append('')
    lines.append('__all__ = [')
    for symbol in sorted(all_symbols):
        lines.append(f"    '{symbol}',")
    lines.append(']')
    lines.append('')

    with open(os.path.join(output_dir, '__init__.py'), 'w') as f:
        f.write('\n'.join(lines))


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
    """Generate C++ datatype files into <pkg>/cpp_interface/{include,src,tests}."""
    context = dict(
        state_parameters=_get_cpp_parameters(config_data, 'state'),
        actuation_parameters=_get_cpp_parameters(config_data, 'actuation'),
        online_parameters=_get_cpp_parameters(config_data, 'parameters'),
        controller_package=controller_package,
        core_package=core_package,
    )

    include_dir = _cpp_include_dir(controller_root, controller_package)
    src_dir = _cpp_src_dir(controller_root, controller_package)
    tests_dir = _cpp_tests_dir(controller_root, controller_package)

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

    include_dir = _cpp_include_dir(controller_root, controller_package)
    src_dir = _cpp_src_dir(controller_root, controller_package)

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
    """Generate ``<cpp_include>/acados_mpc_yaml.hpp``."""
    context = dict(
        actuation_parameters=_get_cpp_parameters(config_data, 'actuation'),
        online_parameters=_get_cpp_parameters(config_data, 'parameters'),
        controller_package=controller_package,
        core_package=core_package,
    )

    include_dir = _cpp_include_dir(controller_root, controller_package)
    generate_file(
        template_path=_template_path(MPC_YAML_HPP_TEMPLATE),
        output_path=os.path.join(include_dir, 'acados_mpc_yaml.hpp'),
        context=context,
        trailing_blank_line=True,
    )


def generate_cpp_interface_cmakelists(
        controller_root: str,
        controller_package: str,
        core_package: str = CORE_PACKAGE) -> None:
    """Generate ``<pkg>/cpp_interface/CMakeLists.txt``.

    This CMakeLists is a template: it expects the acados C code to be present
    in a sibling directory ``./mpc_generated_code/`` at configure time. It
    errors out otherwise with a message pointing the user at
    ``generate_mpc_interface.sh``.
    """
    context = dict(
        controller_package=controller_package,
        core_package=core_package,
    )
    output_path = os.path.join(
        _cpp_interface_dir(controller_root, controller_package),
        'CMakeLists.txt')
    generate_file(
        template_path=_template_path(CPP_INTERFACE_CMAKE_TEMPLATE),
        output_path=output_path,
        context=context,
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


def generate_controller(
        controller_root: Path,
        package_name: str,
        config_file: Path) -> None:
    """Generate every artifact for a controller.

    Parameters
    ----------
    controller_root:
        Root directory of the controller (output is written here).
    package_name:
        Python/C++ package name, e.g. ``mpc_acados_position``.
    config_file:
        Path to the ``model_definition.yaml``.
    """
    controller_root = Path(controller_root).resolve()
    config_file = Path(config_file).resolve()

    if not config_file.is_file():
        raise FileNotFoundError(f'Config not found: {config_file}')

    config_data = YamlConfig(str(config_file))

    lib_dir = controller_root / package_name
    output_py_dir = lib_dir / 'model_definition'

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
    generate_cpp_interface_cmakelists(
        str(controller_root),
        controller_package=package_name,
    )

    cpp_interface_dir = lib_dir / 'cpp_interface'
    cpp_files = _collect_cpp_like_files(str(cpp_interface_dir))
    _clang_format_files(cpp_files)

    print(f'[{package_name}] Python datatypes -> {output_py_dir}')
    print(f'[{package_name}] Python utils     -> {lib_dir / "utils"}')
    print(f'[{package_name}] C++ include      -> {cpp_interface_dir / "include" / package_name}')
    print(f'[{package_name}] C++ src          -> {cpp_interface_dir / "src"}')
    print(f'[{package_name}] C++ tests        -> {cpp_interface_dir / "tests"}')
    print(f'[{package_name}] C++ CMakeLists   -> {cpp_interface_dir / "CMakeLists.txt"}')


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
             'Defaults to the directory containing --config '
             '(i.e. <controller_root>/model_definition.yaml -> <controller_root>).',
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(argv)
    config_file = Path(args.config).resolve()
    package_name = _read_package_name(config_file)

    if args.output_root is not None:
        controller_root = Path(args.output_root).resolve()
    else:
        # Convention: config lives at <controller_root>/model_definition.yaml
        controller_root = config_file.parent

    generate_controller(
        controller_root=controller_root,
        package_name=package_name,
        config_file=config_file,
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
