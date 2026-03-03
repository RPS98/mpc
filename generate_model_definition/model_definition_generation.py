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
"""Script for MAV Model definition generation."""

import argparse
import os
import shutil
import subprocess

from generate_model_definition.utils.read_config import YamlConfig, generate_file

# ====================
# CONFIGURATION
# ====================
PY_TEMPLATE_PATH = 'generate_model_definition/templates/templ_model_definition.py.j2'
CPP_HPP_TEMPLATE_PATH = 'generate_model_definition/templates/templ_acados_mpc_datatype.hpp.j2'
CPP_CPP_TEMPLATE_PATH = 'generate_model_definition/templates/templ_acados_mpc_datatype.cpp.j2'
CPP_GTEST_TEMPLATE_PATH = 'generate_model_definition/templates/templ_acados_mpc_gtest.cpp.j2'


def _sanitize_cpp_comment(comment: str) -> str:
    """Sanitize YAML comments for safe inclusion in Doxygen comments."""
    return ' '.join(str(comment).replace('*/', '* /').split())


def generate_python_datatypes(config_data: YamlConfig, output_dir: str,
                              template_path: str = PY_TEMPLATE_PATH):
    """Generate Python datatypes from model definition sections."""
    os.makedirs(output_dir, exist_ok=True)

    for section_name, section in config_data.sections.items():
        context = dict(
            section_name=section_name,
            class_name=section_name.capitalize(),
            size=section.size,
            num_parameters=section.num_parameters,
            parameters=section.as_list(),
        )

        generate_file(
            template_path=template_path,
            output_path=os.path.join(output_dir, f'{section_name.lower()}.py'),
            context=context
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
        output_root_dir: str,
        hpp_template_path: str = CPP_HPP_TEMPLATE_PATH,
        cpp_template_path: str = CPP_CPP_TEMPLATE_PATH,
        gtest_template_path: str = CPP_GTEST_TEMPLATE_PATH):
    """Generate C++ interface files from model definition YAML."""
    state_parameters = _get_cpp_parameters(config_data, 'state')
    actuation_parameters = _get_cpp_parameters(config_data, 'actuation')
    online_parameters = _get_cpp_parameters(config_data, 'parameters')
    context = dict(
        state_parameters=state_parameters,
        actuation_parameters=actuation_parameters,
        online_parameters=online_parameters,
    )

    output_hpp = os.path.join(
        output_root_dir, 'cpp_interface/include/acados_mpc/acados_mpc_datatype.hpp')
    output_cpp = os.path.join(
        output_root_dir, 'cpp_interface/src/acados_mpc_datatype.cpp')
    output_gtest = os.path.join(
        output_root_dir, 'cpp_interface/tests/acados_mpc_gtest.cpp')

    generate_file(
        template_path=hpp_template_path,
        output_path=output_hpp,
        context=context,
        trailing_blank_line=True
    )
    generate_file(
        template_path=cpp_template_path,
        output_path=output_cpp,
        context=context,
        trailing_blank_line=True
    )
    generate_file(
        template_path=gtest_template_path,
        output_path=output_gtest,
        context=context,
        trailing_blank_line=True
    )


def _parse_args():
    parser = argparse.ArgumentParser(
        description='Generate Python and C++ datatypes from a YAML model definition.')
    parser.add_argument('-c', '--config-file',
                        default='generate_model_definition/model_definition.yaml',
                        help='Path to the model_definition YAML file (default: %(default)s)')
    parser.add_argument('-o', '--output-dir',
                        default='mpc',
                        help='Root directory for generated Python and C++ files (default: %(default)s)')
    parser.add_argument('-t', '--python-template-path', default=PY_TEMPLATE_PATH,
                        help='Path to jinja2 template used to generate Python files (default: %(default)s)')
    parser.add_argument('--cpp-hpp-template-path', default=CPP_HPP_TEMPLATE_PATH,
                        help='Path to jinja2 template used to generate C++ header file (default: %(default)s)')
    parser.add_argument('--cpp-cpp-template-path', default=CPP_CPP_TEMPLATE_PATH,
                        help='Path to jinja2 template used to generate C++ source file (default: %(default)s)')
    parser.add_argument('--cpp-gtest-template-path', default=CPP_GTEST_TEMPLATE_PATH,
                        help='Path to jinja2 template used to generate C++ gtest file (default: %(default)s)')
    return parser.parse_args()


if __name__ == '__main__':
    args = _parse_args()
    config_file = args.config_file
    output_root_dir = args.output_dir
    config_data = YamlConfig(config_file)

    output_py_dir = os.path.join(output_root_dir, 'model_definition')
    generate_python_datatypes(config_data, output_py_dir, args.python_template_path)
    generate_cpp_datatypes(
        config_data,
        output_root_dir,
        args.cpp_hpp_template_path,
        args.cpp_cpp_template_path,
        args.cpp_gtest_template_path)
    _clang_format_files(_collect_cpp_like_files(output_root_dir))

    print(f'Python datatypes generated successfully into {output_py_dir!r}.')
    print(
        f"C++ interface files generated successfully into "
        f"{os.path.join(output_root_dir, 'cpp_interface')!r}."
    )
