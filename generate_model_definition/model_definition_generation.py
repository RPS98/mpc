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

import os
import argparse

from generate_model_definition.utils.read_config import YamlConfig, generate_file

# ====================
# CONFIGURATION
# ====================
# Directory containing Jinja2 template
TEMPLATE_DIR = 'generate_model_definition/templates/templ_model_definition.py.j2'


def generate_datatypes(config_path: str, output_dir: str, template_path: str = TEMPLATE_DIR):
    # Ensure output directory exists
    os.makedirs(output_dir, exist_ok=True)

    # Config loading
    config_data = YamlConfig(config_path)

    # Template for generating classes
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


def _parse_args():
    parser = argparse.ArgumentParser(
        description='Generate Python datatypes from a YAML model definition.')
    parser.add_argument('-c', '--config-file',
                        default='generate_model_definition/model_definition.yaml',
                        help='Path to the model_definition YAML file (default: %(default)s)')
    parser.add_argument('-o', '--output-dir',
                        default='mpc',
                        help='Directory for generated Python files in model_definition folder (default: %(default)s)')
    parser.add_argument('-t', '--template-path', default=TEMPLATE_DIR,
                        help='Path to jinja2 template used to generate files (default: %(default)s)')
    return parser.parse_args()


if __name__ == '__main__':
    args = _parse_args()
    config_file = args.config_file
    output_dir = args.output_dir + '/model_definition/'
    generate_datatypes(config_file, output_dir, args.template_path)
    print(
        f'Python datatypes generated successfully into {output_dir!r} from {config_file!r}.')
