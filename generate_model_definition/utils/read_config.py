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
"""Utility functions for MAV Model code generation."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import os

import yaml
from jinja2 import Environment, FileSystemLoader


def snake_to_camel(snake_str: str) -> str:
    """Convert snake_case string to camelCase."""
    parts = snake_str.split('_')
    return parts[0].lower() + ''.join(word.capitalize() for word in parts[1:])

def snake_to_pascal(snake_str: str) -> str:
    """Convert snake_case string to PascalCase."""
    camel = snake_to_camel(snake_str)
    return camel[0].upper() + camel[1:]

class Parameter:
    """Represents a single parameter with value and description."""

    def __init__(self, name, value, description, param_type, size, offset):
        self.name = name
        self.value = value
        self.description = description
        self.param_type = param_type
        self.size = size
        self.offset = offset

    def __repr__(self):
        return (f'Parameter(name={self.name}, value={self.value}, '
                f'description={self.description}, type={self.param_type}, '
                f'size={self.size}, offset={self.offset})')


class YamlSection:
    """Represents a section in the YAML file, containing multiple parameters."""

    def __init__(self, name):
        self.name = name
        self.parameters = {}
        self.size = 0  # Total size (flattened vector size)
        self.num_parameters = 0  # Number of parameters
        self.param_sizes = []  # List of sizes for each parameter
        self.current_offset = 0  # Track offset when adding parameters

    def add_parameter(self, name, value, description, param_type):
        param_size = self._determine_size(value)
        self.parameters[name] = Parameter(
            name, value, description, param_type, param_size, self.current_offset)
        self.size += param_size
        self.num_parameters += 1
        self.param_sizes.append(param_size)
        self.current_offset += param_size

    def _determine_size(self, value):
        """Determine size contribution: 1 if scalar, len(value) if list."""
        if isinstance(value, list):
            return len(value)
        return 1

    def as_list(self):
        """Export parameters as list of dicts for Jinja2 templates."""
        return [
            {
                "name": p.name,
                "value": p.value,
                "type": p.param_type,
                "size": p.size,
                "description": p.description,
                "offset": p.offset
            }
            for p in self.parameters.values()
        ]

    def __getitem__(self, key):
        return self.parameters[key]

    def __iter__(self):
        return iter(self.parameters.items())

    def keys(self):
        return self.parameters.keys()

    def values(self):
        return self.parameters.values()

    def items(self):
        return self.parameters.items()


class YamlConfig:
    """Main class to load and parse the YAML file with comments."""

    def __init__(self, filepath, use_camel_case=True):
        self.filepath = filepath
        self.sections = {}
        self.use_camel_case = use_camel_case

        # Load YAML keys and values (ignoring comments initially)
        with open(filepath, 'r') as f:
            self.yaml_data = yaml.safe_load(f)

        # Parse with comments line by line
        self._parse_with_comments()

    def _parse_with_comments(self):
        with open(self.filepath, 'r') as f:
            lines = f.readlines()

        current_section = None
        indent_level = None

        for line in lines:
            stripped = line.strip()

            # Detect new section
            if stripped and not stripped.startswith('#') and ':' in stripped and not stripped.startswith('-'):
                if not line.startswith(' '):
                    section_name = stripped.split(':')[0]
                    if section_name in self.yaml_data:
                        current_section = YamlSection(section_name)
                        self.sections[section_name] = current_section
                        indent_level = len(line) - len(line.lstrip())
                    continue

                # Parse parameter inside section
                if current_section and len(line) - len(line.lstrip()) > indent_level:
                    if ':' in line:
                        key_part, rest = line.split(':', 1)
                        param_name = key_part.strip()

                        if '#' in rest:
                            val_part, comment = rest.split('#', 1)
                            description = comment.strip()
                        else:
                            val_part = rest
                            description = 'No description provided'

                        val_str = val_part.strip()
                        value, param_type = self._convert_value(val_str)
                        current_section.add_parameter(param_name, value, description, param_type)

    @staticmethod
    def _convert_value(val_str):
        """
        Try to convert a YAML string to int, float, list, or keep as str.
        Also return the inferred type as a string.
        """
        if val_str.startswith('[') and val_str.endswith(']'):
            list_values = [YamlConfig._convert_value(x.strip())[0]
                           for x in val_str[1:-1].split(',')]
            return list_values, 'list'
        try:
            int_val = int(val_str)
            return int_val, 'int'
        except ValueError:
            try:
                float_val = float(val_str)
                return float_val, 'float'
            except ValueError:
                return val_str, 'str'


def generate_file(template_path, output_path, context):
    env = Environment(loader=FileSystemLoader(os.path.dirname(template_path)))
    env.filters['camelCase'] = snake_to_camel
    env.filters['pascalCase'] = snake_to_pascal

    template = env.get_template(os.path.basename(template_path))
    context['template_file'] = os.path.basename(template_path)
    rendered = template.render(**context)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w') as f:
        f.write(rendered)

    print(f'Generated {output_path}')


def build_flat_expected_data(section):
    flat_data = []
    for param_name, param in section.items():
        if param.size == 1:
            flat_data.append(1.0)
        else:
            flat_data.extend([float(i + 1) for i in range(param.size)])
    return flat_data


if __name__ == '__main__':
    config = YamlConfig('config.yaml')

    for section_name, section in config.sections.items():
        print(f"{section_name}:")
        for param_name, param in section.items():
            print(f"  {param_name}: {param.value} # {param.description}")
            print(f"  Type: {param.param_type}")
            print(f"  Size: {param.size}")
        print(f"  Total size: {section.size}")
        print(f"  Number of parameters: {section.num_parameters}")
        print(f"  Parameter sizes: {section.param_sizes}")
