#!/usr/bin/env python3

# Copyright 2026 Universidad Politécnica de Madrid
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


"""Convert YAML configuration to a dictionary-like object with attribute access."""

import yaml
import numpy as np


class DictWithAttributeAccess:
    """Dictionary-like object that allows attribute access and converts lists to numpy arrays."""

    def __init__(self, data: dict):
        """
        Initialize from dictionary.

        :param data: Dictionary to convert
        :type data: dict
        """
        for key, value in data.items():
            if isinstance(value, dict):
                # Recursively convert nested dictionaries
                setattr(self, key, DictWithAttributeAccess(value))
            elif isinstance(value, list):
                # Convert lists to numpy arrays
                # Detect type: int or float
                if len(value) == 0:
                    setattr(self, key, np.array([], dtype=int))
                elif all(isinstance(x, int) and not isinstance(x, bool) for x in value):
                    setattr(self, key, np.array(value, dtype=int))
                else:
                    setattr(self, key, np.array(value, dtype=float))
            else:
                setattr(self, key, value)

    def __repr__(self):
        """String representation."""
        items = []
        for key, value in self.__dict__.items():
            if isinstance(value, DictWithAttributeAccess):
                items.append(f"{key}=<DictWithAttributeAccess>")
            elif isinstance(value, np.ndarray):
                items.append(f"{key}={value}")
            else:
                items.append(f"{key}={value}")
        return f"DictWithAttributeAccess({', '.join(items)})"

    def __str__(self):
        """Pretty string representation."""
        return self._to_string(indent=0)

    def _to_string(self, indent=0):
        """Recursive string representation with indentation."""
        lines = []
        prefix = "  " * indent
        for key, value in self.__dict__.items():
            if isinstance(value, DictWithAttributeAccess):
                lines.append(f"{prefix}{key}:")
                lines.append(value._to_string(indent + 1))
            elif isinstance(value, np.ndarray):
                lines.append(f"{prefix}{key}: {value} (np.ndarray)")
            else:
                lines.append(f"{prefix}{key}: {value}")
        return "\n".join(lines)

    def get(self, key, default=None):
        """Get attribute with default."""
        return getattr(self, key, default)


def yaml_to_dict(yaml_path: str) -> DictWithAttributeAccess:
    """
    Load YAML file and convert to dictionary with attribute access.

    :param yaml_path: Path to YAML file
    :type yaml_path: str
    :return: Dictionary-like object with attribute access
    :rtype: DictWithAttributeAccess
    """
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    return DictWithAttributeAccess(data)


if __name__ == '__main__':
    # Example usage
    solver_definition = yaml_to_dict('solver_definition.yaml')

    print("=== Solver Definition ===")
    print(solver_definition)
    print("\n" + "=" * 60 + "\n")

    # Test attribute access
    print("=== Testing attribute access ===")
    print(f"N_horizon: {solver_definition.mpc.N_horizon}")
    print(f"tf: {solver_definition.mpc.tf}")
    print(f"Q: {solver_definition.mpc.Q}")
    print(f"Type of Q: {type(solver_definition.mpc.Q)}")
    print(f"Q is numpy array: {isinstance(solver_definition.mpc.Q, np.ndarray)}")
    print(f"\nR: {solver_definition.mpc.R}")
    print(f"Type of R: {type(solver_definition.mpc.R)}")
    print(f"\nexport_dir: {solver_definition.solver.export_dir}")
    print(f"integrator_type: {solver_definition.integrator.integrator_type}")
