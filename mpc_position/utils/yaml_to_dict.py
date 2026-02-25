#!/usr/bin/env python3

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
