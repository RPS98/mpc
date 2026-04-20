# mpc_acados_core

Controller MPC library built on [acados](https://github.com/acados/acados) for quadrotor aerial vehicles.

Controller variants that depend on this core:

- [controllers/position/README.md](controllers/position/README.md) — position-tracking MPC
- [controllers/trajectory/README.md](controllers/trajectory/README.md) — trajectory-tracking MPC

---

## Repository layout

```
mpc_acados/
├── pyproject.toml                 # "mpc_acados_core" pip distribution
├── mpc_acados_core/
│   ├── acados_solver.py           # AcadosMPCSolverBase
│   ├── mpc_controller.py          # MPCBase / MPCDataBase
│   ├── drone_model.py
│   ├── mpc_datatype.py            # Reference, Gains, Bounds (generic)
│   ├── logging/                   # CsvLogger (Python) + helpers
│   ├── plotting/                  # plot_results.py
│   ├── utils/                     # quaternion, yaml, solver_config helpers
│   └── generate/                  # code generator
│       ├── model_definition_generation.py
│       └── templates/             # .j2 templates
└── mpc_acados_core/cpp_interface/
    ├── CMakeLists.txt             # INTERFACE target
    ├── cmake/compile_acados.cmake
    └── include/mpc_acados_core/logging/csv_logger.hpp
```

---

## Dependencies

- Python 3.10+
- [acados](https://github.com/acados/acados) built with `ACADOS_SOURCE_DIR` set
- CMake 3.16+, C++17 compiler
- Eigen3, yaml-cpp
- Python packages: `numpy`, `casadi`, `acados_template`, `pyyaml`, `jinja2`,
  `matplotlib`, `tqdm`

---

## Install (Python)

```bash
pip install -e /path/to/mpc_acados
# or add to PYTHONPATH:
export PYTHONPATH=/path/to/mpc_acados:$PYTHONPATH
```

---

## Build core C++ only

```bash
cmake -S /path/to/mpc_acados -B build
cmake --build build
```

This builds only the `mpc_acados_core` INTERFACE target. Each controller is an
independent CMake project — see the controller-specific README for build
instructions.

---

## Code generator

The generator reads a `model_definition.yaml` (which must contain a
`package_name` field) and writes Python modules, C++ headers/sources,
and an optional `mpc_config_template.yaml`.

```bash
python3 -m mpc_acados_core.generate.model_definition_generation \
  --config /path/to/model_definition.yaml

# With explicit output root (defaults to two levels above the config file):
python3 -m mpc_acados_core.generate.model_definition_generation \
  --config /path/to/model_definition.yaml \
  --output-root /path/to/controller_root
```

---

## License

BSD-3-Clause. See [LICENSE](LICENSE).
