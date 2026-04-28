# mpc_acados_core

Controller MPC library built on [acados](https://github.com/acados/acados) for quadrotor aerial vehicles.

Controller variants that depend on this core:

- [controllers/position/README.md](controllers/position/README.md) — position-tracking MPC
- [controllers/trajectory/README.md](controllers/trajectory/README.md) — trajectory-tracking MPC

---

## Repository layout

```
mpc/
├── CMakeLists.txt                 # Root: C++ INTERFACE lib + pure-Python mirror
├── pyproject.toml                 # "mpc_acados_core" pip distribution
├── mpc_acados_core/
│   ├── acados_solver.py           # AcadosMPCSolverBase
│   ├── mpc_controller.py          # MPCBase / MPCDataBase
│   ├── drone_model.py
│   ├── mpc_datatype.py            # Reference, Gains, Bounds (generic)
│   ├── logging/                   # CsvLogger (Python) + helpers
│   ├── plotting/                  # plot_results.py
│   ├── utils/                     # quaternion, yaml, solver_config helpers
│   ├── generate/                  # code generator
│   │   ├── model_definition_generation.py
│   │   └── templates/             # .j2 templates
│   └── cpp_interface/
│       ├── CMakeLists.txt         # INTERFACE target
│       ├── cmake/compile_acados.cmake
│       └── include/mpc_acados_core/logging/csv_logger.hpp
└── controllers/
    ├── position/                  # → pip package "mpc_acados_position"
    └── trajectory/                # → pip package "mpc_acados_trajectory"
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

The three packages (`mpc_acados_core`, `mpc_acados_position`,
`mpc_acados_trajectory`) are pure-Python. To make them importable:

```bash
# Option A: pip install (editable)
pip install -e /path/to/mpc
pip install -e /path/to/mpc/controllers/position
pip install -e /path/to/mpc/controllers/trajectory

# Option B: export PYTHONPATH (no pip)
export PYTHONPATH=/path/to/mpc:/path/to/mpc/controllers/position:/path/to/mpc/controllers/trajectory:$PYTHONPATH
```

**For quick shell use**, add this to your `.bashrc` or set it before running CMake:

```bash
MPC_ROOT="/path/to/mpc"
export PYTHONPATH="${MPC_ROOT}:$PYTHONPATH"
export PYTHONPATH="${MPC_ROOT}/controllers/position:$PYTHONPATH"
export PYTHONPATH="${MPC_ROOT}/controllers/trajectory:$PYTHONPATH"
```

When this submodule is driven from a parent CMake project (e.g.
[mpc_examples](https://github.com/RPS98/mpc_examples)) that defines
`PYBIND_PY_MIRROR_ROOT`, the root `CMakeLists.txt` automatically creates
symbolic links from that directory to the three packages, so a single
`PYTHONPATH` entry (`build/python/`) exposes all of them with no `pip install`
step.

---

## Build core C++ only

```bash
cmake -S /path/to/mpc -B build
cmake --build build
```

This builds only the `mpc_acados_core` INTERFACE target. Each controller is an
independent CMake project — see the controller-specific README for build
instructions.

---

## Code generators

### 1. Datatype / template generator (`model_definition.yaml` → library)

Reads a `model_definition.yaml` (which must contain a `package_name` field)
and writes Python modules, C++ headers/sources, and an optional
`mpc_config_template.yaml`.

```bash
python3 -m mpc_acados_core.generate.model_definition_generation \
  --config /path/to/model_definition.yaml

# With explicit output root (defaults to two levels above the config file):
python3 -m mpc_acados_core.generate.model_definition_generation \
  --config /path/to/model_definition.yaml \
  --output-root /path/to/controller_root
```

### 2. `mpc_interface/` generator (`solver_definition_*.yaml` → consumer subproject)

Each controller package ships a tiny CLI that, given a solver-definition
YAML, produces an `mpc_interface/` folder ready for `add_subdirectory(...)`.
The CLI works from any directory as long as `mpc_acados_core` and the
controller package are importable.

```bash
# Position
python3 -m mpc_acados_position.acados_solver \
  -c /path/to/solver_definition_mpc_position.yaml \
  [-o /path/to/output_dir]

# Trajectory
python3 -m mpc_acados_trajectory.acados_solver \
  -c /path/to/solver_definition_mpc_trajectory.yaml \
  [-o /path/to/output_dir]
```

Without `-o`, the output is written next to the YAML file. The layout
produced is:

```
<output_dir>/mpc_interface/
├── CMakeLists.txt
├── include/, src/, tests/            (C++ template, copied from the package)
└── mpc_generated_code/mpc_generated_code/
    ├── libacados_ocp_solver_mpc.so
    └── libacados_sim_solver_mpc.so
```

The shared logic lives in
`mpc_acados_core.generate.mpc_interface_generation` (function
`generate_mpc_interface(...)` + `run_cli(...)`), so both controllers reuse
the same implementation.

---

## License

BSD-3-Clause. See [LICENSE](LICENSE).
