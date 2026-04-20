# mpc_acados

A generic MPC library built on top of [acados](https://github.com/acados/acados)
for quadrotor-style vehicles. The library is split into a controller-agnostic
**core** and one or more **controller variants** that plug their specific
types (state / actuation / online parameters) into the core.

Two variants ship with this repository out of the box:

| Variant      | Pip package              | C++ library              | Online parameters |
| ------------ | ------------------------ | ------------------------ | ----------------- |
| `position`   | `mpc_acados_position`    | `libmpc_acados_position` | 7 (`desired_position[3]`, `desired_orientation[4]`) |
| `trajectory` | `mpc_acados_trajectory`  | `libmpc_acados_trajectory` | 9 (adds `desired_velocity[3]`, `desired_acceleration[3]`) |

Both share the same core, the same code generator, the same C++ logger /
Python plotter, and the same example pattern.

## Repository layout

```
mpc_acados/
├── CMakeLists.txt                 # top-level build with per-variant toggles
├── pyproject.toml                 # "mpc_acados_core" pip distribution
├── mpc_acados_core/               # Python + C++ core (nothing generated lives here)
│   ├── acados_solver.py           # base AcadosMPCSolver
│   ├── mpc_controller.py          # base MPC / MPCData
│   ├── drone_model.py
│   ├── mpc_datatype.py            # Reference, Gains, Bounds (generic)
│   ├── logging/                   # CsvLogger (Python) + helpers
│   ├── plotting/                  # plot_results.py (metrics + figures)
│   ├── utils/                     # quaternion, yaml, solver_config helpers
│   ├── generate/                  # code generator (Jinja2 templates)
│   │   ├── model_definition_generation.py
│   │   └── templates/             # .j2 templates used for every variant
│   └── cpp_interface/             # core C++ library (INTERFACE target)
│       ├── CMakeLists.txt
│       ├── cmake/compile_acados.cmake
│       └── include/mpc_acados_core/logging/csv_logger.hpp
│
└── controllers/
    ├── position/                  # controller variant: position tracking
    │   ├── CMakeLists.txt         # produces libmpc_acados_position.so
    │   ├── pyproject.toml         # "mpc_acados_position" pip distribution
    │   ├── generate_model_definition/model_definition.yaml  # input to core generator
    │   ├── config/                # runtime config templates
    │   ├── include/mpc_acados_position/   # C++ headers (generated + wrapper)
    │   ├── src/                   # C++ sources (generated + wrapper)
    │   ├── mpc_acados_position/   # Python package (generated)
    │   ├── examples/              # run_example.py, run_example.cpp, yaml configs
    │   ├── tests/                 # unit tests
    │   └── generate_acados_c_code.py  # small entry point for codegen
    └── trajectory/                # same structure; 9-parameter variant
```

### Inputs vs outputs

Only the following files are **written by hand** inside a controller:

- `generate_model_definition/model_definition.yaml` — describes states, actuations
  and online parameters.
- `config/*.yaml`, `examples/*` — runtime configuration and example runners.
- `CMakeLists.txt`, `pyproject.toml`, `README.md`, `generate_acados_c_code.py`.

Everything else (`include/<package>/`, `src/`, `mpc_acados_<name>/*.py`) is
produced by `mpc_acados_core/generate/` from the `.yaml` above. Regenerate by
running `./generate.sh <controller>` (see below).

## Dependencies

Runtime:
- Python 3.10+
- [acados](https://github.com/acados/acados) (built; `ACADOS_SOURCE_DIR` exported)
- CMake 3.16+, a C++17 compiler
- Eigen3, yaml-cpp

Python packages: `numpy`, `casadi`, `acados_template`, `pyyaml`, `jinja2`,
`matplotlib`, `tqdm`.

## Install (Python)

Editable installs of the three distributions:

```bash
# from the repository root
pip install -e .
pip install -e controllers/position
pip install -e controllers/trajectory
```

## Regenerate controller artifacts

If you change `controllers/<name>/generate_model_definition/model_definition.yaml`,
regenerate the Python modules, C++ headers/sources and config template:

```bash
./generate.sh position
./generate.sh trajectory
./generate.sh all            # both
```

(Or invoke the generator directly:
`python3 -m mpc_acados_core.generate.model_definition_generation --controller position`.)

## Build (C++)

```bash
cmake -S . -B build \
  -DMPC_ACADOS_BUILD_POSITION=ON \
  -DMPC_ACADOS_BUILD_TRAJECTORY=ON \
  -DMPC_ACADOS_BUILD_EXAMPLES=ON
cmake --build build -j
```

CMake will **automatically regenerate the acados C code** (via
`controllers/<name>/generate_acados_c_code.py`) on the first build if it is
missing. To force a rebuild of the acados code, delete
`controllers/<name>/acados_<name>_mpc/` and reconfigure.

## Run the examples

### Python (position)

```bash
cd controllers/position/examples
python3 run_example.py \
  -c simulation_config.yaml \
  -m mpc_config.yaml \
  -f mpc_log.csv
python3 -m mpc_acados_core.plotting.plot_results -f mpc_log.csv
```

### Python (trajectory)

```bash
cd controllers/trajectory/examples
python3 run_example.py -c simulation_config.yaml -m mpc_config.yaml -f mpc_log.csv
```

### C++

```bash
./build/controllers/position/examples/mpc_acados_position_run_example \
  -c controllers/position/examples/simulation_config.yaml \
  -m controllers/position/examples/mpc_config.yaml \
  -f mpc_log.csv

./build/controllers/trajectory/examples/mpc_acados_trajectory_run_example \
  -c controllers/trajectory/examples/simulation_config.yaml \
  -m controllers/trajectory/examples/mpc_config.yaml \
  -f mpc_log.csv
```

Each run produces a CSV that can be visualised with the shared plotter:

```bash
python3 -m mpc_acados_core.plotting.plot_results -f simulator_logs/mpc_log.csv
```

## Adding a new controller variant

1. Create `controllers/<name>/generate_model_definition/model_definition.yaml`
   with the `state`, `actuation` and `parameters` sections for your variant.
2. Create `controllers/<name>/pyproject.toml` (copy from `position`) and rename
   the distribution to `mpc_acados_<name>`.
3. Add a thin `mpc_acados_<name>/__init__.py` that subclasses `MPCBase` and
   `AcadosMPCSolverBase` with the generated types.
4. Add a `generate_acados_c_code.py` entry point (copy from `position`).
5. Create `controllers/<name>/CMakeLists.txt` and
   `controllers/<name>/examples/CMakeLists.txt` (copy from `position`).
6. Register the variant in the top-level `CMakeLists.txt` with a new
   `MPC_ACADOS_BUILD_<NAME>` option.
7. `./generate.sh <name>` to emit datatypes and C++ wrappers.

## License

BSD-3-Clause. See [LICENSE](LICENSE).
