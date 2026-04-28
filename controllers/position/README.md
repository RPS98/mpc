# mpc_acados_position

Position-tracking MPC controller for quadrotor-style vehicles.

Uses **7 online parameters per stage**:

| Parameter             | Size | Description                             |
| --------------------- | ---- | --------------------------------------- |
| `mass`                | 1    | Vehicle mass (kg)                       |
| `desired_position`    | 3    | Reference position in world frame (m)   |
| `desired_orientation` | 4    | Reference orientation as quaternion     |
| `external_force`      | 3    | External force in body frame (N)        |
| `Q`                   | 9    | Stage cost gains (pos + attitude + vel) |
| `Qe`                  | 9    | Terminal cost gains                     |
| `R`                   | 4    | Control input cost gains                |

State: `position[3]`, `orientation[4]`, `linear_velocity[3]` (10 DOF).
Actuation: `thrust` (N), `angular_velocity[3]` (rad/s).

---

## Directory layout

```
controllers/position/
├── mpc_acados_position/                       ← LIBRARY (template)
│   ├── __init__.py                            (hand-written)
│   ├── drone_model.py                         (hand-written)
│   ├── model_definition/                      ← Python datatypes (generated)
│   │   ├── __init__.py, state.py, actuation.py,
│   │   │   parameters.py, dynamics.py
│   ├── utils/
│   │   ├── __init__.py, mpc_yaml.py           (generated)
│   └── cpp_interface/                         ← C++ template (not buildable alone)
│       ├── CMakeLists.txt                     (generated)
│       ├── include/mpc_acados_position/*.hpp  (generated)
│       ├── src/*.cpp                          (generated)
│       └── tests/                             (generated)
├── mpc_acados_position_example/               ← CONSUMER PROJECT
│   ├── CMakeLists.txt                         (entry point C++ build)
│   ├── generate_mpc_interface.sh              (thin wrapper around the Python CLI)
│   ├── solver_definition_mpc_position.yaml    (input to the Python CLI)
│   ├── mpc_interface/                         (generated, gitignored)
│   │   ├── CMakeLists.txt, include/, src/, tests/
│   │   └── mpc_generated_code/mpc_generated_code/
│   │       ├── libacados_ocp_solver_mpc.so
│   │       └── libacados_sim_solver_mpc.so
│   ├── examples/
│   │   ├── run_example.py, run_example.cpp
│   │   └── simulation_config.yaml, mpc_config.yaml, sim_yaml.hpp
│   ├── run_py_example.sh
│   └── run_cpp_example.sh
├── generate_datatypes.sh                      ← regenerate the library
├── model_definition.yaml                      ← MODEL DEFINITION (input)
├── pyproject.toml
└── README.md
```

`cpp_interface/` is deliberately not self-contained: it expects the
acados-generated C code in a sibling directory named `mpc_generated_code/`.
A consumer is expected to copy `cpp_interface/` somewhere writable, drop the
acados artifacts next to it, and `add_subdirectory(<the_copy>)`. That is
exactly what the `mpc_acados_position.acados_solver` Python CLI does to
produce `mpc_interface/` (invokable from any directory; see workflow below).

---

## Prerequisites

1. **acados** — build from source with `ACADOS_SOURCE_DIR` exported:
   ```bash
   export ACADOS_SOURCE_DIR=/path/to/acados
   ```
2. **mpc_acados_core** and **mpc_acados_position** — install or put on `PYTHONPATH`:
   ```bash
   # Option A: editable install
   pip install -e /path/to/mpc
   pip install -e /path/to/mpc/controllers/position

   # Option B: PYTHONPATH export (e.g., in .bashrc or before running CMake/CLI)
   export PYTHONPATH=/path/to/mpc:/path/to/mpc/controllers/position:$PYTHONPATH
   ```
3. Python packages: `numpy`, `casadi`, `acados_template`, `pyyaml`,
   `jinja2`, `matplotlib`, `tqdm`.
4. C++ toolchain: CMake 3.16+, C++17 compiler, Eigen3, yaml-cpp.

---

## Workflow

### 1. Regenerate the library (when `model_definition.yaml` changes)

```bash
cd controllers/position
./generate_datatypes.sh
```

Produces Python modules under `mpc_acados_position/model_definition/` and
`mpc_acados_position/utils/`, the C++ headers/sources/tests under
`mpc_acados_position/cpp_interface/`, and the C++ template's
`cpp_interface/CMakeLists.txt`.

Equivalent call (run from anywhere):

```bash
python3 -m mpc_acados_core.generate.model_definition_generation \
  --config /path/to/controllers/position/model_definition.yaml
```

### 2. Install the Python package (optional)

```bash
pip install -e controllers/position
```

### 3. Build the C++ example

```bash
cd controllers/position/mpc_acados_position_example
cmake -S . -B build
cmake --build build -j
```

The very first CMake configure invokes the Python CLI automatically:

```bash
python3 -m mpc_acados_position.acados_solver \
  -c .../solver_definition_mpc_position.yaml \
  -o .../mpc_acados_position_example
```

The CLI:

1. Locates the `cpp_interface/` template inside the installed
   `mpc_acados_position` package (no relative paths).
2. Copies it to `mpc_acados_position_example/mpc_interface/`.
3. Runs acados (via the Python API) with
   `solver_definition_mpc_position.yaml`, writing the C solver to
   `mpc_interface/mpc_generated_code/mpc_generated_code/`.

After that the CMake graph is just
`add_subdirectory(mpc_interface) + add_subdirectory(examples)`.

Equivalent invocations from any directory:

```bash
# Directly via the Python CLI (output defaults to YAML parent dir)
python3 -m mpc_acados_position.acados_solver \
  -c /abs/path/.../solver_definition_mpc_position.yaml

# Via the thin shell wrapper (output goes next to the script)
bash /abs/path/.../mpc_acados_position_example/generate_mpc_interface.sh
```

To force a fresh `mpc_interface/` (e.g. after editing the solver
definition), delete it and re-run CMake, or rerun the CLI / shell wrapper
by hand.

### 4. Run the examples

```bash
# Python:
./run_py_example.sh

# C++ (needs build from step 3):
./run_cpp_example.sh
```

Both produce `simulator_logs/mpc_log.csv` and open a plot.

---

## Using the controller in a separate repository

`mpc_acados_position_example/` is intentionally laid out as a minimal
downstream consumer. To integrate the controller in another project:

1. Make sure `mpc_acados_core` and `mpc_acados_position` are importable
   (editable install or `PYTHONPATH`).
2. Copy `solver_definition_mpc_position.yaml` into your repo (or author
   your own with the same schema).
3. Run the CLI from anywhere:
   ```bash
   python3 -m mpc_acados_position.acados_solver \
     -c /path/to/your/solver_definition_mpc_position.yaml \
     -o /path/in/your/repo/where/to/put/mpc_interface
   ```
4. Set `MPC_ACADOS_CORE_DIR` in your CMakeLists (or pass
   `-DMPC_ACADOS_CORE_DIR=<path-to-mpc_acados_core/cpp_interface>` on the
   configure line) and do `add_subdirectory(mpc_interface)`.

---

## Files

| Path                                                              | Hand-written | Notes                                        |
| ----------------------------------------------------------------- | ------------ | -------------------------------------------- |
| `model_definition.yaml`                                           | yes          | Input to `generate_datatypes.sh`.            |
| `generate_datatypes.sh`                                           | yes          | Regenerates the library template.            |
| `pyproject.toml`                                                  | yes          | Pip distribution `mpc_acados_position`.      |
| `mpc_acados_position/__init__.py`                                 | yes          | Package exports.                             |
| `mpc_acados_position/drone_model.py`                              | yes          | CasADi quadrotor dynamics.                   |
| `mpc_acados_position/model_definition/*.py`                       | generated    | State, actuation, parameters, dynamics.      |
| `mpc_acados_position/utils/mpc_yaml.py`                           | generated    | YAML loader.                                 |
| `mpc_acados_position/cpp_interface/CMakeLists.txt`                | generated    | Builds the library target (consumed).        |
| `mpc_acados_position/cpp_interface/include/mpc_acados_position/*` | generated    | C++ headers.                                 |
| `mpc_acados_position/cpp_interface/src/*.cpp`                     | generated    | C++ sources.                                 |
| `mpc_acados_position/cpp_interface/tests/*`                       | generated    | Gtest skeleton.                              |
| `mpc_acados_position_example/CMakeLists.txt`                      | yes          | Example C++ entry point.                     |
| `mpc_acados_position/acados_solver.py`                            | yes          | Python CLI entry point (`-m mpc_acados_position.acados_solver`). |
| `mpc_acados_position_example/generate_mpc_interface.sh`           | yes          | Thin wrapper around the Python CLI.          |
| `mpc_acados_position_example/solver_definition_mpc_position.yaml` | yes          | Acados solver config.                        |
| `mpc_acados_position_example/examples/*`                          | yes          | Python + C++ run scripts and their configs. |
| `mpc_acados_position_example/run_{py,cpp}_example.sh`             | yes          | Convenience wrappers.                        |
| `mpc_acados_position_example/mpc_interface/`                      | generated    | Gitignored; produced by `generate_mpc_interface.sh`. |

---

## License

BSD-3-Clause.
