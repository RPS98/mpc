# mpc_acados_position

Position-tracking MPC controller for quadrotor-style vehicles.

Uses **7 online parameters per stage**:

| Parameter            | Size | Description                              |
| -------------------- | ---- | ---------------------------------------- |
| `mass`               | 1    | Vehicle mass (kg)                        |
| `desired_position`   | 3    | Reference position in world frame (m)    |
| `desired_orientation`| 4    | Reference orientation as quaternion      |
| `external_force`     | 3    | External force in body frame (N)         |
| `Q`                  | 9    | Stage cost gains (pos + attitude + vel)  |
| `Qe`                 | 9    | Terminal cost gains                      |
| `R`                  | 4    | Control input cost gains                 |

State: `position[3]`, `orientation[4]`, `linear_velocity[3]` (10 DOF).  
Actuation: `thrust` (N), `angular_velocity[3]` (rad/s).

---

## Directory layout

```
controllers/position/
├── mpc_acados_position/              ← THE LIBRARY
│   ├── *.py                          (generated Python modules)
│   ├── include/mpc_acados_position/  (generated C++ headers)
│   ├── src/                          (generated C++ sources)
│   ├── acados_generated/             (generated acados C code)
│   └── tests/                        (generated C++ gtest skeleton)
├── mpc_acados_position_example/      ← GENERATE + EXAMPLE
│   ├── generate.sh                   (regenerate mpc_acados_position/ from model_definition.yaml)
│   ├── generate_acados_c_code.py     (generate acados C solver)
│   ├── config/mpc_config_template.yaml
│   └── examples/                     (run_example.py, run_example.cpp, configs)
├── model_definition.yaml             ← MODEL DEFINITION (input to the generator)
├── pyproject.toml
├── CMakeLists.txt
└── README.md
```

---

## Prerequisites

1. **acados** — build from source with `ACADOS_SOURCE_DIR` set:
   ```bash
   export ACADOS_SOURCE_DIR=/path/to/acados
   ```

2. **mpc_acados_core** — install the core library:
   ```bash
   pip install -e /path/to/mpc_acados_core_repo
   # or add to PYTHONPATH:
   export PYTHONPATH=/path/to/mpc_acados_core_repo:$PYTHONPATH
   ```

3. Python packages: `numpy`, `casadi`, `acados_template`, `pyyaml`, `jinja2`,
   `matplotlib`, `tqdm`

4. C++ build tools: CMake 3.16+, C++17 compiler, Eigen3, yaml-cpp

---

## Workflow

### Step 1 — Generate Python/C++ datatypes

Generates `mpc_acados_position/` (Python modules, C++ headers, C++ sources)
from `model_definition.yaml`. Only needed when the model definition changes.

```bash
cd mpc_acados_position_example
./generate.sh
```

Or from anywhere:

```bash
python3 -m mpc_acados_core.generate.model_definition_generation \
  --config /path/to/this/controller/model_definition.yaml
```

### Step 2 — Generate acados C code

Compiles the CasADi model and runs acados to produce the C solver under
`mpc_acados_position/acados_generated/`. Only needed once (or after changing
the solver definition).

```bash
cd mpc_acados_position_example
python3 generate_acados_c_code.py
# or point to a custom solver definition:
python3 generate_acados_c_code.py --yaml examples/solver_definition_mpc_position.yaml
```

### Step 3 — Build C++

```bash
cmake -S /path/to/this/controller -B build
cmake --build build -j

# With mpc_acados_core in a non-default location:
cmake -S /path/to/this/controller -B build \
  -DMPC_ACADOS_CORE_DIR=/path/to/mpc_acados_core/cpp_interface
cmake --build build -j
```

CMake will **auto-run step 2** at configure time if the generated `.so` is
missing. To force regeneration, delete `mpc_acados_position/acados_generated/`
and reconfigure.

### Step 4 — Install Python package (optional)

```bash
pip install -e /path/to/this/controller
```

---

## Run the examples

### Python

```bash
cd mpc_acados_position_example
python3 examples/run_example.py \
  -c examples/simulation_config.yaml \
  -m examples/mpc_config.yaml \
  -f mpc_log.csv

# Plot results:
python3 -m mpc_acados_core.plotting.plot_results -f simulator_logs/mpc_log.csv
```

### C++

```bash
./build/mpc_acados_position_example/examples/mpc_acados_position_run_example \
  -c mpc_acados_position_example/examples/simulation_config.yaml \
  -m mpc_acados_position_example/examples/mpc_config.yaml \
  -f mpc_log.csv

# Plot results:
python3 -m mpc_acados_core.plotting.plot_results -f simulator_logs/mpc_log.csv
```

Available arguments:

| Flag | Default                                                    | Description                   |
| ---- | ---------------------------------------------------------- | ----------------------------- |
| `-c` | `mpc_acados_position_example/examples/simulation_config.yaml` | Simulation + waypoint config  |
| `-m` | `mpc_acados_position_example/examples/mpc_config.yaml`    | MPC gains and bounds config   |
| `-f` | `mpc_log.csv`                                             | Output CSV filename           |

---

## Files

| Path                                                        | Hand-written | Notes                                    |
| ----------------------------------------------------------- | ------------ | ---------------------------------------- |
| `model_definition.yaml`                                     | yes          | Input to the code generator.             |
| `CMakeLists.txt`                                            | yes          | Builds `libmpc_acados_position.so`.      |
| `pyproject.toml`                                            | yes          | Pip distribution `mpc_acados_position`.  |
| `mpc_acados_position_example/generate.sh`                   | yes          | Invokes the core generator.              |
| `mpc_acados_position_example/generate_acados_c_code.py`     | yes          | Generates the acados C solver.           |
| `mpc_acados_position_example/examples/`                     | yes          | Run scripts and YAML configs.            |
| `mpc_acados_position/drone_model.py`                        | yes          | CasADi quadrotor dynamics.               |
| `mpc_acados_position/__init__.py`                           | yes          | Package exports.                         |
| `mpc_acados_position_example/config/mpc_config_template.yaml` | generated  | Array sizes derived from model.          |
| `mpc_acados_position/include/mpc_acados_position/*.hpp`     | generated    | C++ datatype headers and wrappers.       |
| `mpc_acados_position/src/*.cpp`                             | generated    | C++ sources.                             |
| `mpc_acados_position/*.py` (state, actuation, ...)          | generated    | Python modules.                          |
| `mpc_acados_position/tests/`                                | generated    | C++ gtest skeleton.                      |
| `mpc_acados_position/acados_generated/`                     | generated    | acados C code (from step 2).             |

---

## License

BSD-3-Clause.
