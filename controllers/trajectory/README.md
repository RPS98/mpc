# mpc_acados_trajectory

Trajectory-tracking MPC controller for quadrotor-style vehicles.

Uses **9 online parameters per stage**:

| Parameter              | Size | Description                                 |
| ---------------------- | ---- | ------------------------------------------- |
| `mass`                 | 1    | Vehicle mass (kg)                           |
| `desired_position`     | 3    | Reference position in world frame (m)       |
| `desired_orientation`  | 4    | Reference orientation as quaternion         |
| `desired_velocity`     | 3    | Reference linear velocity (m/s)             |
| `desired_acceleration` | 3    | Reference linear acceleration (m/s²)        |
| `external_force`       | 3    | External force in body frame (N)            |
| `Q`                    | 9    | Stage cost gains (pos + attitude + vel)     |
| `Qe`                   | 9    | Terminal cost gains                         |
| `R`                    | 4    | Control input cost gains                    |

State: `position[3]`, `orientation[4]`, `linear_velocity[3]` (10 DOF).  
Actuation: `thrust` (N), `angular_velocity[3]` (rad/s).

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

Generates `mpc_acados_trajectory/` (Python modules) and `include/`/`src/`
(C++ headers and sources) from `generate_model_definition/model_definition.yaml`.
Only needed when the model definition changes.

```bash
cd /path/to/this/controller
./generate.sh
```

Or from anywhere:

```bash
python3 -m mpc_acados_core.generate.model_definition_generation \
  --config /path/to/this/controller/generate_model_definition/model_definition.yaml
```

### Step 2 — Generate acados C code

Compiles the CasADi model and runs acados to produce the C solver under
`acados_trajectory_mpc/mpc_generated_code/`. Only needed once (or after changing
the solver definition).

```bash
cd /path/to/this/controller
python3 generate_acados_c_code.py
# or point to a custom solver definition:
python3 generate_acados_c_code.py --yaml examples/solver_definition_mpc_trajectory.yaml
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
missing. To force regeneration, delete `acados_trajectory_mpc/` and reconfigure.

### Step 4 — Install Python package (optional)

```bash
pip install -e /path/to/this/controller
```

---

## Run the examples

### Python

```bash
cd /path/to/this/controller
python3 examples/run_example.py \
  -c examples/simulation_config.yaml \
  -m examples/mpc_config.yaml \
  -f mpc_log.csv

# Plot results:
python3 -m mpc_acados_core.plotting.plot_results -f simulator_logs/mpc_log.csv
```

### C++

```bash
./build/examples/mpc_acados_trajectory_run_example \
  -c examples/simulation_config.yaml \
  -m examples/mpc_config.yaml \
  -f mpc_log.csv

# Plot results:
python3 -m mpc_acados_core.plotting.plot_results -f simulator_logs/mpc_log.csv
```

Available arguments:

| Flag | Default                               | Description                   |
| ---- | ------------------------------------- | ----------------------------- |
| `-c` | `examples/simulation_config.yaml`    | Simulation + waypoint config  |
| `-m` | `examples/mpc_config.yaml`           | MPC gains and bounds config   |
| `-f` | `mpc_log.csv`                        | Output CSV filename           |

---

## Files

| Path                                         | Hand-written | Notes                                     |
| -------------------------------------------- | ------------ | ----------------------------------------- |
| `generate_model_definition/model_definition.yaml` | yes     | Input to the code generator.              |
| `generate.sh`                                | yes          | Invokes the core generator.               |
| `generate_acados_c_code.py`                  | yes          | Generates the acados C solver.            |
| `CMakeLists.txt`                             | yes          | Builds `libmpc_acados_trajectory.so`.     |
| `pyproject.toml`                             | yes          | Pip distribution `mpc_acados_trajectory`. |
| `examples/`                                  | yes          | Run scripts and YAML configs.             |
| `config/mpc_config_template.yaml`            | generated    | Array sizes derived from model.           |
| `include/mpc_acados_trajectory/*.hpp`        | generated    | C++ datatype headers and wrappers.        |
| `src/*.cpp`                                  | generated    | C++ sources.                              |
| `mpc_acados_trajectory/*.py`                 | generated    | Python modules (state, actuation, ...).   |
| `tests/`                                     | generated    | C++ gtest skeleton.                       |
| `acados_trajectory_mpc/`                     | generated    | acados C code (from step 2).              |

---

## License

BSD-3-Clause.
