# mpc_acados_trajectory_example

Minimal consumer project showing how to integrate the
`mpc_acados_trajectory` controller into a downstream repository.

The directory is intentionally self-contained: a single script generates the
C++ `mpc_interface/` subproject, another builds it, and two wrappers run the
closed-loop Python and C++ examples.

---

## Directory layout

```
mpc_acados_trajectory_example/
├── CMakeLists.txt                           ← C++ entry point (add_subdirectory only)
├── build.sh                                 ← generate (if needed) + configure + build
├── generate_mpc_interface.sh                ← produces mpc_interface/ from library + acados
├── solver_definition_mpc_trajectory.yaml    ← acados solver config (input to generate)
├── mpc_interface/                           ← GENERATED, gitignored
│   ├── CMakeLists.txt, include/, src/, tests/
│   └── mpc_generated_code/mpc_generated_code/
│       ├── libacados_ocp_solver_mpc.so
│       └── libacados_sim_solver_mpc.so
├── examples/
│   ├── CMakeLists.txt
│   ├── run_example.py, run_example.cpp
│   ├── simulation_config.yaml, mpc_config.yaml
│   └── sim_yaml.hpp
├── run_py_example.sh
└── run_cpp_example.sh
```

---

## Prerequisites

1. `acados` built from source with `ACADOS_SOURCE_DIR` exported.
2. `mpc_acados_core` on `PYTHONPATH` or installed (`pip install -e
   /path/to/mpc_acados_core`).
3. `mpc_acados_trajectory` installed (`pip install -e
   /path/to/controllers/trajectory`). If you cloned this repo, run first
   `controllers/trajectory/generate_datatypes.sh` to generate the library.
4. Python: `numpy`, `casadi`, `acados_template`, `pyyaml`, `jinja2`,
   `matplotlib`, `tqdm`. The Python example also needs
   `dynamic_trajectory_generator_py`.
5. C++ toolchain: CMake 3.16+, C++17, Eigen3, yaml-cpp.

---

## 1. Generate `mpc_interface/`

```bash
./generate_mpc_interface.sh
```

What it does:

1. Copies `../mpc_acados_trajectory/cpp_interface/` into `mpc_interface/`.
2. Invokes acados (via `AcadosMPCSolver`) with
   `solver_definition_mpc_trajectory.yaml`, writing the C solver to
   `mpc_interface/mpc_generated_code/mpc_generated_code/`.

Re-run this script whenever `solver_definition_mpc_trajectory.yaml` or the
library's `cpp_interface/` template changes. `build.sh` also triggers it
automatically when `mpc_interface/` is missing.

---

## 2. Build the C++ example

```bash
./build.sh              # incremental build
./build.sh --clean      # wipe build/ first
./build.sh --regen      # wipe mpc_interface/ and regenerate before building
```

This wraps the usual CMake flow:

```bash
cmake -S . -B build
cmake --build build -j
```

After a successful build you get:

```
build/mpc_interface/libmpc_acados_trajectory.so
build/examples/mpc_acados_trajectory_run_example
```

---

## 3. Run the examples

```bash
./run_py_example.sh     # Python closed-loop + plot
./run_cpp_example.sh    # C++ closed-loop + plot
```

Both produce `simulator_logs/mpc_log.csv` and open a plot via
`mpc_acados_core.plotting.plot_results`.

Inputs (editable):
- `examples/simulation_config.yaml` — simulator + controller wiring.
- `examples/mpc_config.yaml` — online parameters (mass, gains, references).

---

## Using this as a template in another repo

1. Copy this directory into your project.
2. Edit `generate_mpc_interface.sh` and set `LIB_CPP_DIR` to your local
   `mpc_acados_trajectory/cpp_interface/` path.
3. Pass your `mpc_acados_core/cpp_interface/` location with
   `-DMPC_ACADOS_CORE_DIR=...` (or edit the default in `CMakeLists.txt`).
4. `./generate_mpc_interface.sh && ./build.sh`.
5. From your top-level `CMakeLists.txt`, either `add_subdirectory(<this
   dir>)` or `add_subdirectory(<this dir>/mpc_interface)` if you only want
   the library target `mpc_acados_trajectory::mpc_acados_trajectory`.
