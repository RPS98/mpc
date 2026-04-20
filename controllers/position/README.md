# mpc_acados_position

Position-tracking MPC controller variant. Uses 7 online parameters per stage:
`desired_position[3]` + `desired_orientation[4]`.

See [`../../README.md`](../../README.md) for the overall architecture, build
and install instructions. The Python distribution is `mpc_acados_position`
and depends on `mpc_acados_core`.

## Files in this directory

| Path                                        | Hand-written? | Notes                                      |
| ------------------------------------------- | ------------- | ------------------------------------------ |
| `generate_model_definition/model_definition.yaml` | yes           | Input to the core code generator.          |
| `config/mpc_config_template.yaml`           | generated     | Regenerated whenever the yaml above changes. |
| `include/mpc_acados_position/*.hpp`         | generated     | C++ headers (datatypes + wrapper).         |
| `src/*.cpp`                                 | generated     | C++ sources.                               |
| `mpc_acados_position/*.py`                  | generated     | Python package.                            |
| `examples/run_example.py`, `run_example.cpp` | yes           | Standalone runners.                        |
| `examples/*.yaml`                           | yes           | Runtime configs used by the examples.      |
| `generate_acados_c_code.py`                 | yes           | Invokes acados to produce the C solver.    |
| `CMakeLists.txt`                            | yes           | Builds `libmpc_acados_position.so`.        |

## Regenerate

```bash
# from repo root
./generate.sh position
```
