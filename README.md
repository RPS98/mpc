# MPC for Quadrotor using Acados

This repo implements a Model Predictive Controller (MPC) for a quadrotor, using [Acados](https://docs.acados.org/index.html#) library. The MPC is implemented in Python and C++.


## Installation

Clone this repository:

```bash
git clone https://github.com/RPS98/mpc.git
cd mpc
```

## Compile acados

Follow the instructions in the [Acados documentation](https://docs.acados.org/installation/index.html) to compile the library.

#### 1. Clone the repository

```bash
git clone https://github.com/acados/acados.git -b v0.5.3
cd acados
git submodule update --recursive --init
```

#### 2. Build the library

```bash
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON ..
# add more optional arguments e.g. -DACADOS_WITH_OSQP=OFF/ON -DACADOS_INSTALL_DIR=<path_to_acados_installation_folder> above
make install -j4
```

#### 3. Export the path to the library

```bash
export ACADOS_SOURCE_DIR="<path_to_acados_folder>"  # E.g. ~/acados
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACADOS_SOURCE_DIR/lib
export PYTHONPATH=$PYTHONPATH:$ACADOS_SOURCE_DIR/interfaces/acados_template/
```

#### 4. Install tera_renderer

For manual installation follow these instructions:

 1. Download binaries from https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux
 2. Copy them in <path_to_acados_folder>/acados/bin
 3. Strip the version and platform from the binaries: as t_renderer-v0.0.34-X -> t_renderer
 4. Enable execution privilege on the file "t_renderer" with:

```bash
chmod +x $ACADOS_ROOT_DIR/bin/t_renderer
```

#### 5. Install acados_template

From the acados root folder, run:

```bash
pip install -e interfaces/acados_template
```

#### 6. Install mpc package

From the mpc root folder, run:

```bash
pip install -e .
```

## 7. Usage

Configure a solver_definition yaml file in the project you want to use the MPC controller, e.g., `solver_definition.yaml`.
Then, run the following command in the terminal:

```bash
python3 -m mpc_position.acados_solver -c <path_to_solver_definition_yaml>
```

This will generate the c code in solver.export_dir. If cpp module is enabled, it will also generate a c++ project, to be able to compile it using add_submodule in cmake.
