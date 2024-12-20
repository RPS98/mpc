# MPC

This repo implements a Model Predictive Controller (MPC) for a quadrotor, using [Acados](https://docs.acados.org/index.html#) library. The MPC is implemented in Python and C++.


## Installation

Clone this repository:

```bash
git clone hhttps://github.com/RPS98/mpc.git
cd mpc
```

## Compile acados

Follow the instructions in the [Acados documentation](https://docs.acados.org/installation/index.html) to compile the library.

#### 1. Clone the repository

```bash
git clone https://github.com/acados/acados.git
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
export ACADOS_ROOT_DIR="<path_to_acados_folder>"
export PYTHONPATH=$PYTHONPATH:"$ACADOS_ROOT_DIR/interfaces/acados_template/"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$ACADOS_ROOT_DIR/acados/lib"
export ACADOS_SOURCE_DIR="$ACADOS_ROOT_DIR/acados"
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

```bash
pip install -e <acados_root>/interfaces/acados_template
```

## Generate C code for MPC

The MPC is implemented in Python and C++. The Python code generates the C code for the MPC. To generate the C code, run the following command:

```bash
export PYTHONPATH=$PYTHONPATH:"<path_to_this_repo>"
```

```bash
python3 mpc/mpc_controller.py
```

## Build this repository

To build this repostory, follow the instructions from the root folder:

```bash
mkdir -p build
cd build
make -j4
```

## Example of the MPC using acados sim solver

You can run the MPC using the Python interface, from the root folder:

```bash
python3 examples/run_example.py
```

> [!NOTE] 
> [dynamic_trajectory_generator](https://github.com/miferco97/dynamic_trajectory_generator.git) dependency is required to run the example. Clone the repository and add the path to the LD_LIBRARY_PATH.


## Example of the MPC using acados sim solver with C++ interface

You can build the C++ interface and run the MPC using the C++ interface, from the root folder:

```bash
mkdir -p build
cd build
cmake ..
make -j4 -DBUILD_EXAMPLES=ON
```

You can run the example:

```bash
./build/examples/acados_mpc_run_example
```
