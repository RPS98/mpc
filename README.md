# MPC

This repo implements a Model Predictive Controller (MPC) for a quadrotor, using [Acados](https://docs.acados.org/index.html#) library. The MPC is implemented in Python and C++.

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
 2. Copy them in /home/rafa/acados/bin
 3. Strip the version and platform from the binaries: as t_renderer-v0.0.34-X -> t_renderer)
 4. Enable execution privilege on the file "t_renderer" with:

```bash
chmod +x $ACADOS_ROOT_DIR/bin/t_renderer
```

## Generate C code for MPC

The MPC is implemented in Python and C++. The Python code generates the C code for the MPC. To generate the C code, run the following command:

```bash
export PYTHONPATH=$PYTHONPATH:"<path_to_this_repo>"
```

```bash
python3 mpc/export_c_code.py
```

## Test the MPC using the Python interface or the C++ interface

Once the C code is generated, you can test the MPC using the Python interface or the C++ interface.

```bash
python3 examples/integrator_example.py
```

```bash
./build/examples/acados_mpc_integrator_example
```

You can check the results in the `mpc_log.csv` file, and plot them with:

```bash
python3 examples/utils/plot_results.py
```