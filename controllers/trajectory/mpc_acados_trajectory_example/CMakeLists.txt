cmake_minimum_required(VERSION 3.16)
project(mpc_acados_trajectory_example CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# This example acts as a consumer of mpc_acados_trajectory. It:
#   1. On first configure, invokes the mpc_acados_trajectory.acados_solver
#      Python CLI to produce mpc_interface/ (a fresh copy of the library's
#      cpp_interface/ template + the acados C code).
#   2. add_subdirectory(mpc_interface) builds libmpc_acados_trajectory.so.
#   3. add_subdirectory(examples) builds the C++ run_example binary.
#
# Requirements: both ``mpc_acados_core`` and ``mpc_acados_trajectory`` must be
# importable by the Python interpreter CMake finds below (PYTHONPATH or
# editable install).
find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(MPC_INTERFACE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/mpc_interface")
set(MPC_INTERFACE_SO
  "${MPC_INTERFACE_DIR}/mpc_generated_code/mpc_generated_code/libacados_ocp_solver_mpc.so")

if(NOT EXISTS "${MPC_INTERFACE_SO}")
  message(STATUS
    "[mpc_acados_trajectory_example] mpc_interface/ missing, "
    "generating via mpc_acados_trajectory.acados_solver (one-time, ~30s)...")
  execute_process(
    COMMAND ${Python3_EXECUTABLE} -m mpc_acados_trajectory.acados_solver
            -c ${CMAKE_CURRENT_SOURCE_DIR}/solver_definition_mpc_trajectory.yaml
            -o ${CMAKE_CURRENT_SOURCE_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE _gen_result
    OUTPUT_VARIABLE _gen_output
    ERROR_VARIABLE _gen_output
  )
  if(NOT _gen_result EQUAL 0)
    message(FATAL_ERROR
      "mpc_acados_trajectory.acados_solver failed. "
      "Ensure mpc_acados_core and mpc_acados_trajectory are on PYTHONPATH. "
      "Output:\n${_gen_output}")
  endif()
endif()

# Path to mpc_acados_core for the generated mpc_interface/CMakeLists.txt.
# External consumers can override on the command line
# (-DMPC_ACADOS_CORE_DIR=...) before configuring the project.
if(NOT DEFINED MPC_ACADOS_CORE_DIR)
  set(MPC_ACADOS_CORE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mpc_acados_core/cpp_interface"
    CACHE PATH "Path to mpc_acados_core/cpp_interface")
endif()

add_subdirectory(mpc_interface)

# --- Fetch dynamic_trajectory_generator if needed ------------------------
# Both run_example.py and run_example.cpp consume DynamicTrajectory. When
# this directory is configured standalone, fetch the upstream repo into
# thirdparty/dynamic_trajectory_generator/ at configure time, build the
# C++ library (linked into the cpp binary) and the pybind module (imported
# by the Python driver). Consumers that already vendor DTG can disable
# the fetch with -DMPC_FETCH_DYNAMIC_TRAJECTORY_GENERATOR=OFF.
option(MPC_FETCH_DYNAMIC_TRAJECTORY_GENERATOR
       "Fetch + build dynamic_trajectory_generator for the trajectory example" ON)

if(MPC_FETCH_DYNAMIC_TRAJECTORY_GENERATOR)
  include(FetchContent)
  set(_DTG_LOCAL "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/dynamic_trajectory_generator")
  FetchContent_Declare(
    dynamic_trajectory_generator
    GIT_REPOSITORY https://github.com/cvar-upm/dynamic_trajectory_generator.git
    GIT_TAG        master
    SOURCE_DIR     ${_DTG_LOCAL}
  )
  FetchContent_MakeAvailable(dynamic_trajectory_generator)

  # Build the pybind binding (NOT included from DTG's top CMakeLists).
  add_subdirectory(${dynamic_trajectory_generator_SOURCE_DIR}/pybind
                   ${CMAKE_BINARY_DIR}/dtg_pybind)

  # Stage the pure-Python wrapper next to the build dir so the runner
  # script can add a single PYTHONPATH entry that exposes both the
  # wrapper and the freshly-compiled .so.
  set(_DTG_PY_SRC "${dynamic_trajectory_generator_SOURCE_DIR}/pybind/python/dynamic_trajectory_generator_py")
  set(_DTG_PY_DST "${CMAKE_BINARY_DIR}/python/dynamic_trajectory_generator_py")
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/python")
  if(EXISTS "${_DTG_PY_DST}" OR IS_SYMLINK "${_DTG_PY_DST}")
    file(REMOVE_RECURSE "${_DTG_PY_DST}")
  endif()
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
                  "${_DTG_PY_SRC}" "${_DTG_PY_DST}")
endif()

add_subdirectory(examples)
