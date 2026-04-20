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
#   1. Runs generate_mpc_interface.sh the first time to produce mpc_interface/
#      (a fresh copy of the library's cpp_interface/ + the acados C code).
#   2. add_subdirectory(mpc_interface) builds libmpc_acados_trajectory.so.
#   3. add_subdirectory(examples) builds the C++ run_example binary.
set(MPC_INTERFACE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/mpc_interface")
set(MPC_INTERFACE_SO
  "${MPC_INTERFACE_DIR}/mpc_generated_code/mpc_generated_code/libacados_ocp_solver_mpc.so")

if(NOT EXISTS "${MPC_INTERFACE_SO}")
  message(STATUS
    "[mpc_acados_trajectory_example] mpc_interface/ missing, "
    "running generate_mpc_interface.sh (one-time, ~30s)...")
  execute_process(
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/generate_mpc_interface.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE _gen_result
    OUTPUT_VARIABLE _gen_output
    ERROR_VARIABLE _gen_output
  )
  if(NOT _gen_result EQUAL 0)
    message(FATAL_ERROR
      "generate_mpc_interface.sh failed. Output:\n${_gen_output}")
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
add_subdirectory(examples)
