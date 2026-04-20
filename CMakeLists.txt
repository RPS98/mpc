cmake_minimum_required(VERSION 3.16)
project(mpc_acados
  VERSION 0.1.0
  DESCRIPTION "Generic MPC library built on acados with pluggable variants."
  LANGUAGES CXX
)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Per-variant build toggles.
option(MPC_ACADOS_BUILD_POSITION "Build the position controller variant." ON)
option(MPC_ACADOS_BUILD_TRAJECTORY "Build the trajectory controller variant." ON)
option(MPC_ACADOS_BUILD_EXAMPLES "Build the C++ examples for each variant." ON)

# Python is required to (re)generate acados C code if absent.
find_package(Python3 REQUIRED COMPONENTS Interpreter)

message(STATUS "mpc_acados build summary:")
message(STATUS "  position   : ${MPC_ACADOS_BUILD_POSITION}")
message(STATUS "  trajectory : ${MPC_ACADOS_BUILD_TRAJECTORY}")
message(STATUS "  examples   : ${MPC_ACADOS_BUILD_EXAMPLES}")

# Core library (INTERFACE) — always built because every variant depends on it.
add_subdirectory(mpc_acados_core/cpp_interface)

if(MPC_ACADOS_BUILD_POSITION)
  add_subdirectory(controllers/position)
endif()

if(MPC_ACADOS_BUILD_TRAJECTORY)
  add_subdirectory(controllers/trajectory)
endif()
