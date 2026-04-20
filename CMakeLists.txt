cmake_minimum_required(VERSION 3.16)
project(mpc_acados_core
  VERSION 0.1.0
  DESCRIPTION "Generic MPC core library built on acados."
  LANGUAGES CXX
)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Core INTERFACE library.
add_subdirectory(mpc_acados_core/cpp_interface)

# Controllers are independent CMake projects under controllers/<name>/.
# Build them with:
#   cmake -S controllers/position -B build/position
#   cmake -S controllers/trajectory -B build/trajectory
