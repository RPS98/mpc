cmake_minimum_required(VERSION 3.16)
project(mpc_acados_core CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Build the acados runtime (or attach to an existing one) and publish
# ACADOS_INCLUDE_DIRS / ACADOS_LIB in the current scope.
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/compile_acados.cmake)

# The core is header-only: CsvLogger (and in the future, other shared helpers).
# Expose it as an INTERFACE target that also re-exports the acados include
# directories and libraries so controllers don't have to include
# compile_acados.cmake themselves.
add_library(mpc_acados_core INTERFACE)
add_library(mpc_acados_core::mpc_acados_core ALIAS mpc_acados_core)

target_include_directories(mpc_acados_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
foreach(_acados_inc ${ACADOS_INCLUDE_DIRS})
  target_include_directories(mpc_acados_core INTERFACE
    $<BUILD_INTERFACE:${_acados_inc}>
  )
endforeach()

target_link_libraries(mpc_acados_core INTERFACE
  ${ACADOS_LIB}
)

target_compile_features(mpc_acados_core INTERFACE cxx_std_17)

# Propagate acados-related vars to the parent scope so sibling controllers
# (controllers/<name>) can also reference them when generating acados C code.
set(ACADOS_INCLUDE_DIRS "${ACADOS_INCLUDE_DIRS}" PARENT_SCOPE)
set(ACADOS_LIB "${ACADOS_LIB}" PARENT_SCOPE)
set(ACADOS_SOURCE_DIR "$ENV{ACADOS_SOURCE_DIR}" PARENT_SCOPE)
