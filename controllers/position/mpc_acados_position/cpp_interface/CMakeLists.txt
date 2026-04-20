# THIS FILE HAS BEEN AUTOMATICALLY GENERATED USING templ_cpp_interface_cmakelists.txt.j2
#
# CMake target for mpc_acados_position (library template).
#
# This CMakeLists.txt is a template: it is NOT buildable in isolation. It
# expects the acados-generated C code to live in a sibling directory
# ``./mpc_generated_code/``. A consumer project is expected to copy this
# ``cpp_interface/`` next to the acados artifacts (typically as
# ``mpc_interface/`` via ``generate_mpc_interface.sh``) and then do
# ``add_subdirectory(mpc_interface)`` from its own CMakeLists.txt.
#
# The consumer must provide ``mpc_acados_core::mpc_acados_core``. This
# file will pull it via ``add_subdirectory(${MPC_ACADOS_CORE_DIR} ...)`` if
# the target is not already defined and ``MPC_ACADOS_CORE_DIR`` is set.

cmake_minimum_required(VERSION 3.16)
project(mpc_acados_position CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT TARGET mpc_acados_core::mpc_acados_core)
  if(NOT DEFINED MPC_ACADOS_CORE_DIR)
    message(FATAL_ERROR
      "mpc_acados_position: target mpc_acados_core::mpc_acados_core not found and "
      "MPC_ACADOS_CORE_DIR is not set. Either add mpc_acados_core to your build "
      "before add_subdirectory(mpc_acados_position), or pass "
      "-DMPC_ACADOS_CORE_DIR=<path-to-mpc_acados_core/cpp_interface>.")
  endif()
  add_subdirectory(${MPC_ACADOS_CORE_DIR} mpc_acados_core_build)
endif()

# acados writes its C code under ${export_dir}/mpc_generated_code/mpc_generated_code
# (the outer folder is the export_dir set in the solver yaml, the inner one
# is appended by acados itself during template rendering).
set(GEN_CODE_PARENT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/mpc_generated_code")
set(GEN_CODE_DIR        "${GEN_CODE_PARENT_DIR}/mpc_generated_code")

if(NOT EXISTS "${GEN_CODE_DIR}/libacados_ocp_solver_mpc.so")
  message(FATAL_ERROR
    "mpc_acados_position: acados C code not found under\n"
    "    ${GEN_CODE_DIR}\n"
    "Run generate_mpc_interface.sh in the consumer project to produce it.")
endif()

add_library(mpc_acados_position SHARED
  src/acados_mpc.cpp
  src/acados_sim_solver.cpp
  src/acados_mpc_datatype.cpp
)
add_library(mpc_acados_position::mpc_acados_position ALIAS mpc_acados_position)

target_include_directories(mpc_acados_position PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${GEN_CODE_PARENT_DIR}>
  $<BUILD_INTERFACE:${GEN_CODE_DIR}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mpc_acados_position
  PUBLIC
    mpc_acados_core::mpc_acados_core
    ${GEN_CODE_DIR}/libacados_ocp_solver_mpc.so
    ${GEN_CODE_DIR}/libacados_sim_solver_mpc.so
)

target_compile_features(mpc_acados_position PUBLIC cxx_std_17)
