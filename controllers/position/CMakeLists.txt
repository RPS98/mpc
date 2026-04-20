cmake_minimum_required(VERSION 3.16)
project(mpc_acados_position CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(MPC_ACADOS_BUILD_EXAMPLES "Build the C++ examples." ON)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Locate mpc_acados_core — override with -DMPC_ACADOS_CORE_DIR=<path>.
# Default assumes this controller lives inside the mpc_acados monorepo at
# controllers/position/. Adjust if the controller is a standalone repo.
if(NOT TARGET mpc_acados_core::mpc_acados_core)
  if(NOT DEFINED MPC_ACADOS_CORE_DIR)
    set(MPC_ACADOS_CORE_DIR
      "${CMAKE_CURRENT_SOURCE_DIR}/../../mpc_acados_core/cpp_interface"
      CACHE PATH "Path to mpc_acados_core/cpp_interface")
  endif()
  add_subdirectory(${MPC_ACADOS_CORE_DIR} mpc_acados_core_build)
endif()

# Where acados generates the C code when generate_acados_c_code.py is run.
set(MPC_ACADOS_POSITION_CODEGEN_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/acados_position_mpc"
  CACHE PATH
  "Directory that contains mpc_generated_code for the position controller."
)
# acados writes the C code under ${export_dir}/mpc_generated_code/mpc_generated_code
# (the outer folder is the export_dir from the solver yaml, the inner one is
# added automatically by the acados template render step).
set(MPC_ACADOS_POSITION_GEN_INCLUDE_PARENT
  "${MPC_ACADOS_POSITION_CODEGEN_ROOT}/mpc_generated_code")
set(MPC_ACADOS_POSITION_GEN_CODE_DIR
  "${MPC_ACADOS_POSITION_GEN_INCLUDE_PARENT}/mpc_generated_code")

# Regenerate the acados C code at configure time if it is missing. Users can
# force regeneration by removing ${MPC_ACADOS_POSITION_CODEGEN_ROOT} or by
# running generate_acados_c_code.py manually.
if(NOT EXISTS "${MPC_ACADOS_POSITION_GEN_CODE_DIR}/libacados_ocp_solver_mpc.so")
  message(STATUS "[position] Generating acados C code (one-time, ~30s)...")
  execute_process(
    COMMAND ${Python3_EXECUTABLE} generate_acados_c_code.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE _gen_result
    OUTPUT_VARIABLE _gen_output
    ERROR_VARIABLE _gen_output
  )
  if(NOT _gen_result EQUAL 0)
    message(FATAL_ERROR
      "[position] Failed to generate acados C code. Output:\n${_gen_output}")
  endif()
endif()

set(MPC_ACADOS_POSITION_GEN_INCLUDE_DIRS
  ${MPC_ACADOS_POSITION_GEN_INCLUDE_PARENT}
  ${MPC_ACADOS_POSITION_GEN_CODE_DIR}
)
set(MPC_ACADOS_POSITION_GEN_LIBS
  ${MPC_ACADOS_POSITION_GEN_CODE_DIR}/libacados_ocp_solver_mpc.so
  ${MPC_ACADOS_POSITION_GEN_CODE_DIR}/libacados_sim_solver_mpc.so
)

set(MPC_ACADOS_POSITION_SOURCES
  src/acados_mpc.cpp
  src/acados_sim_solver.cpp
  src/acados_mpc_datatype.cpp
)

add_library(mpc_acados_position SHARED ${MPC_ACADOS_POSITION_SOURCES})
add_library(mpc_acados_position::mpc_acados_position ALIAS mpc_acados_position)

target_include_directories(mpc_acados_position PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
foreach(_gen_inc ${MPC_ACADOS_POSITION_GEN_INCLUDE_DIRS})
  target_include_directories(mpc_acados_position PUBLIC
    $<BUILD_INTERFACE:${_gen_inc}>
  )
endforeach()

target_link_libraries(mpc_acados_position
  PUBLIC
    mpc_acados_core::mpc_acados_core
    ${MPC_ACADOS_POSITION_GEN_LIBS}
)

target_compile_features(mpc_acados_position PUBLIC cxx_std_17)

if(MPC_ACADOS_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
