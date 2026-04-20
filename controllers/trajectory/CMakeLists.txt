cmake_minimum_required(VERSION 3.16)
project(mpc_acados_trajectory CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(MPC_ACADOS_TRAJECTORY_CODEGEN_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/acados_trajectory_mpc"
  CACHE PATH
  "Directory that contains mpc_generated_code for the trajectory controller."
)
# acados writes the C code under ${export_dir}/mpc_generated_code/mpc_generated_code
# (the outer folder is the export_dir from the solver yaml, the inner one is
# added automatically by the acados template render step).
set(MPC_ACADOS_TRAJECTORY_GEN_INCLUDE_PARENT
  "${MPC_ACADOS_TRAJECTORY_CODEGEN_ROOT}/mpc_generated_code")
set(MPC_ACADOS_TRAJECTORY_GEN_CODE_DIR
  "${MPC_ACADOS_TRAJECTORY_GEN_INCLUDE_PARENT}/mpc_generated_code")

if(NOT EXISTS "${MPC_ACADOS_TRAJECTORY_GEN_CODE_DIR}/libacados_ocp_solver_mpc.so")
  message(STATUS "[trajectory] Generating acados C code (one-time, ~30s)...")
  execute_process(
    COMMAND ${Python3_EXECUTABLE} generate_acados_c_code.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE _gen_result
    OUTPUT_VARIABLE _gen_output
    ERROR_VARIABLE _gen_output
  )
  if(NOT _gen_result EQUAL 0)
    message(FATAL_ERROR
      "[trajectory] Failed to generate acados C code. Output:\n${_gen_output}")
  endif()
endif()

set(MPC_ACADOS_TRAJECTORY_GEN_INCLUDE_DIRS
  ${MPC_ACADOS_TRAJECTORY_GEN_INCLUDE_PARENT}
  ${MPC_ACADOS_TRAJECTORY_GEN_CODE_DIR}
)
set(MPC_ACADOS_TRAJECTORY_GEN_LIBS
  ${MPC_ACADOS_TRAJECTORY_GEN_CODE_DIR}/libacados_ocp_solver_mpc.so
  ${MPC_ACADOS_TRAJECTORY_GEN_CODE_DIR}/libacados_sim_solver_mpc.so
)

set(MPC_ACADOS_TRAJECTORY_SOURCES
  src/acados_mpc.cpp
  src/acados_sim_solver.cpp
  src/acados_mpc_datatype.cpp
)

add_library(mpc_acados_trajectory SHARED ${MPC_ACADOS_TRAJECTORY_SOURCES})
add_library(mpc_acados_trajectory::mpc_acados_trajectory ALIAS mpc_acados_trajectory)

target_include_directories(mpc_acados_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
foreach(_gen_inc ${MPC_ACADOS_TRAJECTORY_GEN_INCLUDE_DIRS})
  target_include_directories(mpc_acados_trajectory PUBLIC
    $<BUILD_INTERFACE:${_gen_inc}>
  )
endforeach()

target_link_libraries(mpc_acados_trajectory
  PUBLIC
    mpc_acados_core::mpc_acados_core
    ${MPC_ACADOS_TRAJECTORY_GEN_LIBS}
)

target_compile_features(mpc_acados_trajectory PUBLIC cxx_std_17)

if(MPC_ACADOS_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
