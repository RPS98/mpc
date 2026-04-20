find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(mpc_acados_position_run_example run_example.cpp)

target_include_directories(mpc_acados_position_run_example PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(mpc_acados_position_run_example
  PRIVATE
    mpc_acados_position::mpc_acados_position
    Eigen3::Eigen
    yaml-cpp
)
