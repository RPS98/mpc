include_directories(
  examples
  examples/spline
)

# Find yaml-cpp
find_package(yaml-cpp REQUIRED)

file(GLOB EXAMPLES_SOURCE "*_example.cpp")

# Print all examples
foreach(EXAMPLE_FILE ${EXAMPLES_SOURCE})
  get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
  MESSAGE(STATUS "Adding traj example: ${EXAMPLE_NAME}")
endforeach()

foreach(EXAMPLE_FILE ${EXAMPLES_SOURCE})
  get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)

  add_executable(${PROJECT_NAME}_${EXAMPLE_NAME} 
    ${EXAMPLE_FILE} 
    spline/spline.cpp 
    spline/spline_trajectory_generator.cpp)
  target_link_libraries(${PROJECT_NAME}_${EXAMPLE_NAME} acados_mpcc_solver yaml-cpp)
endforeach()