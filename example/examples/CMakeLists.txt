# Try to find dynamic_trajectory_generator

find_package(dynamic_trajectory_generator QUIET)
if(${dynamic_trajectory_generator_FOUND})
  MESSAGE(STATUS "dynamic_trajectory_generator found")
else(${dynamic_trajectory_generator_FOUND})
  MESSAGE(STATUS "dynamic_trajectory_generator not found")
  include(FetchContent)
  FetchContent_Declare(
    dynamic_trajectory_generator
    GIT_REPOSITORY https://github.com/miferco97/dynamic_trajectory_generator.git
    GIT_TAG master
  )
  FetchContent_MakeAvailable(dynamic_trajectory_generator)
endif(${dynamic_trajectory_generator_FOUND})

include_directories(
  examples
)

file(GLOB EXAMPLES_SOURCE "*_example.cpp")

# Print all examples
foreach(EXAMPLE_FILE ${EXAMPLES_SOURCE})
  get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
  MESSAGE(STATUS "Adding traj example: ${EXAMPLE_NAME}")
endforeach()

foreach(EXAMPLE_FILE ${EXAMPLES_SOURCE})
  get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)

  add_executable(${PROJECT_NAME}_${EXAMPLE_NAME} ${EXAMPLE_FILE})
  target_link_options(${PROJECT_NAME}_${EXAMPLE_NAME} PRIVATE "-Wl,--disable-new-dtags")
  set_target_properties(${PROJECT_NAME}_${EXAMPLE_NAME} PROPERTIES
    BUILD_RPATH
      "$<TARGET_FILE_DIR:acados_mpc>;$ENV{ACADOS_SOURCE_DIR}/lib;${CMAKE_SOURCE_DIR}/acados_mpc/mpc_generated_code/mpc_generated_code"
  )
  target_link_libraries(${PROJECT_NAME}_${EXAMPLE_NAME} acados_mpc
    dynamic_trajectory_generator
  )
endforeach()
