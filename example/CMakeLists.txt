cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME acados_mpc)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  Eigen3
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

# Add acados mpc interface
add_subdirectory(acados_mpc)

# Add executable for each example
add_subdirectory(examples)