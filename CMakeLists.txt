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

# Add acados
include(cmake/compile_acados.cmake)

# Generate acados c code
include(cmake/generate_acados_c_code.cmake)

# find dependencies
set(PROJECT_DEPENDENCIES
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
)

set(SOURCE_CPP_FILES
  src/${PROJECT_NAME}.cpp
  src/acados_sim_solver.cpp
  src/acados_mpc_datatype.cpp
)

# Create library
add_library(${PROJECT_NAME} SHARED ${SOURCE_CPP_FILES})

# Export library headers
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Link the library with acados and other dependencies
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_DEPENDENCIES}
  ${ACADOS_C_GENERATED_CODE_LIB}
  ${ACADOS_LIB}
)

# Tests
include(CTest)

# Check if tests are enabled
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()