# We do it beacuse we want acados to make the install properly, to generate the include folder

# Copy acados folder to build directory
if(NOT EXISTS "${CMAKE_CURRENT_BINARY_DIR}/acados")
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/acados" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
endif()

# Build acados in its own build directory
set(ACADOS_DIR "${CMAKE_CURRENT_BINARY_DIR}/acados")
set(ACADOS_BUILD_DIR "${ACADOS_DIR}/build")

if(NOT EXISTS "${ACADOS_BUILD_DIR}")
  message(STATUS "Building acados")
  file(MAKE_DIRECTORY "${ACADOS_BUILD_DIR}")

  # Run CMake and make for acados
  execute_process(COMMAND cmake ..
                  WORKING_DIRECTORY "${ACADOS_BUILD_DIR}"
                  RESULT_VARIABLE result)

  if(result)
      message(FATAL_ERROR "Failed to configure acados")
  endif()

  execute_process(COMMAND make install -j14
                  WORKING_DIRECTORY "${ACADOS_BUILD_DIR}"
                  RESULT_VARIABLE result)

  if(result)
      message(FATAL_ERROR "Failed to build acados")
  endif()

else()
  message(STATUS "acados already built")
endif()

# Export acados include directories
set(ACADOS_INCLUDE_DIRS "${ACADOS_DIR}/include")
set(ACADOS_LIB
  "${ACADOS_DIR}/lib/libacados.so"
  "${ACADOS_DIR}/lib/libblasfeo.so"
  "${ACADOS_DIR}/lib/libhpipm.so"
)
set(ENV{ACADOS_SOURCE_DIR} "${ACADOS_DIR}")

# Include acados headers
include_directories(
  ${ACADOS_INCLUDE_DIRS}
	${ACADOS_INCLUDE_DIRS}/blasfeo/include/
	${ACADOS_INCLUDE_DIRS}/hpipm/include/
	${ACADOS_INCLUDE_DIRS}/acados/
	${ACADOS_INCLUDE_DIRS}/qpOASES_e/
)


