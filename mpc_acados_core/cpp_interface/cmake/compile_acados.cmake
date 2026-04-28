# Copyright 2026 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


# Check if ACADOS_SOURCE_DIR is not set
if(NOT DEFINED ENV{ACADOS_SOURCE_DIR})
  MESSAGE(STATUS "ACADOS_SOURCE_DIR not found, cloning it")

  # Check if acados is already built
  include(FetchContent)
  FetchContent_Declare(
    acados
    GIT_REPOSITORY https://github.com/acados/acados.git
    GIT_TAG v0.5.0
  )
  # FetchContent_MakeAvailable(acados) # We need the make install
  FetchContent_Populate(acados)
  set(ENV{ACADOS_SOURCE_DIR} "${acados_SOURCE_DIR}")
endif()

# Check if acados is already built if folder ${acados_SOURCE_DIR}/build exists
set(ACADOS_BUILD_DIR "$ENV{ACADOS_SOURCE_DIR}/build")
if(NOT EXISTS "${ACADOS_BUILD_DIR}")
  MESSAGE(STATUS "Building acados at ${ACADOS_BUILD_DIR}")
  file(MAKE_DIRECTORY "${ACADOS_BUILD_DIR}")

  # Run CMake and make for acados
  execute_process(COMMAND cmake ..
                  WORKING_DIRECTORY "${ACADOS_BUILD_DIR}"
                  RESULT_VARIABLE result)
  
  if(result)
      message(FATAL_ERROR "Failed to configure acados")
  endif()
  
  execute_process(COMMAND make install
                  WORKING_DIRECTORY "${ACADOS_BUILD_DIR}"
                  RESULT_VARIABLE result)
  
  if(result)
      message(FATAL_ERROR "Failed to build acados")
  endif()

  # Install dependencies
  execute_process(COMMAND pip install -e interfaces/acados_template
    WORKING_DIRECTORY "$ENV{ACADOS_SOURCE_DIR}"
    RESULT_VARIABLE result)
endif()

# Check Tera is installed in acados
set(TERA_RENDERER_NAME "t_renderer")
set(TERA_RENDERER_PATH "$ENV{ACADOS_SOURCE_DIR}/bin/${TERA_RENDERER_NAME}")
if(NOT EXISTS ${TERA_RENDERER_PATH})
  set(TERA_RENDERER_URL "https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux")
  file(DOWNLOAD ${TERA_RENDERER_URL} ${TERA_RENDERER_PATH} SHOW_PROGRESS)
  execute_process(COMMAND chmod +x ${TERA_RENDERER_PATH}
                  RESULT_VARIABLE result)
  
  if(result)
      message(FATAL_ERROR "Failed to make executable ${TERA_RENDERER_PATH}")
  endif()
endif()
  
# Compile acados on acados_SOURCE_DIR
set(ENV{LD_LIBRARY_PATH} "$ENV{ACADOS_SOURCE_DIR}/lib:$ENV{LD_LIBRARY_PATH}")
set(ENV{PYTHONPATH} "$ENV{ACADOS_SOURCE_DIR}/interfaces/acados_template:$ENV{PYTHONPATH}")

# Export acados include directories
set(ACADOS_INCLUDE_DIRS 
  $ENV{ACADOS_SOURCE_DIR}/interfaces/
  $ENV{ACADOS_SOURCE_DIR}/include
	$ENV{ACADOS_SOURCE_DIR}/include/blasfeo/include/
	$ENV{ACADOS_SOURCE_DIR}/include/hpipm/include/
	$ENV{ACADOS_SOURCE_DIR}/include/acados/
	$ENV{ACADOS_SOURCE_DIR}/include/qpOASES_e/
)

set(ACADOS_LIB
  $ENV{ACADOS_SOURCE_DIR}/lib/libacados.so
  $ENV{ACADOS_SOURCE_DIR}/lib/libblasfeo.so
  $ENV{ACADOS_SOURCE_DIR}/lib/libhpipm.so
)

# # Include acados headers
include_directories(
  ${ACADOS_INCLUDE_DIRS}
)
