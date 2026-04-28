# Copyright 2025 Universidad Politécnica de Madrid
# Author: Rafael Perez-Segui <r.psegui@upm.es>
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.16)
project(mpc_acados_core
  VERSION 0.1.0
  DESCRIPTION "Generic MPC core library built on acados."
  LANGUAGES CXX
)

# --- Options ---
# BUILD_PYBIND kept for uniformity with other submodules; this package is
# pure-Python (no pybind11 bindings) so the flag is currently a no-op.
option(BUILD_PYBIND  "Build Python bindings (no-op: mpc is pure-Python)" OFF)
option(BUILD_TESTING "Build C++ tests"                                   OFF)

# --- C++ standard ---
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Core INTERFACE library (C++ side of mpc_acados_core).
add_subdirectory(mpc_acados_core/cpp_interface)

# Controllers (position / trajectory) are independent CMake projects under
# controllers/<name>/<pkg>/cpp_interface/, intended to be instantiated into a
# consumer project after generating the acados C code with
# ``generate_mpc_interface.sh``. They are NOT built from this root.

# ---------------------------------------------------------------------------
# Pure-Python mirror
# ---------------------------------------------------------------------------
# When built as part of the top-level mpc_examples tree, ``PYBIND_PY_MIRROR_ROOT``
# points to the centralized directory where all compiled .so modules and
# pure-Python packages are gathered so a single PYTHONPATH entry exposes
# everything. Since mpc_acados_core / mpc_acados_position / mpc_acados_trajectory
# are pure-Python, we expose them via symlinks — live edits to the source tree
# are immediately visible through the mirror without re-building.
if(DEFINED PYBIND_PY_MIRROR_ROOT)
  file(MAKE_DIRECTORY "${PYBIND_PY_MIRROR_ROOT}")

  set(_mpc_pkg_pairs
    "mpc_acados_core|${CMAKE_CURRENT_SOURCE_DIR}/mpc_acados_core"
    "mpc_acados_position|${CMAKE_CURRENT_SOURCE_DIR}/controllers/position/mpc_acados_position"
    "mpc_acados_trajectory|${CMAKE_CURRENT_SOURCE_DIR}/controllers/trajectory/mpc_acados_trajectory"
  )

  foreach(_pair IN LISTS _mpc_pkg_pairs)
    string(REPLACE "|" ";" _parts "${_pair}")
    list(GET _parts 0 _name)
    list(GET _parts 1 _src)
    set(_dst "${PYBIND_PY_MIRROR_ROOT}/${_name}")

    # Remove any stale file/dir/symlink at the destination, then create a
    # fresh symlink so reconfigures always pick up the current source path.
    if(EXISTS "${_dst}" OR IS_SYMLINK "${_dst}")
      file(REMOVE_RECURSE "${_dst}")
    endif()
    execute_process(
      COMMAND ${CMAKE_COMMAND} -E create_symlink "${_src}" "${_dst}"
      RESULT_VARIABLE _symlink_rc
    )
    if(NOT _symlink_rc EQUAL 0)
      message(WARNING "mpc: failed to symlink ${_src} -> ${_dst} (rc=${_symlink_rc})")
    else()
      message(STATUS "mpc: mirrored ${_name} -> ${_dst}")
    endif()
  endforeach()
endif()
