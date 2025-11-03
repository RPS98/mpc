# Tests CMakeLists.txt

# Test
file(GLOB TEST_SOURCE "*_test.cpp")

foreach(TEST_FILE ${TEST_SOURCE})
  get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

  add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
  target_link_libraries(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_NAME})
endforeach()

# Google Test tests
file(GLOB_RECURSE GTEST_SOURCE "*_gtest.cpp")

if(GTEST_SOURCE)
  find_package(GTest REQUIRED)

  foreach(TEST_FILE ${GTEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} GTest::gtest ${PROJECT_NAME})

    add_test(
      NAME ${PROJECT_NAME}_${TEST_NAME}
      COMMAND ${PROJECT_NAME}_${TEST_NAME}
    )
  endforeach()

  # Coverage tests
  if(BUILD_DEVELOPER_TESTS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
    find_program(LCOV lcov REQUIRED)
    find_program(GENHTML genhtml REQUIRED)

    # Generate coverage script
    set(COVERAGE_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/coverage_script.sh")
    file(WRITE ${COVERAGE_SCRIPT} "#!/bin/bash\n")
    file(APPEND ${COVERAGE_SCRIPT} "${LCOV} --directory . --capture --output-file coverage.info\n")
    file(APPEND ${COVERAGE_SCRIPT} "${LCOV} --remove coverage.info '/usr/*' '*/_deps/*' '${CMAKE_CURRENT_BINARY_DIR}/*' --output-file coverage.info\n")
    file(APPEND ${COVERAGE_SCRIPT} "${LCOV} --list coverage.info\n")
    file(APPEND ${COVERAGE_SCRIPT} "${GENHTML} coverage.info --output-directory coverage\n")
    file(APPEND ${COVERAGE_SCRIPT} "echo 'Coverage tests completed. Open file in ${CMAKE_CURRENT_BINARY_DIR}/coverage/index.html'\n")
    execute_process(COMMAND chmod +x ${COVERAGE_SCRIPT})

    # Add script to CTest
    add_test(NAME RunCoverageScript COMMAND ${COVERAGE_SCRIPT})
  endif()
endif()

if(BUILD_DEVELOPER_TESTS)
  file(GLOB_RECURSE ALL_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../include/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../include/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../src/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../tests/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../tests/*.cpp"
  )

  # CppLint tests
  add_test(
    NAME RunCppLint
    COMMAND cpplint ${ALL_SOURCE_FILES} --counting=detailed --recursive
  )

  # ClangFormat tests
  foreach(SOURCE_FILE ${ALL_SOURCE_FILES})
    add_test(NAME clang-format-${SOURCE_FILE}
      COMMAND clang-format -output-replacements-xml -style=file ${SOURCE_FILE})
    set_tests_properties(clang-format-${SOURCE_FILE} PROPERTIES
      FAIL_REGULAR_EXPRESSION "<replacement "
      PASS_REGULAR_EXPRESSION "")
  endforeach()

  # Google Benchmark tests
  file(GLOB_RECURSE BTEST_SOURCE "*_benchmark.cpp")

  if(BTEST_SOURCE)
    find_package(benchmark REQUIRED)

    foreach(TEST_FILE ${BTEST_SOURCE})
      get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

      add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
      target_link_libraries(${PROJECT_NAME}_${TEST_NAME} benchmark::benchmark ${PROJECT_NAME})
    endforeach()
  endif()
endif()
