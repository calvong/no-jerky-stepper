# Host build of the platform-independent core + unit tests (any C++17 compiler).
#
#   cmake -S test/host -B build-host
#   cmake --build build-host
#   ctest --test-dir build-host --output-on-failure
#   build-host/dump_steps move > steps.csv && python python/plot_steps.py steps.csv
#
# The same tests also run on an (emulated) ESP32-S3 via test/target.
cmake_minimum_required(VERSION 3.16)
project(nj_host_tests CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG v2.6.0)
FetchContent_MakeAvailable(unity)
target_compile_definitions(unity PUBLIC UNITY_INCLUDE_DOUBLE UNITY_SUPPORT_64)

set(ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
add_library(nj_core STATIC
    ${ROOT}/src/motion/poly.cpp
    ${ROOT}/src/motion/min_jerk.cpp
    ${ROOT}/src/motion/cruise.cpp
    ${ROOT}/src/motion/strategy.cpp
    ${ROOT}/src/motion/trajectory.cpp
    ${ROOT}/src/motion/path_planner.cpp
    ${ROOT}/src/stepgen/step_generator.cpp
    ${ROOT}/src/stepgen/step_encoder.cpp
    ${ROOT}/src/core/axis.cpp
    ${ROOT}/src/core/group_planner.cpp)
target_include_directories(nj_core PUBLIC ${ROOT}/include ${ROOT}/src)
if(NOT MSVC)
    target_compile_options(nj_core PRIVATE -Wall -Wextra)
endif()

set(UNIT ${ROOT}/test/unit)
add_executable(nj_tests
    main.cpp
    ${UNIT}/test_runner.cpp
    ${UNIT}/test_motion.cpp
    ${UNIT}/test_stepgen.cpp
    ${UNIT}/test_encoder.cpp
    ${UNIT}/test_retarget.cpp
    ${UNIT}/test_path.cpp
    ${UNIT}/test_group.cpp
    ${UNIT}/test_cruise.cpp)
target_include_directories(nj_tests PRIVATE ${UNIT})
target_link_libraries(nj_tests PRIVATE nj_core unity)

add_executable(dump_steps dump_steps.cpp)
target_link_libraries(dump_steps PRIVATE nj_core)

enable_testing()
add_test(NAME nj_tests COMMAND nj_tests)
