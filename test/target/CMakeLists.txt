# On-target / QEMU unit test app. Runs the platform-independent tests in test/unit on an
# (emulated) ESP32-S3:
#   idf.py -C test/target set-target esp32s3 build
#   idf.py -C test/target qemu        (or flash + monitor on hardware)
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../..")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(nj_unit_tests)
