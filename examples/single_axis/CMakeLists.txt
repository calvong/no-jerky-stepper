cmake_minimum_required(VERSION 3.16)
# Use the component from this repository.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../..")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(nj_single_axis)
