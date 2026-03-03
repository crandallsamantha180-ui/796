cmake_minimum_required(VERSION 3.16)

# This line tells the build system exactly where to look for your IMU library
set(EXTRA_COMPONENT_DIRS "components") 

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(lab_2)