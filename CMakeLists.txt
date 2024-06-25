# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_world)

target_sources(app PRIVATE
  src/battery.c
  src/buttons.c
  src/bma456.c
  src/bma456_config.c
  src/bhi360.c
  src/bhi360_config.c
  src/mouse.c
)

target_include_directories(app PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/sensor-apis>
)
