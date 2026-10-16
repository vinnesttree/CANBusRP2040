# Copyright (c) 2026 CANBusRP2040 contributors
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(phoenix6_test)

target_sources(app PRIVATE src/proto.c src/desc.c src/client.c)
# Private protocol header of the library
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/phoenix6)
