# Copyright (c) 2026 CANBusRP2040 contributors
# SPDX-License-Identifier: Apache-2.0
#
# Runs Zephyr's own CAN controller API test suite (tests/drivers/can/api)
# against the MCP25625 driver and its emulator.

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mcp25625_zephyr_can_api)

set(UPSTREAM ${ZEPHYR_BASE}/tests/drivers/can/api/src)

target_include_directories(app PRIVATE ${UPSTREAM})
target_sources(app PRIVATE ${UPSTREAM}/common.c)
target_sources(app PRIVATE ${UPSTREAM}/classic.c)
target_sources(app PRIVATE ${UPSTREAM}/transceiver.c)
target_sources(app PRIVATE ${UPSTREAM}/utilities.c)
target_sources_ifdef(CONFIG_CAN_FD_MODE app PRIVATE ${UPSTREAM}/canfd.c)
target_sources_ifdef(CONFIG_CAN_STATS app PRIVATE ${UPSTREAM}/stats.c)
target_sources_ifdef(CONFIG_PM_DEVICE app PRIVATE ${UPSTREAM}/powermgmt.c)
