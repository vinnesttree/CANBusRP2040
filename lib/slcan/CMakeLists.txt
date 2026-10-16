# Copyright (c) 2026 CANBusRP2040 contributors
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(slcan.c)
zephyr_library_sources_ifdef(CONFIG_SLCAN_UART slcan_uart.c)
