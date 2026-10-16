# Copyright (c) 2026 CANBusRP2040 contributors
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_CAN_MCP25625)
  zephyr_library()
  zephyr_library_sources(can_mcp25625.c)
  zephyr_library_sources_ifdef(CONFIG_EMUL_MCP25625 can_mcp25625_emul.c)
endif()
