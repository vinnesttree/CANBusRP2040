# Copyright (c) 2026 CANBusRP2040 contributors
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(
  phoenix6_client.c
  phoenix6_desc.c
  phoenix6_generated.c
  phoenix6_proto.c
)
