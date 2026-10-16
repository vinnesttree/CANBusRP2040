# Copyright (c) 2026 CANBusRP2040 contributors
# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_SLCAN slcan)
add_subdirectory_ifdef(CONFIG_FRC_CAN frc_can)
add_subdirectory_ifdef(CONFIG_KRAKEN kraken)
add_subdirectory_ifdef(CONFIG_PHOENIX6 phoenix6)
