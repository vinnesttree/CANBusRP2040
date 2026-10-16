# Copyright (c) 2026 CANBusRP2040 contributors
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(kraken_motor.c)
zephyr_library_sources_ifdef(CONFIG_KRAKEN_PWM kraken_pwm.c)
zephyr_library_sources_ifdef(CONFIG_KRAKEN_CAN_MONITOR kraken_can_monitor.c)
