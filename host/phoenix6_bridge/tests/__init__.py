# Copyright (c) 2026 CANBusRP2040 contributors
# SPDX-License-Identifier: Apache-2.0
import os

# Use the simulated devices of the Phoenix 6 library; must be set before
# phoenix6 is imported.
os.environ.setdefault("CTR_TARGET", "Simulation")
