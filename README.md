# CANBusRP2040

A [Zephyr RTOS](https://zephyrproject.org) module for using a **Microchip
MCP25625-E/SS CAN board** with a microcontroller. It was written for the
**RP2040** (Raspberry Pi Pico), but nothing in the libraries is specific to it:
they work on any Zephyr board with SPI and a GPIO interrupt (see
[Board support](#board-support)). It contains:

- a Zephyr CAN controller driver for the MCP25625 (`microchip,mcp25625`), so the
  board works with the standard Zephyr CAN API,
- **CANable-style USB-to-CAN adapter** firmware (the slcan / Lawicel protocol
  over USB CDC ACM), usable from Linux SocketCAN, python-can and similar tools,
- **Kraken X60 / X44** motor support: PWM control, motor data and model, and a
  CAN bus monitor for CTR Electronics devices,
- **every Phoenix 6 control request** of the Talon FX (the Kraken's motor
  controller), plus its configuration and status signals, as a C API on the
  microcontroller. The requests are carried out by CTR Electronics' official
  Phoenix 6 library on a Linux host bridge (see
  [Phoenix 6 from the firmware](#phoenix-6-from-the-firmware)).

> **Hardware status:** the code is tested on Zephyr's `native_sim`: the driver
> against a register-level MCP25625 emulator (including Zephyr's own CAN driver
> conformance suite), and the Phoenix 6 path end to end against the simulated
> Talon FX of the official Phoenix 6 library. All samples build for `rpi_pico`,
> `rpi_pico2` and `nrf52840dk`. It has **not yet been run on real hardware**.
> Please report your results.

## Contents

| Path | What it is |
|---|---|
| `drivers/can/` | MCP25625 CAN driver and its SPI emulator (for tests) |
| `dts/bindings/can/microchip,mcp25625.yaml` | Devicetree binding |
| `boards/shields/mcp25625_pico/` | Devicetree "shield": MCP25625 board wired to a Pico (RP2040 / RP2350) |
| `boards/shields/mcp25625_arduino/` | Devicetree "shield": MCP25625 board on any Arduino R3 header |
| `lib/slcan/` | slcan protocol engine and UART / USB CDC ACM transport |
| `lib/kraken/`, `lib/frc_can/` | Kraken support and FIRST CAN helpers |
| `lib/phoenix6/` | Phoenix 6 API (generated) and the client of the host bridge |
| `host/phoenix6_bridge/` | Host bridge (Python) running the requests with the official Phoenix 6 library |
| `scripts/gen_phoenix6.py` | Regenerates the Phoenix 6 API for another Phoenix 6 version |
| `include/canbus_rp2040/` | Public headers |
| `samples/usb_can_adapter/` | CANable-style USB-to-CAN adapter |
| `samples/phoenix6_bridge/` | Phoenix 6 requests from the firmware, with a `p6` shell |
| `samples/kraken_tool/` | Shell tool: Kraken PWM control, motor data, CAN device monitor |
| `samples/can_basic/` | Minimal send/receive example |
| `tests/` | Driver, slcan, Kraken, Phoenix 6 and Zephyr conformance tests (`native_sim`) |
| `docs/phoenix6_bridge_protocol.md` | Serial protocol between the firmware and the host bridge |

## Kraken support: what is possible where

The Kraken X60 and X44 are driven by CTR Electronics' Talon FX controller.
CTRE's **Phoenix 6 CAN protocol** (control requests, configuration, status
signals, FOC, Motion Magic, ...) **is not publicly documented**, so this project
does not reimplement it: a guessed protocol could command a motor in ways you
did not intend. Instead, there are three paths:

| Feature | Phoenix 6 program on a Linux host, this board as USB-to-CAN adapter | Phoenix 6 API in the firmware, through the host bridge | Standalone on the microcontroller |
|---|---|---|---|
| Every Phoenix 6 control request (duty cycle, voltage, torque/FOC, position, velocity, Motion Magic, followers, ...) | ✅ | ✅ (`phoenix6_set_control()`) | ❌ (protocol undocumented) |
| Configuration (PID slots, current limits, neutral mode, inversion, soft limits, ...) | ✅ | ✅ (`phoenix6_config_set()`) | ❌ |
| Status signals (position, velocity, current, temperature, faults, ...) | ✅ | ✅ (`phoenix6_subscribe()`) | ❌ |
| Open-loop speed control | ✅ | ✅ | ✅ PWM (`kraken_pwm`) |
| List Kraken / CTRE devices on the bus by CAN ID | ✅ (Phoenix Tuner X) | – | ✅ CAN monitor (`kraken_can_monitor`) |
| Robot controller (roboRIO) heartbeat decoding | – | – | ✅ (`frc_can`) |
| Motor data, DC motor model, unit conversions | – | – | ✅ (`kraken_motor`) |
| Needs a Linux host | yes | yes | no |

Phoenix 6 supports Linux SocketCAN adapters for non-FRC use; the
`usb_can_adapter` and `phoenix6_bridge` firmware provide one (see below).

## Hardware

### Wiring (`mcp25625_pico` shield)

| MCP25625 board | Raspberry Pi Pico |
|---|---|
| SCK | GP18 (SPI0 SCK) |
| SI (MOSI) | GP19 (SPI0 TX) |
| SO (MISO) | GP16 (SPI0 RX) |
| CS | GP17 |
| INT | GP20 |
| STBY | GP21 (optional) |
| VIO | 3V3(OUT) |
| VDD | VBUS (5 V) |
| VSS | GND |

- **VIO must be 3.3 V.** VIO sets the MCP25625 logic level and the RP2040 is
  not 5 V tolerant. A board that ties VIO to VDD (5 V) cannot be connected
  directly.
- **VDD** powers the integrated CAN transceiver and needs 5 V.
- **STBY**: the driver keeps the transceiver in standby (STBY high) while the
  controller is stopped. If STBY is tied low on your board, remove
  `standby-gpios` (see below).
- **Crystal:** the shield assumes 16 MHz. The FRC/Kraken bus runs at
  **1 Mbit/s, which needs a crystal of at least 16 MHz** (8 MHz is limited to
  500 kbit/s).
- **Termination:** a CAN bus needs 120 Ω at both ends. Check whether your board
  has a termination jumper.

Pins and the crystal frequency can be changed with an extra overlay, e.g.
`my-board.overlay`:

```dts
&mcp25625_pico {
	osc-freq = <20000000>;
	/delete-property/ standby-gpios;
};
```

and building with `-DEXTRA_DTC_OVERLAY_FILE=my-board.overlay`.

## Getting started

1. Install the Zephyr SDK and its host dependencies by following the
   [Zephyr getting started guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)
   (Python requirements, `west`, Zephyr SDK with the ARM toolchain).
2. Create a workspace from this repository. `west.yml` pins **Zephyr v4.4.2**
   and fetches only the modules needed for the Pico:

   ```sh
   west init -m https://github.com/vinnesttree/CANBusRP2040 --mr <branch> canbus-ws
   cd canbus-ws
   west update
   pip install -r zephyr/scripts/requirements.txt
   ```

3. Build a sample (`rpi_pico` board, `mcp25625_pico` shield):

   ```sh
   west build -b rpi_pico canbus-rp2040/samples/usb_can_adapter -- -DSHIELD=mcp25625_pico
   ```

   For another board, use its name and shield, e.g.
   `west build -b nrf52840dk/nrf52840 ... -- -DSHIELD=mcp25625_arduino`, and
   fetch its HAL first (see [Board support](#board-support)).

4. Flash: hold BOOTSEL while plugging in the Pico and copy
   `build/zephyr/zephyr.uf2` to the `RPI-RP2` drive (or use `west flash` with a
   debug probe).

Log output of all samples goes to the board's console UART (on a Pico: UART0,
GP0 TX / GP1 RX, 115200 baud).

## Using the library in your own project

The repository is a self-contained Zephyr module: drivers, devicetree binding,
shields, libraries and headers are picked up automatically once Zephyr knows
about it, and every feature is switched on with one Kconfig option. It is
tested with **Zephyr v4.4.2**.

### 1. Add the module

Any of these works:

- **West manifest** of your workspace (`west update` fetches it):

  ```yaml
  manifest:
    projects:
      - name: canbus-rp2040
        url: https://github.com/vinnesttree/CANBusRP2040
        revision: <commit or tag>
        path: modules/lib/canbus-rp2040
  ```

- **Copied or as a git submodule** inside your application, with this line in
  the application's `CMakeLists.txt` *before* `find_package(Zephyr ...)`:

  ```cmake
  list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/modules/CANBusRP2040)
  ```

- **On the command line**: `west build ... -- -DZEPHYR_EXTRA_MODULES=/path/to/CANBusRP2040`.

### 2. Describe the wiring in devicetree

Use a shield if your board has a matching connector:

| Shield | Boards | Pins |
|---|---|---|
| `mcp25625_pico` | Raspberry Pi Pico family (`pico_header`, e.g. `rpi_pico`, `rpi_pico2`) | [Wiring](#wiring-mcp25625_pico-shield) |
| `mcp25625_arduino` | Any board with an Arduino R3 header (`arduino_header` / `arduino_spi`): most Nordic DKs, STM32 Nucleo, NXP FRDM, ... | SPI on D11–D13, CS D10, INT D2, STBY D9 |

and build with `-DSHIELD=<shield>`. For any other wiring, add the node to your
board overlay; only an SPI bus, a chip select and an interrupt-capable GPIO are
needed:

```dts
&spi1 {                                   /* any SPI controller */
	status = "okay";
	cs-gpios = <&gpio0 17 GPIO_ACTIVE_LOW>;

	mcp25625: can@0 {
		compatible = "microchip,mcp25625";
		reg = <0>;
		spi-max-frequency = <DT_FREQ_M(10)>;
		osc-freq = <DT_FREQ_M(16)>;       /* crystal on the board */
		int-gpios = <&gpio0 20 GPIO_ACTIVE_LOW>;
		standby-gpios = <&gpio0 21 GPIO_ACTIVE_HIGH>;  /* optional */
		bitrate = <1000000>;
	};
};

/ {
	chosen {
		zephyr,canbus = &mcp25625;
	};
};
```

The SPI controller also needs its pins (`pinctrl-0`) if the board does not set
them up already.

### 3. Enable what you need

| Feature | `prj.conf` | Header |
|---|---|---|
| MCP25625 driver | `CONFIG_CAN=y` (the driver is enabled by the devicetree node) | `<zephyr/drivers/can.h>`, extras in `<canbus_rp2040/drivers/can/mcp25625.h>` |
| slcan USB-to-CAN | `CONFIG_SLCAN=y` | `<canbus_rp2040/slcan.h>` |
| Kraken PWM, motor model, CAN monitor | `CONFIG_KRAKEN=y` (+ `CONFIG_PWM=y` for PWM) | `<canbus_rp2040/kraken.h>` |
| Phoenix 6 through the host bridge | `CONFIG_PHOENIX6=y` | `<canbus_rp2040/phoenix6.h>` |
| FIRST CAN identifiers and heartbeat | `CONFIG_FRC_CAN=y` | `<canbus_rp2040/frc_can.h>` |

The serial transports select the UART interrupt API they need; only
`CONFIG_SERIAL=y` must be set, which most boards do by default. The libraries
take their devices as arguments (`const struct device *`), so they work with any
CAN controller and any UART or USB CDC ACM port, not only the MCP25625 and USB.

A minimal application, bridging the MCP25625 to a serial port with slcan:

```c
#include <zephyr/drivers/can.h>
#include <canbus_rp2040/slcan.h>

int main(void)
{
	/* my_uart: any UART node, or a USB CDC ACM port as in samples/usb_can_adapter */
	return slcan_uart_start(DEVICE_DT_GET(DT_NODELABEL(my_uart)),
				DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus)), NULL, NULL);
}
```

`slcan_uart_start()` can run several instances at once (one CAN controller per
port, `CONFIG_SLCAN_UART_INSTANCES`), e.g. for a board with two CAN buses.

## Board support

Nothing in the drivers or libraries uses RP2040-specific code; they only use
Zephyr's devicetree and driver APIs (SPI, GPIO, CAN, UART, PWM). Verified:

| Board | SoC | How |
|---|---|---|
| `rpi_pico` | RP2040 (Cortex-M0+) | All samples build (`mcp25625_pico`) |
| `rpi_pico2/rp2350a/m33` | RP2350 (Cortex-M33) | All samples build (`mcp25625_pico`) |
| `nrf52840dk/nrf52840` | nRF52840 (Cortex-M4F) | All samples build (`mcp25625_arduino`) |
| `native_sim` | host | All tests run (SPI / GPIO / UART emulators) |

What a board needs:

| Feature | Requirement |
|---|---|
| MCP25625 driver | SPI controller (up to 10 MHz), a GPIO with edge interrupts for INT |
| slcan, Phoenix 6 bridge link | A UART with the interrupt driven API, or a USB device controller (CDC ACM) |
| USB samples | A USB device controller supported by Zephyr's `device_next` stack (`zephyr_udc0`) |
| Kraken PWM | A PWM channel with 1 µs resolution, 5 ms period |

To use another board:

1. Fetch its HAL. The workspace manifest (`west.yml`) only fetches the Raspberry
   Pi HAL; add yours to `name-allowlist` (e.g. `hal_nordic`, `hal_stm32`,
   `hal_nxp`) and run `west update`, or use the module from your own workspace.
2. Use a shield or write the devicetree node as shown above.
3. For `samples/kraken_tool`, add a `boards/<board>.overlay` with the PWM channel
   (see `samples/kraken_tool/boards/rpi_pico.overlay`); the sample builds
   without PWM otherwise.

The `canbus_rp2040` name of the module, the header directory and the Kconfig
menu is historical; it does not limit the supported boards.

## Using the MCP25625 driver in your application

The driver implements the standard
[Zephyr CAN API](https://docs.zephyrproject.org/latest/hardware/peripherals/can/controller.html),
so any Zephyr CAN code works unchanged:

```c
const struct device *can = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
struct can_frame frame = {.id = 0x123, .dlc = 2, .data = {0xDE, 0xAD}};

can_start(can);
can_send(can, &frame, K_MSEC(100), NULL, NULL);
```

See `samples/can_basic` for a complete example.

### Driver features

- Normal, listen-only and loopback modes; one-shot (no automatic retransmission)
  and triple sampling (`CAN_MODE_ONE_SHOT`, `CAN_MODE_3_SAMPLES`).
- Bit timing from the oscillator frequency in devicetree, with the MCP2515
  family timing constraints checked.
- STBY (transceiver standby) and RESET pin control; generic `phys` transceivers
  are also supported.
- Interrupt handling that reads both receive buffers in arrival order (receive
  buffer 0 rolls over into buffer 1).
- Error state change callbacks including recovery (polled, because the chip does
  not interrupt on recovery), pending transmission completed with
  `-ENETUNREACH` on bus-off, RX overrun accounting (`CONFIG_CAN_STATS`).
- Optional hardware acceptance filters to reduce SPI traffic on busy buses
  (`mcp25625_set_hw_acceptance()`), driver counters (`mcp25625_get_stats()`),
  see `include/canbus_rp2040/drivers/can/mcp25625.h`.
- Device power management (sleep mode) with `CONFIG_PM_DEVICE`.

### Limitations

- Classic CAN only (the MCP25625 has no CAN FD).
- A single transmit buffer is used, so frames go out in the order they were
  queued (like Zephyr's MCP2515 driver).
- Bus-off recovery is always automatic (`CAN_MODE_MANUAL_RECOVERY` is not
  supported by the chip).
- No hardware RX timestamps.
- The chip has only two receive buffers. At high bus loads (a Kraken bus at
  1 Mbit/s) the interrupt thread must be serviced quickly; keep
  `CONFIG_CAN_MCP25625_INT_THREAD_PRIO` high and the SPI clock at 10 MHz.
  Throughput has not been measured on hardware yet.

Kconfig options: `CONFIG_CAN_MCP25625_INT_THREAD_STACK_SIZE`,
`CONFIG_CAN_MCP25625_INT_THREAD_PRIO`, `CONFIG_CAN_MCP25625_MAX_FILTERS`,
`CONFIG_CAN_MCP25625_STATE_POLL_INTERVAL_MS`.

Zephyr's in-tree `microchip,mcp2515` driver can also drive the MCP25625
(controlling STBY through a `can-transceiver-gpio` node); the libraries in this
repository work with either driver, or with any other Zephyr CAN controller.

## USB-to-CAN adapter (CANable-style)

`samples/usb_can_adapter` turns the Pico into a USB-to-CAN adapter that speaks
the slcan protocol of the CANable (slcan firmware) and Lawicel adapters on a USB
CDC ACM port.

### Linux SocketCAN

```sh
# -s8 = 1 Mbit/s, -o = open the channel, -c = close it on exit
sudo slcand -o -c -s8 /dev/ttyACM0 slcan0
sudo ip link set up slcan0

candump slcan0
cansend slcan0 123#DEADBEEF
```

### python-can

```python
import can

bus = can.Bus(interface="slcan", channel="/dev/ttyACM0", bitrate=1000000)
bus.send(can.Message(arbitration_id=0x123, data=[1, 2, 3], is_extended_id=False))
print(bus.recv(1.0))
```

### Protocol

| Command | Meaning |
|---|---|
| `O` / `L` / `C` | Open (normal) / open listen-only / close |
| `S0`…`S8` | 10k, 20k, 50k, 100k, 125k, 250k, 500k, 800k, 1M bit/s |
| `sxxyy` | Bit timing from SJA1000 BTR0/BTR1 (16 MHz), converted to a bitrate |
| `M0` / `M1` | CANable: normal / silent mode |
| `A0` / `A1` | CANable: automatic retransmission off / on |
| `Mxxxxxxxx` / `mxxxxxxxx` | Lawicel acceptance code / mask (only "accept all") |
| `Z0` / `Z1` | Receive timestamps off / on (ms, 0–59999) |
| `t` `T` `r` `R` | Transmit standard / extended data / remote frame |
| `F` | Read and clear status flags |
| `V` / `N` | Version / serial number |

Differences from the CANable firmware, mostly following the Lawicel protocol
and the Linux slcan driver:

- Commands are acknowledged with CR / BEL (`CONFIG_SLCAN_CMD_ACK`, disable to
  behave exactly like the CANable, which sends no replies).
- `S7` is 800 kbit/s as in the Lawicel protocol and Linux (the CANable README
  lists 750 kbit/s).
- Error state changes are reported as `s<state><rx errors><tx errors>` and
  lost frames as `e1o`, the formats parsed by the Linux slcan driver
  (`CONFIG_SLCAN_STATE_REPORTS`, `CONFIG_SLCAN_ERROR_REPORTS`).
- No CAN FD commands (the MCP25625 is a classic CAN controller).

LED (`led0`, GP25 on the Pico): short blink every second while the channel is
closed, on while open with a short off-blink for every frame, fast blinking for
two seconds after an error.

The adapter uses the Zephyr project's test USB VID/PID by default; set
`CONFIG_CDC_ACM_SERIAL_VID` / `CONFIG_CDC_ACM_SERIAL_PID` to your own IDs for
devices you distribute. The protocol engine can be reused with any transport
(`include/canbus_rp2040/slcan.h`).

## Phoenix 6 from the firmware

The `phoenix6` library makes every Talon FX control request of Phoenix 6 a C
struct with Phoenix 6's parameter names and defaults, together with the
configuration fields and status signals. Because CTRE does not publish the CAN
protocol, the firmware does not put the requests on the CAN bus itself: it sends
them over a USB serial port to the host bridge, which carries them out with CTR
Electronics' official Phoenix 6 library and streams status signals back. The
board can be the bridge's SocketCAN adapter at the same time:

```text
 microcontroller                                 Linux host (PC, Raspberry Pi, Jetson)
┌───────────────────────────┐   USB port 2   ┌──────────────────────────────────┐
│ your code                 │ ─────────────► │ phoenix6_bridge                  │
│  phoenix6_set_control()   │ ◄───────────── │  official Phoenix 6 library      │
│  phoenix6_get_signal()    │  results,      │        │ SocketCAN (slcan0)      │
│                           │  signals       │        ▼                         │
│ slcan        ◄──────────────── USB port 1 ──── slcand                         │
│ MCP25625                  │                └──────────────────────────────────┘
└──┬────────────────────────┘
   │ CAN, 1 Mbit/s
 Kraken / Talon FX
```

Any other SocketCAN adapter (or a CANivore) works as well; the bridge only needs
a serial link to the firmware.

### Requests in C

```c
#include <canbus_rp2040/phoenix6.h>

phoenix6_bridge_start(DEVICE_DT_GET(DT_NODELABEL(phoenix6_uart)));
phoenix6_bridge_wait_connected(K_SECONDS(10));

/* Configuration: one field, the rest of its group is kept */
phoenix6_config_set(1, PHOENIX6_CONFIG_SLOT0_K_V, 0.12, 100, NULL);
phoenix6_config_set(1, PHOENIX6_CONFIG_MOTOR_OUTPUT_NEUTRAL_MODE,
		    PHOENIX6_NEUTRAL_MODE_BRAKE, 100, NULL);

/* Status signals streamed at 50 Hz */
phoenix6_subscribe(1, PHOENIX6_SIGNAL_VELOCITY, 50.0f, NULL);

/* Velocity closed loop, 10 rot/s, Phoenix 6 defaults for everything else */
struct phoenix6_control req = PHOENIX6_VELOCITY_VOLTAGE(10.0);
req.velocity_voltage.slot = 0;

while (true) {
	struct phoenix6_signal_value v;

	phoenix6_feed_enable(100);          /* actuators run only while fed */
	phoenix6_set_control(1, &req);      /* Talon FX with CAN ID 1 */
	if (phoenix6_get_signal(1, PHOENIX6_SIGNAL_VELOCITY, &v) == 0) {
		printk("velocity %.2f rot/s\n", v.value);
	}
	k_msleep(20);
}
```

- Every request has `PHOENIX6_<NAME>(required...)`, filling in the Phoenix 6
  defaults, and a struct `struct phoenix6_<name>` in
  `include/canbus_rp2040/phoenix6_generated.h`. Differential requests
  (`Diff_*`) take the parameter structs of their two requests, e.g.
  `PHOENIX6_DIFF_VELOCITY_VOLTAGE_POSITION(PHOENIX6_VELOCITY_VOLTAGE_PARAMS(5.0), PHOENIX6_POSITION_VOLTAGE_PARAMS(0.25))`.
- Also available: `phoenix6_config_get()`, `phoenix6_factory_default()`,
  `phoenix6_set_position()`, `phoenix6_clear_sticky_faults()`,
  `phoenix6_last_control_status()` (the Phoenix 6 `StatusCode` of the last
  request), and descriptor tables to look everything up by name.
- Call `phoenix6_set_control()` periodically, as in the example: after a link
  loss the bridge has put the motors in neutral, and only a new request moves
  them again.

### Running it

1. Flash `samples/phoenix6_bridge` (`-DSHIELD=mcp25625_pico` or
   `mcp25625_arduino`). The board shows up as two serial ports.
2. On the host, create the SocketCAN interface on the first port:
   `sudo slcand -o -c -s8 /dev/ttyACM0 slcan0 && sudo ip link set up slcan0`.
3. Install and start the bridge on the second port:

   ```sh
   pip install ./host/phoenix6_bridge          # installs phoenix6==26.3.0 and pyserial
   phoenix6-bridge --port /dev/ttyACM1 --canbus slcan0 -v
   ```

4. Use the `p6` shell on the console UART, or your own code:

   ```text
   uart:~$ p6 status
   uart:~$ p6 enable on
   uart:~$ p6 info VelocityVoltage
   uart:~$ p6 control 1 VelocityVoltage velocity=10 slot=1
   uart:~$ p6 control 1 Follower leader_id=2 motor_alignment=OPPOSED
   uart:~$ p6 config set 1 motor_output.neutral_mode BRAKE
   uart:~$ p6 signal 1 velocity 50
   uart:~$ p6 stop 1
   uart:~$ p6 list controls
   ```

### What to know

- **Versions:** the firmware is generated from the Phoenix 6 version installed
  on the host (26.3.0). At connection time the bridge compares a hash of the
  API, and refuses to start if `schema.json` does not match its installed
  `phoenix6`. For another version: install it, run
  `python -m phoenix6_bridge.schema` (in `host/phoenix6_bridge`) and
  `scripts/gen_phoenix6.py`, and rebuild the firmware.
- **Licensing and bus type:** 66 of the 99 requests require a Phoenix Pro
  license and 60 of them also a CANivore (CAN FD), as stated by Phoenix 6
  (`p6 list controls` shows both). The MCP25625 is a CAN 2.0 controller, so the
  CANivore-only requests (the `Diff_*` and `DynamicMotionMagic*` requests)
  cannot be used on a bus attached to it. `enable_foc` defaults to true as in
  Phoenix 6; FOC also needs Phoenix Pro.
- **Safety:** the firmware and the bridge exchange PING / PONG every 100 ms.
  If nothing arrives for 500 ms, or the firmware reconnects, or the bridge
  stops (Ctrl+C or SIGTERM), the bridge sends `NeutralOut` to every Talon FX it
  used and disables actuators (`feed_enable(0)`). Actuators also only run while
  the firmware feeds the enable (`phoenix6_feed_enable()`).
- **Bandwidth:** use USB if you can. A `VelocityVoltage` request is 53 bytes
  on the wire, so a hardware UART at 115200 baud (11.5 kB/s) carries about 200
  per second in total, e.g. four motors at 50 Hz, before PINGs and enable.
- **Latency** from `phoenix6_set_control()` to the CAN bus has not been
  measured on hardware yet; it includes two USB transfers and the host.
- The serial protocol is documented in
  [docs/phoenix6_bridge_protocol.md](docs/phoenix6_bridge_protocol.md).

## Kraken: Phoenix 6 programs on a Linux host

1. Flash `samples/usb_can_adapter` and connect the MCP25625 CAN-H / CAN-L to
   the Kraken's CAN wires (yellow = CAN-H, green = CAN-L). Terminate the bus.
2. Create the SocketCAN interface at 1 Mbit/s:
   `sudo slcand -o -c -s8 /dev/ttyACM0 slcan0 && sudo ip link set up slcan0`.
3. Install CTRE's Phoenix 6 for non-FRC Linux (for Python: `pip install phoenix6`)
   and pass the SocketCAN interface name as the CAN bus.

An illustrative Python sketch (check
[CTRE's Phoenix 6 documentation](https://v6.docs.ctr-electronics.com) for the
API of your Phoenix 6 version):

```python
import time
from phoenix6 import CANBus, controls, hardware, unmanaged

kraken = hardware.TalonFX(1, CANBus("slcan0"))  # device ID 1 on SocketCAN slcan0
request = controls.DutyCycleOut(0.1)

while True:
    unmanaged.feed_enable(0.100)                 # non-FRC enable signal
    kraken.set_control(request)
    print(kraken.get_velocity().value)           # rotations per second
    time.sleep(0.02)
```

Notes from CTRE's documentation:

- Outside FRC, actuators only run while the application feeds the enable
  signal (`feed_enable`).
- Devices that were used on an FRC robot can be FRC-locked and will not enable
  without a roboRIO; see CTRE's "Enabling Actuators" documentation.
- FOC commutation requires a Phoenix Pro license.

## Kraken: standalone on the microcontroller (`samples/kraken_tool`)

```sh
west build -b rpi_pico canbus-rp2040/samples/kraken_tool -- -DSHIELD=mcp25625_pico
```

The sample provides a `kraken` shell command on the console UART:

```text
uart:~$ kraken spec x60 foc
uart:~$ kraken pwm neutral
uart:~$ kraken pwm set 0.25
uart:~$ kraken pwm off
uart:~$ kraken can start
uart:~$ kraken can devices
uart:~$ kraken can heartbeat
```

The generic Zephyr `can` shell commands are enabled as well.

### PWM control

The Talon FX detects CAN or PWM control automatically on the same two wires.
CTRE documents the PWM signal on CAN-H (yellow) and ground on CAN-L (green), a
1–2 ms pulse and a 2.9–100 ms period. The sample outputs PWM on **GP15**:

- GP15 → Kraken yellow wire, Pico GND → Kraken green wire.
- Use a motor that is **not** on the CAN bus at the same time.
- The RP2040 drives 3.3 V logic. The public documentation consulted for this
  project does not state the Talon FX PWM input threshold; verify that 3.3 V is
  recognised or use a 5 V buffer (e.g. 74AHCT125).
- Pulses follow WPILib's `PWMTalonFX`: 2.004 ms full forward, 1.520 / 1.480 ms
  deadband edges, 1.500 ms neutral, 0.997 ms full reverse. The period is
  `CONFIG_KRAKEN_PWM_PERIOD_US` (5 ms).
- The tool starts with no pulses (motor controller disabled); `kraken pwm off`
  stops the pulses again.

API: `kraken_pwm_init()`, `kraken_pwm_set()`, `kraken_pwm_neutral()`,
`kraken_pwm_disable()`, `kraken_pwm_set_inverted()`.

### CAN bus monitor

`kraken_can_monitor_start()` lists every CTR Electronics device that transmits
on the bus by FRC device type and device number (the CAN ID configured in
Phoenix Tuner X), together with the API identifiers it uses, and decodes the
robot controller heartbeat. Under the FIRST CAN specification motor controllers
use device type 2, so a Kraken cannot be told apart from a Falcon or another
CTRE motor controller by its identifier alone. Frame payloads are not decoded.

### Motor data and model

`kraken_motor_spec_get()` returns the published data at 12 V, as used by
WPILib's `DCMotor`:

| Motor | Free speed | Stall torque | Stall current | Free current |
|---|---|---|---|---|
| Kraken X60 | 6000 RPM | 7.09 N·m | 366 A | 2 A |
| Kraken X60 (FOC) | 5800 RPM | 9.37 N·m | 483 A | 2 A |
| Kraken X44 | 7758 RPM | 4.11 N·m | 279 A | 2 A |
| Kraken X44 (FOC) | 7368 RPM | 5.01 N·m | 329 A | 2 A |

`kraken_motor_model_init()` derives resistance, Kv and Kt; helpers compute
current, torque, voltage and speed, and convert between RPM, rad/s and rotations
(the Phoenix 6 position unit).

## FIRST CAN helpers

`include/canbus_rp2040/frc_can.h` implements the public
[FIRST CAN device specification](https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html):
29-bit identifier packing (device type, manufacturer, API class/index, device
number), the manufacturer and device type codes, the broadcast Disable frame and
the universal heartbeat (`0x01011840`).

## Tests

All tests run on Zephyr's `native_sim`. The driver tests use an emulator of
the MCP25625 register set and SPI instruction set (modes, TX/RX buffers with
rollover, filters and masks, interrupt and error flags, INT pin).

```sh
west twister -T canbus-rp2040/tests -p native_sim/native/64
```

| Test | Coverage |
|---|---|
| `tests/drivers/can/mcp25625` | Driver behaviour against the emulator (19 tests) |
| `tests/drivers/can/zephyr_api` | Zephyr's own CAN controller API test suite, run against this driver |
| `tests/drivers/can/zephyr_timing` | Zephyr's CAN timing test suite |
| `tests/lib/slcan` | Protocol parser/formatter and command handling over the driver |
| `tests/lib/slcan_uart` | UART transport end to end (UART emulator ↔ driver emulator) |
| `tests/lib/kraken` | FRC CAN helpers, heartbeat bit layout, motor model, PWM, bus monitor |
| `tests/lib/phoenix6` | Protocol vectors, generated tables and initializers, client against a fake host |
| `tests/e2e/phoenix6_sim` | Firmware client ↔ real host bridge ↔ simulated Talon FX of Phoenix 6 |
| `host/phoenix6_bridge/tests` | Protocol, schema vs. installed `phoenix6`, every request / config field / signal through the bridge in Phoenix 6 simulation |

The Phoenix 6 tests need `pip install phoenix6==26.3.0 pyserial`:

```sh
# host bridge
cd canbus-rp2040/host/phoenix6_bridge && python -m unittest discover -s tests -t .
# end to end
west build -b native_sim/native/64 canbus-rp2040/tests/e2e/phoenix6_sim -d build/e2e
python canbus-rp2040/scripts/phoenix6_e2e.py build/e2e/zephyr/zephyr.exe
```

The emulator reproduces the documented MCP2515/MCP25625 behaviour, and the
Phoenix 6 simulation is CTRE's; neither is a substitute for testing on real
hardware.

## References

- [MCP25625 product page (Microchip)](https://www.microchip.com/en-us/product/MCP25625)
- [Zephyr CAN controller API](https://docs.zephyrproject.org/latest/hardware/peripherals/can/controller.html)
  and Zephyr's in-tree MCP2515 driver
- [CANable slcan firmware](https://github.com/normaldotcom/canable-fw),
  [CANable 2.0 firmware](https://github.com/normaldotcom/canable2-fw),
  Linux `drivers/net/can/slcan`
- [FIRST CAN device specification](https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html)
- WPILib `PWMTalonFX` and `DCMotor`
- [CTRE Phoenix 6 documentation](https://v6.docs.ctr-electronics.com): non-FRC
  installation, enabling actuators, Talon FX hardware reference; the
  [`phoenix6` Python package](https://pypi.org/project/phoenix6/) is the source
  of the generated API (names, parameter order, types and defaults only)
- [WestCoast Products Kraken documentation](https://docs.wcproducts.com)

## License

Apache-2.0, see [LICENSE](LICENSE).
