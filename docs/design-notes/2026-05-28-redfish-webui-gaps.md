# Redfish & Web UI Feature Gaps

## Overview

The P550 feature parity work added shell commands for power management, SOM
protocol, fan control, boot mode selection, and board identity. These features
are not yet exposed through Redfish or the web dashboard.

## Task 1: Graceful Shutdown/Restart in Redfish

**Files:** `src/redfish.c`

Add `GracefulShutdown` and `GracefulRestart` to the existing
`ComputerSystem.Reset` POST handler (line ~1486). Currently only supports
`On`, `ForceOff`, `PowerCycle`.

```c
} else if (strcmp(payload.reset_type, "GracefulShutdown") == 0) {
    power_graceful_off();
} else if (strcmp(payload.reset_type, "GracefulRestart") == 0) {
    power_graceful_restart();
}
```

Also update `reset_type_values` array (line ~1246) from size 3 to 5 and add
the new values to the GET response.

**Web UI:** Add "Graceful Shutdown" and "Graceful Restart" buttons to the
Host Power Control page alongside existing On/Off/Reset.

## Task 2: SOM Status in Redfish and Web UI

**Files:** `src/redfish.c`, `src/eswin/som_protocol.h`, `static_web_resources/index.html`

Add `Oem.WallaBMC.SomDaemonState` to the `Systems/system` GET response:

```c
// In system_get_handler:
.som_daemon_state = som_is_alive() ? "Online" : "Offline",
```

**Web UI:** Add an ONLINE/OFFLINE badge to the System Overview page. Fetch
from `/redfish/v1/Systems/system` and display `Oem.WallaBMC.SomDaemonState`.

## Task 3: Power Monitoring in Redfish and Web UI

**Files:** `src/redfish.c`, `src/power_monitor.h`, `static_web_resources/index.html`

Add a new Redfish endpoint: `GET /redfish/v1/Chassis/1/Power`

```json
{
    "PowerControl": [{
        "PowerConsumedWatts": 17.5,
        "PowerMetrics": {
            "InputVoltage": 12.05,
            "InputCurrent": 1.45
        }
    }]
}
```

Call `power_monitor_read()` to get voltage/current/power.

**Web UI:** Add a "Power" card to System Overview showing voltage, current,
and watts. Auto-refresh every 5 seconds.

**Note:** INA226 current/power reads 0.000 — likely hardware wiring issue
with the shunt pins. Voltage (12V) works correctly. The Redfish endpoint
should still be implemented; the data will be correct once the hardware
issue is resolved.

## Task 4: Fan Control in Redfish and Web UI

**Files:** `src/redfish.c`, `src/fan.h`, `static_web_resources/index.html`

Add a new Redfish endpoint: `GET/PATCH /redfish/v1/Chassis/1/Thermal`

```json
{
    "Fans": [
        {"Name": "Fan 0", "Reading": 1200, "ReadingUnits": "RPM",
         "Oem": {"DutyCycle": 50}},
        {"Name": "Fan 1", "Reading": 0, "ReadingUnits": "RPM",
         "Oem": {"DutyCycle": 50}}
    ]
}
```

GET calls `fan_get_duty()` and `fan_get_rpm()`. PATCH sets duty via
`fan_set_duty()`.

**Web UI:** Add fan panel with duty cycle sliders (0-100%) and RPM readouts.

**Note:** Fan 0/1 on J18 (SOC fan header) are controlled by the SoC, not
the MCU. MCU PWM outputs go to chassis fan headers. Tachometer reads 0 RPM
because no fan is connected to the MCU's tach pins.

## Task 5: Boot Mode in Redfish and Web UI

**Files:** `src/redfish.c`, `src/eswin/bootsel.h`, `static_web_resources/index.html`

Add boot source to `Systems/system` GET response and PATCH handler:

```json
{
    "Boot": {
        "BootSourceOverrideTarget": "Hdd",
        "BootSourceOverrideMode": "UEFI",
        "Oem": {
            "WallaBMC": {
                "BootSel": 2,
                "BootSelMode": "Hardware",
                "BootSource": "SCPU ROM -> SPI NOR"
            }
        }
    }
}
```

PATCH calls `bootsel_set_sw_mode()` or `bootsel_set_hw_mode()`.

**Web UI:** Add boot mode dropdown to System Overview or a dedicated section.
Show current mode (HW/SW), current BOOT_SEL value, and decoded boot source.

## Implementation Notes

- Redfish patterns: follow existing `REDFISH_HANDLER` macro, `json_obj_descr`
  structs, and `user_data_json_append` for GET responses
- Web UI patterns: follow existing fetch + DOM update pattern in index.html
- All data sources exist in C — just needs HTTP/JSON wiring
- Test with `redfishtool` from a laptop:
  `redfishtool -r <bmc-ip> -vvv Systems -I system`

## INA226 Current Investigation

Voltage reads correctly (12.05V). Current and power read 0.000.

- Calibration register IS written during deferred init (ina2xx_common.c:111)
- CAL = 5120000000 / (1000 × 1000) = 5120 (our DTS)
- Vendor used CAL = 2048 with current_LSB = 2.5mA
- Shunt voltage register reading 0 suggests hardware issue (shunt pins
  not in current path, or shunt resistor not present/connected)
- First reading showed 17.525W power — intermittent, needs investigation
- This is a hardware probing task, not a software fix
