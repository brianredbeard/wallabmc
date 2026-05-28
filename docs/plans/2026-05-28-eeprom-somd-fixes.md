# EEPROM board_info and somd Command Handling Fix Plan

Created: 2026-05-28
Author: redbeard@dead-city.org
Status: COMPLETE
Approved: Yes
Iterations: 0
Worktree: No
Type: Bugfix

## Summary

**Symptom:** (1) `board_info` returns "Board identity not available" — EEPROM on I2C1 cannot be read. (2) `power restart`/`power shutdown` fail because somd replies "unsupported" (result=3) to CMD_RESTART and CMD_POWER_OFF.

**Trigger:** (1) Any invocation of `board_info` shell command. (2) Running `power restart` or `power shutdown` with somd running on the host.

**Root Cause:**

1. `src/board_identity.c:99` — `device_is_ready(eeprom_dev)` returns false because the Zephyr `at24` EEPROM driver probes I2C1 during kernel init, before `board_identity_init()` enables the I2C mux GPIO (PA3). The I2C1 bus is shared between MCU, SoC, and FT4232H via a TMUX1574 mux (U77). Without PA3 driven, the MCU doesn't own the bus and the EEPROM probe gets no ACK. Same pattern as the INA226 fix — the device needs `zephyr,deferred-init` so the driver doesn't probe until the mux is ready.

2. `host/somd/somd.c:240-261` — `handle_request()` only has cases for `HFP_CMD_BOARD_STATUS` and `HFP_CMD_PVT_INFO`. CMD_RESTART (0x08) and CMD_POWER_OFF (0x01) hit the `default` case which replies UNSUPPORTED. The host daemon needs to handle these by triggering the appropriate system action (`reboot`/`poweroff`).

## Investigation

- The vendor firmware (`es_check_carrier_board_info` in the ELF) reads EEPROM at I2C address 0xA0 (7-bit 0x50), 51 bytes from offset 0, with backup at offset 80. It acquires `gEEPROM_Mutex` but does NOT toggle PA3 per-access — the mux is set during GPIO init at startup.
- WallaBMC's `board_identity_init()` enables the mux, but by that time the at24 driver has already failed its init probe. `device_is_ready()` is permanently false after a failed init.
- The INA226 had the identical problem: device on a power domain not available at boot. Fixed with `zephyr,deferred-init` + lazy `device_init()`. Same approach applies here.
- somd's CMD_RESTART/CMD_POWER_OFF gap was observed on hardware: BMC sends CMD_RESTART (0x08) → somd replies result=3 → BMC falls back to "No reset GPIO" (now fixed, but graceful path still fails).

## Fix Approach

**Chosen:** Deferred init for EEPROM + somd command handlers

**Why:** Matches the proven INA226 fix pattern for EEPROM. For somd, straightforward additions to the existing switch statement with `reboot()` and `poweroff()` syscalls.

**Files:**
- `boards/sifive/hifive_premier_p550_mcu/hifive_premier_p550_mcu.dts` — add `zephyr,deferred-init` to EEPROM node
- `src/board_identity.c` — add lazy `device_init()` before `device_is_ready()`, enable I2C mux before init
- `host/somd/somd.c` — add CMD_RESTART and CMD_POWER_OFF handlers

**Tests:** Hardware verification — `board_info` returns carrier board data, `power restart` gracefully reboots the host.

## Progress

- [x] Task 1: Fix EEPROM deferred init and I2C mux sequencing
- [x] Task 2: Add CMD_RESTART and CMD_POWER_OFF to somd
- [x] Task 3: Verify
      **Tasks:** 3 | **Done:** 3

## Tasks

### Task 1: Fix EEPROM deferred init and I2C mux sequencing

**Objective:** Make `board_info` successfully read carrier board identity from EEPROM

**Files:**
- `boards/sifive/hifive_premier_p550_mcu/hifive_premier_p550_mcu.dts`
- `src/board_identity.c`

**Steps:**
1. Add `zephyr,deferred-init;` to the `eeprom@50` DTS node (same as INA226 fix)
2. In `board_identity_init()`: enable I2C mux GPIO FIRST, THEN call `device_init(eeprom_dev)`, THEN check `device_is_ready()`. The mux must be active before the at24 driver probes.
3. Keep the retry logic in `cmd_board_info` for robustness

**Verify:** Flash BMC, run `board_info` — should display serial, MAC addresses, product ID

### Task 2: Add CMD_RESTART and CMD_POWER_OFF to somd

**Objective:** Enable graceful shutdown/restart from BMC via SOM protocol

**Files:**
- `host/somd/somd.c`

**Steps:**
1. Add `CMD_POWER_OFF` (0x01) handler: reply OK, then call `execl("/sbin/poweroff", "poweroff", NULL)` (or `system("poweroff")`)
2. Add `CMD_RESTART` (0x08) handler: reply OK, then call `execl("/sbin/reboot", "reboot", NULL)` (or `system("reboot")`)
3. Reply must be sent BEFORE the system call — once reboot/poweroff starts, the process dies
4. Add `tcdrain(fd)` after writing the reply to ensure it's flushed before exec

**Verify:** Deploy somd to host, run `power restart` from BMC shell — host should reboot gracefully, somd restarts via systemd after boot

### Task 3: Verify

**Objective:** Full verification of both fixes

**Verify:**
- `board_info` displays valid carrier board data
- `power_info` still works (INA226 regression check)
- `som status` shows ONLINE after host boots
- `power restart` gracefully reboots host (somd sends reply, host reboots, somd restarts, BMC detects ONLINE again)
- `power shutdown` gracefully powers off host
