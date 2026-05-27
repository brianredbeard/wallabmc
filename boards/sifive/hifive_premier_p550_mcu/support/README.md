# Flashing WallaBMC on the HiFive Premier P550

## Prerequisites

1. **OpenOCD** — install via your package manager:
   ```bash
   # macOS
   brew install open-ocd

   # Ubuntu/Debian
   sudo apt-get install openocd
   ```

2. **USB-C debug cable** — connect to the P550 debug port (exposes FT4232H with 4 channels: SoC JTAG, MCU JTAG, SoC UART, MCU UART)

## Flash from CI artifacts

Download the `wallabmc-firmware-hifive_premier_p550_mcu` artifact from a successful [GitHub Actions run](https://github.com/tenstorrent/wallabmc/actions). Unzip and flash:

```bash
openocd -f boards/sifive/hifive_premier_p550_mcu/support/p550_openocd.cfg \
  -c 'program mcuboot.hex verify; program wallabmc.signed.hex verify reset exit'
```

## Flash from a local build

```bash
openocd -f boards/sifive/hifive_premier_p550_mcu/support/p550_openocd.cfg \
  -c 'program build/mcuboot/zephyr/zephyr.hex verify; program build/wallabmc/zephyr/zephyr.signed.hex verify reset exit'
```

## Troubleshooting

If the MCU won't halt (DAP WAIT stalls), run `sudo` and try a reset first:

```bash
sudo openocd -f boards/sifive/hifive_premier_p550_mcu/support/p550_openocd.cfg -c 'init; reset; exit'
sudo openocd -f boards/sifive/hifive_premier_p550_mcu/support/p550_openocd.cfg \
  -c 'init; halt; stm32f4x unlock 0; program mcuboot.hex verify; program wallabmc.signed.hex verify reset exit'
```

Close any active serial console sessions before flashing — the FTDI kernel driver claims all FT4232H channels.

## Restore SiFive vendor firmware

```bash
wget https://raw.githubusercontent.com/sifiveinc/hifive-premier-p550-tools/refs/heads/master/mcu-firmware/STM32F407VET6_BMC.elf
openocd -f boards/sifive/hifive_premier_p550_mcu/support/p550_openocd.cfg \
  -c 'program STM32F407VET6_BMC.elf verify reset exit'
```
