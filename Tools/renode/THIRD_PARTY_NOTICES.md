# Third-party notices

## Renode and Emul8-derived files

The following files are adapted from Renode or its predecessor, Emul8:

- `peripherals/stm32/AP_STM32F4_I2C.cs`
- `platforms/stm32f405_base.repl`
- `platforms/stm32f746_ap_base.repl`
- `platforms/stm32h743_base.repl`

Copyright (c) Antmicro <www.antmicro.com>. These files are distributed under
the [MIT licence](https://github.com/antmicro/renode/blob/v1.16.1/LICENSE).
The platform sources came from Renode 1.16.1 and
[ArduPilot's Renode fork](https://github.com/ArduPilot/renode/tree/2a060779f4e2b87d1ae7238a041d858369818805).

## STMicroelectronics CMSIS-SVD files

`data/STM32F405.svd.gz` and `data/STM32G474.svd.gz` are
STMicroelectronics CMSIS-SVD deliverables. They are redistributed under the
[STMicroelectronics End User License Agreement, version 1.0][st-eula],
which permits copying these deliverables for development tools and for
representations used to develop and debug software for the identified
STMicroelectronics devices.

[st-eula]: https://github.com/cmsis-svd/cmsis-svd-data/blob/main/data/STMicro/License.html
