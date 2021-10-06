# Kernel Abeja Maya

This is custom kernel (3.18.79) source for Bluboo Maya (MT6580) by Ruben1863

## Known information:
| Subsystem | Driver name | Availability | Working |
|-----------|-------------|--------------|---------|
| LCM driver | `s6d7aa0_dsi_vdo_common` | Yes | Yes |
| Touch panel | `FTS_TS (i2c 1-0038)` | Yes | Yes |
| GPU | `Mali-400 MP2` | Yes | Yes |
| Camera #1 | `hi843b_mipi_raw` | Yes | Yes |
| Camera #2 | `gc5005_mipi_raw` | Yes | Yes |
| Accelerometer | `MXC622X (i2c 2-0015)` | Yes | Yes |
| ALS/PS | `Unknown` | - | - |
| Flash | `DS2016` | - | Yes |
| Lens | `DW9714AF` | Yes | Yes |
| RAM | `2 GB LPDDR3_1066` | - | Yes |
| Sound | `amp_6323pmic_spk` | Yes | Yes |
| Accdet | `mt6580-accdet` | - | Yes |
| Other | `kd_camera_hw (i2c 0-0036)` | Yes | Yes |

## Known Issues
- Hi843b: Poor quality? Video recording now working (Also just 8mpx)
- Gc5005: Poor quality? (Also just 5mpx)
- ALS/PS: sensor is unknwon, so for now there isn't light/proximity sensor.

## Current kernel features:
* Underclock CPU To 260MHz.
* Overclock GPU To 600MHz (Disabled "CONFIG_GPU_OC=y").
* Underclock GPU To 90MHz (Disabled "CONFIG_GPU_OC=y").

## Acknowledgements:

* [(@jmpfbmx)](https://github.com/jmpfbmx)
* [(@parthibx24)](https://github.com/parthibx24)
* [(@cvolo4yzhka)](https://github.com/cvolo4yzhka)
* [(@zoggn)](https://github.com/zoggn)
* [(@svoboda18)](https://github.com/svoboda18)
* [(@raymondmiracle)](https://github.com/raymondmiracle)
* [(@hsagent)](https://github.com/hsagent)
* [(@0xXA)](https://github.com/0xXA)
