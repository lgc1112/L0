SET PATH=F:\install\keil\ARM\ARMCC\Bin;C:\Program Files (x86)\Java\jdk1.8.0_91\bin;C:\Program Files (x86)\Java\jdk1.8.0_91\jre\bin;C:\Program Files (x86)\Windows Kits\8.1\Windows Performance Toolkit\;C:\Program Files\Microsoft SQL Server\110\Tools\Binn\;C:\Users\Administrator\AppData\Local\GitHubDesktop\bin
SET CPU_TYPE=STM32L053R8
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=STM32L053R8_NUCLEO
SET CPU_CLOCK=0x007A1200
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\main.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\bochiot_protocol.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\crc.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\i2c.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\loar.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\sx1272-lora.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\uart_debug.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\dataprocess.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_msp.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_it.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_nucleo.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\usbd_conf.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\usbd_desc.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_adc.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_adc_ex.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_cortex.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_dma.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_flash.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_flash_ex.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_gpio.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_iwdg.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_pcd.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_pcd_ex.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_pwr.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_pwr_ex.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_rcc.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_rcc_ex.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_rtc.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_rtc_ex.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_spi.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_tim.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_tim_ex.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_uart.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\stm32l0xx_hal_uart_ex.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\usbd_customhid.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\usbd_core.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\usbd_ctlreq.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\usbd_ioreq.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\system_stm32l0xx.__i"
"F:\install\keil\ARM\ARMCC\Bin\ArmAsm" --Via ".\obj\startup_stm32l053xx._ia"
"F:\install\keil\ARM\ARMCC\Bin\ArmLink" --Via ".\OBJ\STM32L053R8_NUCLEO.lnp"
"F:\install\keil\ARM\ARMCC\Bin\fromelf.exe" ".\OBJ\STM32L053R8_NUCLEO.axf" --i32combined --output ".\OBJ\STM32L053R8_NUCLEO.hex"
