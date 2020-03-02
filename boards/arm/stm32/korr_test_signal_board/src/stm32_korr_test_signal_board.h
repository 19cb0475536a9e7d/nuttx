/****************************************************************************
 * boards/arm/stm32/stm32f103-minimum/src/stm32f103_minimum.h
 *
 *   Copyright (C) 2016, 2018 Gregory Nutt. All rights reserved.
 *   Author: Laurent Latil <laurent@latil.nom.fr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32_STM32F103_MINIMUM_SRC_H
#define __BOARDS_ARM_STM32_STM32F103_MINIMUM_SRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
#include <nuttx/config.h>
#include <stdint.h>

#include <arch/chip/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* procfs File System */
#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* How many SPI modules does this chip support? The LM3S6918 supports 2 SPI
 * modules (others may support more -- in such case, the following must be
 * expanded).
 */
#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 3
#  undef CONFIG_STM32_SPI3
#endif

/* GPIOs **************************************************************/
/* LEDs */
#define GPIO_LED1 (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN0)
#define GPIO_LED2 (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN1)
#define GPIO_LED3 (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN2)
#define GPIO_LED4 (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN3)

/* BUTTONs */
#define GPIO_BTN_USER1 (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT | GPIO_EXTI | GPIO_PORTC | GPIO_PIN13)
#define GPIO_BTN_USER2 (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT | GPIO_EXTI | GPIO_PORTC | GPIO_PIN14)
#define GPIO_BTN_USER3 (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT | GPIO_EXTI | GPIO_PORTC | GPIO_PIN15)

#define MIN_IRQBUTTON BUTTON_USER1
#define MAX_IRQBUTTON BUTTON_USER3
#define NUM_IRQBUTTONS (BUTTON_USER1 - BUTTON_USER3 + 1)

/* SPI chip selects */
#define FLASH_SPI2_CS (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN11)
#define FPGA_SPI2_CS (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN12)

/* USB Soft Connect Pullup: PC.13 */
#define GPIO_USB_PULLUP (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN12)

/****************************************************************************
 * Public Functions
 ****************************************************************************/
#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture specific initialization
 *
 *   CONFIG_LIB_BOARDCTL=y:
 *     If CONFIG_NSH_ARCHINITIALIZE=y:
 *       Called from the NSH library (or other application)
 *     Otherwise, assumed to be called from some other application.
 *
 *   Otherwise CONFIG_BOARD_LATE_INITIALIZE=y:
 *     Called from board_late_initialize().
 *
 *   Otherwise, bad news:  Never called
 *
 ****************************************************************************/
int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/
#  ifdef CONFIG_DEV_GPIO
int stm32_gpio_initialize(void);
#  endif

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/
#  ifdef CONFIG_ADC
int stm32_adc_setup(void);
#  endif

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Hy-Mini STM32v board.
 *
 ****************************************************************************/
void stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_w25initialize
 *
 * Description:
 *   Called to initialize Winbond W25 memory
 *
 ****************************************************************************/
int stm32_w25initialize(int minor);

/****************************************************************************
 * Name: stm32_fpga_initialize
 *
 * Description:
 *   Called to initialize FPGA from bitstream file in W25 memory
 *
 ****************************************************************************/
int stm32_w25initialize(int minor);

/****************************************************************************
 * Name: stm32_rgbled_setup
 *
 * Description:
 *   This function is called by board initialization logic to configure the
 *   RGB LED driver.  This function will register the driver as /dev/rgbled0.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/
#  ifdef CONFIG_RGBLED
int stm32_rgbled_setup(void);
#  endif

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the board.
 *
 ****************************************************************************/
void stm32_usbinitialize(void);
#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_STM32F103_MINIMUM_SRC_H */
