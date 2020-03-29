/****************************************************************************
 * boards/arm/stm32/korr_test_signal_board/src/stm32_fpga.c
 *
 *   Copyright (C) 2020 Oleg Shishlyannikov. All rights reserved.
 *   Author: Oleg Shishlyannikov <oleg.shishlyannikov.1992@gmail.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mount.h>

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef CONFIG_STM32_SPI2
#  include <nuttx/spi/spi.h>
#endif

#include "stm32_korr_test_signal_board.h"
#include "stm32_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing the watchdog
 * timer
 */

#define FPGA_SPI_PORT 2

/* Configuration ************************************************************/
/* Can't support the W25 device if it SPI2 or W25 support is not enabled */

#define HAVE_FPGA 1
#if !defined(CONFIG_STM32_SPI2) || !defined(CONFIG_FPGA) || !defined(HAVE_W25)
#  undef HAVE_FPGA
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_fpgainitialize
 *
 * Description:
 *   Initialize and register the FPGA file system.
 *
 ****************************************************************************/

int stm32_fpgainitialize(int minor) {
  int ret;
#ifdef HAVE_FPGA
  FAR struct spi_dev_s *spi;
  FAR struct fpga_dev_s *fpga;
  /* Get the SPI port */

  spi = stm32_spibus_initialize(FPGA_SPI_PORT);
  if (!spi) {
    syslog(LOG_ERR, "ERROR: Failed to initialize SPI port %d\n", FPGA_SPI_PORT);
    return -ENODEV;
  }

  /* Now bind the SPI interface to the W25 SPI FLASH driver */
  fpga = fpga_initialize(spi);
  if (!fpga) {
    syslog(LOG_ERR,
           "ERROR: Failed to bind SPI port %d to the Xilinx"
           "XC6SLX fpga driver\n",
           FPGA_SPI_PORT);
    return -ENODEV;
  }
#endif /* HAVE_FPGA */

  return OK;
}
