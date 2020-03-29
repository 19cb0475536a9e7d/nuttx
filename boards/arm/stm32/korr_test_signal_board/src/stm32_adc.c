/****************************************************************************
 * boards/arm/stm32/stm32f103-minimum/src/stm32_adc.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <debug.h>
#include <errno.h>

#include <arch/board/board.h>
#include <nuttx/analog/adc.h>
#include <nuttx/board.h>

#include "chip.h"
#include "stm32_adc.h"
#include "stm32_korr_test_signal_board.h"

#ifdef CONFIG_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ********************************************************************/
/* Up to 3 ADC interfaces are supported */

#  if STM32_NADC < 3
#    undef CONFIG_STM32_ADC3
#  endif

#  if STM32_NADC < 2
#    undef CONFIG_STM32_ADC2
#  endif

#  if STM32_NADC < 1
#    undef CONFIG_STM32_ADC1
#  endif

#  if defined(CONFIG_STM32_ADC1) || defined(CONFIG_STM32_ADC2) || defined(CONFIG_STM32_ADC3)
/* The number of ADC channels in the conversion list */

#    define ADC1_NCHANNELS 4
#    define ADC2_NCHANNELS 0
#    define ADC3_NCHANNELS 0

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Identifying number of each ADC channel to be used with Variable Resistor (Pontentiometer)*/

#    ifdef CONFIG_STM32_ADC1
static const uint8_t g_adc1_chanlist[ADC1_NCHANNELS] = {2, 3, 6, 7}; /* ADC12_IN0 */

/* Configurations of pins used byte each ADC channels */

static const uint32_t g_adc1_pinlist[ADC1_NCHANNELS] = {GPIO_ADC123_IN2, GPIO_ADC123_IN3, GPIO_ADC12_IN6,
                                                        GPIO_ADC12_IN7};
#    endif

#    ifdef CONFIG_STM32_ADC2
static const uint8_t g_adc2_chanlist[ADC2_NCHANNELS] = {}; /* Not used */

/* Configurations of pins used byte each ADC channels */

static const uint32_t g_adc2_pinlist[ADC2_NCHANNELS] = {};
#    endif

#    ifdef CONFIG_STM32_ADC3
static const uint8_t g_adc3_chanlist[ADC3_NCHANNELS] = {}; /* ADC12_IN0 */

/* Configurations of pins used byte each ADC channels */

static const uint32_t g_adc3_pinlist[ADC3_NCHANNELS] = {};
#    endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

int32_t stm32_adc_setup(void) {
#    if defined CONFIG_STM32_ADC1 || defined CONFIG_STM32_ADC2 || defined CONFIG_STM32_ADC3
  static bool initialized = false;
  struct adc_dev_s *adc;
  int32_t ret, i, adc_minor = 0;
#      ifdef CONFIG_STM32_ADC1

  /* Check if we have already initialized */

  if (!initialized) {
    /* Configure the pins as analog inputs for the selected channels */

    for (i = 0; i < ADC1_NCHANNELS; i++) {
      stm32_configgpio(g_adc1_pinlist[i]);
    }

    /* Call stm32_adcinitialize() to get an instance of the ADC interface */

    adc = stm32_adcinitialize(1, g_adc1_chanlist, ADC1_NCHANNELS);
    if (adc == NULL) {
      aerr("ERROR: Failed to get ADC interface\n");
      return -ENODEV;
    }

    /* Register the ADC driver at "/dev/adc0" */

    ret = adc_register("/dev/adc0", adc);
    if (ret < 0) {
      aerr("ERROR: adc_register failed: %d\n", ret);
      return ret;
    }
  }
#      endif

#      ifdef CONFIG_STM32_ADC2
  /* Check if we have already initialized */

  if (!initialized) {
    /* Configure the pins as analog inputs for the selected channels */

    for (i = 0; i < ADC2_NCHANNELS; i++) {
      stm32_configgpio(g_adc2_pinlist[i]);
    }

    /* Call stm32_adcinitialize() to get an instance of the ADC interface */

    adc = stm32_adcinitialize(2, g_adc2_chanlist, ADC2_NCHANNELS);
    if (adc == NULL) {
      aerr("ERROR: Failed to get ADC interface\n");
      return -ENODEV;
    }

    /* Register the ADC driver at "/dev/adc0" */

    ret = adc_register("/dev/adc1", adc);
    if (ret < 0) {
      aerr("ERROR: adc_register failed: %d\n", ret);
      return ret;
    }
  }
#      endif

#      ifdef CONFIG_STM32_ADC3
  /* Check if we have already initialized */

  if (!initialized) {
    /* Configure the pins as analog inputs for the selected channels */

    for (i = 0; i < ADC3_NCHANNELS; i++) {
      stm32_configgpio(g_adc3_pinlist[i]);
    }

    /* Call stm32_adcinitialize() to get an instance of the ADC interface */

    adc = stm32_adcinitialize(3, g_adc3_chanlist, ADC3_NCHANNELS);
    if (adc == NULL) {
      aerr("ERROR: Failed to get ADC interface\n");
      return -ENODEV;
    }

    /* Register the ADC driver at "/dev/adc0" */

    ret = adc_register("/dev/adc2", adc);
    if (ret < 0) {
      aerr("ERROR: adc_register failed: %d\n", ret);
      return ret;
    }

    /* Now we are initialized */

  }
#      endif

  initialized = true;
  return OK;
#    else /* defined CONFIG_STM32_ADC1 || defined CONFIG_STM32_ADC2 || defined CONFIG_STM32_ADC3 */
  return -ENOSYS;
#    endif
}

#  endif /* CONFIG_STM32_ADC1 || CONFIG_STM32_ADC2 || CONFIG_STM32_ADC3 */
#endif   /* CONFIG_ADC */
