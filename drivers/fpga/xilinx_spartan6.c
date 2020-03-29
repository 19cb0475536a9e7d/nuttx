/************************************************************************************
 * drivers/fpga/xilinx_spartan6.c
 * Driver for Xilinx XC6SLX FPGA
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/fpga/fpga_mgr.h>
#include <nuttx/fpga/xilinx_spartan6.h>
#include <nuttx/fs/fs.h>
#include <nuttx/signal.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

/* Per the data sheet, the W25 parts can be driven with either SPI mode 0 (CPOL=0
 * and CPHA=0) or mode 3 (CPOL=1 and CPHA=1). But I have heard that other devices
 * can operate in mode 0 or 1.  So you may need to specify CONFIG_W25_SPIMODE to
 * select the best mode for your device.  If CONFIG_W25_SPIMODE is not defined,
 * mode 0 will be used.
 */

#ifndef CONFIG_FPGA_SPIMODE
#  define CONFIG_XC6SLX_SPIMODE SPIDEV_MODE0
#endif

/* SPI Frequency.  May be up to 25MHz. */
#ifndef CONFIG_FPGA_SPIFREQUENCY
#  define CONFIG_XC6SLX_SPIFREQUENCY 20000000
#endif

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s must
 * appear at the beginning of the definition so that you can freely cast between
 * pointers to struct mtd_dev_s and struct w25_dev_s.
 */

enum fpga_conf_mode_e {
  FPGA_CONF_PASSIVE_SERIAL = 0, /* CPOL=0 CHPHA=0 */
  FPGA_CONF_ACTIVE_SERIAL,      /* CPOL=0 CHPHA=1 */
  FPGA_CONF_MODE2,              /* CPOL=1 CHPHA=0 */
  FPGA_CONF_MODE3,              /* CPOL=1 CHPHA=1 */
};

struct xilinx_spartan6_opts_s {
  CODE int32_t (*done)();
  CODE int32_t (*prog_b_up)();
  CODE int32_t (*prog_b_down)();
  CODE int32_t (*m0_up)(FAR struct spi_dev_s *dev, bool lock);
  CODE int32_t (*m0_down)(FAR struct spi_dev_s *dev, bool lock);
  CODE int32_t (*m1_up)(FAR struct spi_dev_s *dev, bool lock);
  CODE int32_t (*m1_down)(FAR struct spi_dev_s *dev, bool lock);
  CODE int32_t (*hswapen_up)(FAR struct spi_dev_s *dev, bool lock);
  CODE int32_t (*hswap_down)(FAR struct spi_dev_s *dev, bool lock);
};

struct xilinx_spartan6_spi_conf_s {
  struct spi_dev_s *spi;
  struct xilinx_spartan6_opts_s *opts;
};

static enum fpga_mgr_states_e xilinx_spi_state(struct fpga_manager_s *mgr) {
  struct xilinx_spartan6_spi_conf_s *conf = mgr->priv;

  if (!conf->opts->done())
    return FPGA_MGR_STATE_RESET;

  return FPGA_MGR_STATE_UNKNOWN;
}

static int xilinx_spi_write_init(struct fpga_manager_s *mgr, struct fpga_image_info_s *info, const char *buf,
                                 size_t count) {
  struct xilinx_spartan6_spi_conf_s *conf = mgr->priv;
  const size_t prog_latency_7500us = 7500;
  const size_t prog_pulse_1us = 1;

  if (info->flags & FPGA_MGR_PARTIAL_RECONFIG) {
    ferr("Partial reconfiguration not supported.\n");
    return -EINVAL;
  }
  
  conf->opts->prog_b_up();
  udelay(prog_pulse_1us); /* min is 500 ns */
  conf->opts->prog_b_down();

  if (conf->opts->done()) {
    ferr("Unexpected DONE pin state...\n");
    return -EIO;
  }

  /* program latency */
  usleep_range(prog_latency_7500us, prog_latency_7500us + 100);
  return 0;
}

static int xilinx_spi_write(struct fpga_manager_s *mgr, const char *buf, size_t count) {
  struct xilinx_spartan6_spi_conf_s *conf = mgr->priv;
  const char *fw_data = buf;
  const char *fw_data_end = fw_data + count;

  while (fw_data < fw_data_end) {
    size_t remaining, stride;
    int ret;

    remaining = fw_data_end - fw_data;
    stride = min_t(size_t, remaining, SZ_4K);

    ret = spi_write(conf->spi, fw_data, stride);
    if (ret) {
      ferr("SPI error in firmware write: %d\n", ret);
      return ret;
    }

    fw_data += stride;
  }

  return 0;
}

static int32_t xilinx_spi_apply_cclk_cycles(struct xilinx_spartan6_spi_conf_s *conf) {
  struct spi_dev_s *spi = conf->spi;
  const uint8_t din_data[1] = {0xff};
  int32_t ret;

  ret = spi_write(conf->spi, din_data, sizeof(din_data));
  if (ret)
    ferr(&spi->dev, "applying CCLK cycles failed: %d\n", ret);

  return ret;
}

static int32_t xilinx_spi_write_complete(struct fpga_manager_s *mgr, struct fpga_image_info_s *info) {
  struct xilinx_spartan6_spi_conf_s *conf = mgr->priv;
  unsigned long timeout;
  int32_t ret;

  if (conf->opts->done())
    return xilinx_spi_apply_cclk_cycles(conf);

  timeout = jiffies + usecs_to_jiffies(info->config_complete_timeout_us);

  while (time_before(jiffies, timeout)) {

    ret = xilinx_spi_apply_cclk_cycles(conf);
    if (ret)
      return ret;

    if (conf->opts->done())
      return xilinx_spi_apply_cclk_cycles(conf);
  }

  ferr("Timeout after config data transfer.\n");
  return -ETIMEDOUT;
}

static const struct fpga_manager_ops_s xilinx_spi_ops = {
    .state = xilinx_spi_state,
    .write_init = xilinx_spi_write_init,
    .write = xilinx_spi_write,
    .write_complete = xilinx_spi_write_complete,
};

static int32_t xilinx_spi_probe(struct spi_dev_s *spi) {
  struct xilinx_spartan6_spi_conf_s *conf;
  struct fpga_manager_s *mgr;

  conf = devm_kzalloc(&spi->dev, sizeof(*conf), GFP_KERNEL);
  if (!conf)
    return -ENOMEM;

  conf->spi = spi;

  /* PROGRAM_B is active low */
  conf->prog_b = devm_gpiod_get(&spi->dev, "prog_b", GPIOD_OUT_LOW);
  if (IS_ERR(conf->prog_b)) {
    dev_err(&spi->dev, "Failed to get PROGRAM_B gpio: %ld\n", PTR_ERR(conf->prog_b));
    return PTR_ERR(conf->prog_b);
  }

  conf->done = devm_gpiod_get(&spi->dev, "done", GPIOD_IN);
  if (IS_ERR(conf->done)) {
    dev_err(&spi->dev, "Failed to get DONE gpio: %ld\n", PTR_ERR(conf->done));
    return PTR_ERR(conf->done);
  }

  mgr = devm_fpga_mgr_create(&spi->dev, "Xilinx Slave Serial FPGA Manager", &xilinx_spi_ops, conf);
  if (!mgr)
    return -ENOMEM;

  spi_set_drvdata(spi, mgr);
  return fpga_mgr_register(mgr);
}

static int32_t xilinx_spi_remove(struct spi_dev_s *spi) {
  struct fpga_manager_s *mgr = spi_get_drvdata(spi);
  fpga_mgr_unregister(mgr);
  return 0;
}
