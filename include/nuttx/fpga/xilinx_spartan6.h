#ifndef __INCLUDE_NUTTX_FPGA_XILINX_SPARTAN6_H
#define __INCLUDE_NUTTX_FPGA_XILINX_SPARTAN6_H

#include <nuttx/compiler.h>
#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#define CONFIG_FPGA_FIFOSIZE 8

begin_packed_struct struct fpga_msg_s { int32_t fpga_data; } end_packed_struct;

struct fpga_fifo_s {
  sem_t af_sem;    /* Counting semaphore */
  uint8_t af_head; /* Index to the head [IN] index
                    * in the circular buffer */
  uint8_t af_tail; /* Index to the tail [OUT] index
                    * in the circular buffer */
                   /* Circular buffer of DAC messages */
  struct fpga_msg_s af_buffer[CONFIG_FPGA_FIFOSIZE];
};

struct fpga_dev_s;
struct dac_ops_s {
  /* Reset the FPGA device.  Called early to initialize the hardware. This
   * is called, before ao_setup() and on error conditions.
   */

  CODE void (*fpga_reset)(FAR struct fpga_dev_s *dev);

  /* Configure the FPGA. This method is called the first time that the FPGA
   * device is opened.  This will occur when the port is first opened.
   * This setup includes configuring and attaching DAC interrupts.
   * Interrupts are all disabled upon return.
   */

  CODE int (*fpga_setup)(FAR struct fpga_dev_s *dev);

  /* Disable the FPGA.  This method is called when the FPGA is closed.
   * This method reverses the operation the setup method.
   */
  CODE void (*fpga_shutdown)(FAR struct fpga_dev_s *dev);

  /* This method will send one message on the DAC */
  CODE int (*fpga_send)(FAR struct fpga_dev_s *dev, FAR struct fpga_msg_s *msg);

  /* All ioctl calls will be routed through this method */
  CODE int (*fpga_ioctl)(FAR struct fpga_dev_s *dev, int32_t cmd, unsigned long arg);
};

struct fpga_dev_s {
  uint8_t fpga_dev_ocount;           /* The number of times the device has
                                      * been opened */
  sem_t fpga_dev_closesem;           /* Locks out new opens while close is
                                      * in progress */
  struct fpga_fifo_s fpga_xmit;      /* Describes receive FIFO */
  const struct fpga_ops_s *fpga_ops; /* Arch-specific operations */
  void *fpga_priv;                   /* Used by the arch-specific logic */
};

#if defined(__cplusplus)
extern "C" {
#endif

/****************************************************************************
 * Name: fpga_register
 *
 * Description:
 *   Register a FPGA driver.
 *
 * Input Parameters:
 *    path - The full path to the DAC device to be registered.  This could
 *      be, as an example, "/dev/fpga0"
 *    dev - An instance of the device-specific FPGA interface
 *
 * Returned Value:
 *    Zero on success; A negated errno value on failure.
 *
 ****************************************************************************/

int dac_register(FAR const char *path, FAR struct fpga_dev_s *dev);
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_FPGA_XILINX_SPARTAN6_H */
