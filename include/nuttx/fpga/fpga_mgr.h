#ifndef __INCLUDE_NUTTX_FPGA_FPGA_MGR_H
#define __INCLUDE_NUTTX_FPGA_FPGA_MGR_H

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#define BIT(nr) (1UL << (nr))
struct fpga_manager_s;
/**
 * enum fpga_mgr_states - fpga framework states
 * @FPGA_MGR_STATE_UNKNOWN: can't determine state
 * @FPGA_MGR_STATE_POWER_OFF: FPGA power is off
 * @FPGA_MGR_STATE_POWER_UP: FPGA reports power is up
 * @FPGA_MGR_STATE_RESET: FPGA in reset state
 * @FPGA_MGR_STATE_FIRMWARE_REQ: firmware request in progress
 * @FPGA_MGR_STATE_FIRMWARE_REQ_ERR: firmware request failed
 * @FPGA_MGR_STATE_WRITE_INIT: preparing FPGA for programming
 * @FPGA_MGR_STATE_WRITE_INIT_ERR: Error during WRITE_INIT stage
 * @FPGA_MGR_STATE_WRITE: writing image to FPGA
 * @FPGA_MGR_STATE_WRITE_ERR: Error while writing FPGA
 * @FPGA_MGR_STATE_WRITE_COMPLETE: Doing post programming steps
 * @FPGA_MGR_STATE_WRITE_COMPLETE_ERR: Error during WRITE_COMPLETE
 * @FPGA_MGR_STATE_OPERATING: FPGA is programmed and operating
 */
enum fpga_mgr_states_e {
  /* default FPGA states */
  FPGA_MGR_STATE_UNKNOWN,
  FPGA_MGR_STATE_POWER_OFF,
  FPGA_MGR_STATE_POWER_UP,
  FPGA_MGR_STATE_RESET,

  /* getting an image for loading */
  FPGA_MGR_STATE_FIRMWARE_REQ,
  FPGA_MGR_STATE_FIRMWARE_REQ_ERR,

  /* write sequence: init, write, complete */
  FPGA_MGR_STATE_WRITE_INIT,
  FPGA_MGR_STATE_WRITE_INIT_ERR,
  FPGA_MGR_STATE_WRITE,
  FPGA_MGR_STATE_WRITE_ERR,
  FPGA_MGR_STATE_WRITE_COMPLETE,
  FPGA_MGR_STATE_WRITE_COMPLETE_ERR,

  /* fpga is programmed and operating */
  FPGA_MGR_STATE_OPERATING,
};

/**
 * DOC: FPGA Manager flags
 *
 * Flags used in the &fpga_image_info->flags field
 *
 * %FPGA_MGR_PARTIAL_RECONFIG: do partial reconfiguration if supported
 *
 * %FPGA_MGR_EXTERNAL_CONFIG: FPGA has been configured prior to Linux booting
 *
 * %FPGA_MGR_ENCRYPTED_BITSTREAM: indicates bitstream is encrypted
 *
 * %FPGA_MGR_BITSTREAM_LSB_FIRST: SPI bitstream bit order is LSB first
 *
 * %FPGA_MGR_COMPRESSED_BITSTREAM: FPGA bitstream is compressed
 */
#define FPGA_MGR_PARTIAL_RECONFIG BIT(0)
#define FPGA_MGR_EXTERNAL_CONFIG BIT(1)
#define FPGA_MGR_ENCRYPTED_BITSTREAM BIT(2)
#define FPGA_MGR_BITSTREAM_LSB_FIRST BIT(3)
#define FPGA_MGR_COMPRESSED_BITSTREAM BIT(4)

/**
 * struct fpga_image_info - information specific to a FPGA image
 * @flags: boolean flags as defined above
 * @enable_timeout_us: maximum time to enable traffic through bridge (uSec)
 * @disable_timeout_us: maximum time to disable traffic through bridge (uSec)
 * @config_complete_timeout_us: maximum time for FPGA to switch to operating
 *	   status in the write_complete op.
 * @firmware_name: name of FPGA image firmware file
 * @sgt: scatter/gather table containing FPGA image
 * @buf: contiguous buffer containing FPGA image
 * @count: size of buf
 * @region_id: id of target region
 * @dev: device that owns this
 * @overlay: Device Tree overlay
 */
struct fpga_image_info_s {
  uint32_t flags;
  uint32_t enable_timeout_us;
  uint32_t disable_timeout_us;
  uint32_t config_complete_timeout_us;
  char *firmware_name;
  const char *buf;
  size_t count;
  int32_t region_id;
  struct fpga_dev_s *dev;
};

/**
 * struct fpga_manager_ops - ops for low level fpga manager drivers
 * @initial_header_size: Maximum number of bytes that should be passed into write_init
 * @state: returns an enum value of the FPGA's state
 * @status: returns status of the FPGA, including reconfiguration error code
 * @write_init: prepare the FPGA to receive confuration data
 * @write: write count bytes of configuration data to the FPGA
 * @write_sg: write the scatter list of configuration data to the FPGA
 * @write_complete: set FPGA to operating state after writing is done
 * @fpga_remove: optional: Set FPGA into a specific state during driver remove
 * @groups: optional attribute groups.
 *
 * fpga_manager_ops are the low level functions implemented by a specific
 * fpga manager driver.  The optional ones are tested for NULL before being
 * called, so leaving them out is fine.
 */
struct fpga_manager_ops_s {
  size_t initial_header_size;
  enum fpga_mgr_states (*state)(struct fpga_manager_s *mgr);
  uint64_t (*status)(struct fpga_manager_s *mgr);
  int32_t (*write_init)(struct fpga_manager_s *mgr, struct fpga_image_info_s *info, const char *buf, size_t count);
  int32_t (*write)(struct fpga_manager_s *mgr, const char *buf, size_t count);
  int32_t (*write_complete)(struct fpga_manager_s *mgr, struct fpga_image_info_s *info);
  void (*fpga_remove)(struct fpga_manager_s *mgr);
};

/* FPGA manager status: Partial/Full Reconfiguration errors */
#define FPGA_MGR_STATUS_OPERATION_ERR BIT(0)
#define FPGA_MGR_STATUS_CRC_ERR BIT(1)
#define FPGA_MGR_STATUS_INCOMPATIBLE_IMAGE_ERR BIT(2)
#define FPGA_MGR_STATUS_IP_PROTOCOL_ERR BIT(3)
#define FPGA_MGR_STATUS_FIFO_OVERFLOW_ERR BIT(4)

/**
 * struct fpga_compat_id - id for compatibility check
 *
 * @id_h: high 64bit of the compat_id
 * @id_l: low 64bit of the compat_id
 */
struct fpga_compat_id_s {
  uint64_t id_h;
  uint64_t id_l;
};

/**
 * struct fpga_manager - fpga manager structure
 * @name: name of low level fpga manager
 * @dev: fpga manager device
 * @ref_mutex: only allows one reference to fpga manager
 * @state: state of fpga manager
 * @compat_id: FPGA manager id for compatibility check.
 * @mops: pointer to struct of fpga manager ops
 * @priv: low level driver private date
 */
struct fpga_manager_s {
  const char *name;
  struct fpga_dev_s *dev;
  enum fpga_mgr_states_e state;
  struct fpga_compat_id_s *compat_id;
  const struct fpga_manager_ops_s *mops;
  void *priv;
};

#define to_fpga_manager(d) container_of(d, struct fpga_manager, dev)

struct fpga_image_info_s *fpga_image_info_alloc(struct fpga_dev_s *dev);
void fpga_image_info_free(struct fpga_image_info_s *info);
int32_t fpga_mgr_load(struct fpga_manager_s *mgr, struct fpga_image_info_s *info);
int32_t fpga_mgr_lock(struct fpga_manager_s *mgr);
void fpga_mgr_unlock(struct fpga_manager_s *mgr);
struct fpga_manager_s *of_fpga_mgr_get(struct fpga_dev_s *fpga);
struct fpga_manager_s *fpga_mgr_get(struct fpga_dev_s *dev);
void fpga_mgr_put(struct fpga_manager_s *mgr);
struct fpga_manager_s *fpga_mgr_create(struct fpga_dev_s *dev, const char *name, const struct fpga_manager_ops_s *mops,
                                       void *priv);
void fpga_mgr_free(struct fpga_manager_s *mgr);
int32_t fpga_mgr_register(struct fpga_manager_s *mgr);
void fpga_mgr_unregister(struct fpga_manager_s *mgr);
struct fpga_manager_s *devm_fpga_mgr_create(struct fpga_dev_s *dev, const char *name,
                                            const struct fpga_manager_ops_s *mops, void *priv);

#endif /*__INCLUDE_NUTTX_FPGA_FPGA_MGR_H */
