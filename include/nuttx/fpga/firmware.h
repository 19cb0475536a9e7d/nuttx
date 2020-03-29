#ifndef __INCLUDE_NUTTX_FPGA_FIRMWARE_H
#define __INCLUDE_NUTTX_FPGA_FIRMWARE_H

#include <nuttx/compiler.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/errno.h>
#include <sys/types.h>

#define FW_ACTION_NOHOTPLUG 0
#define FW_ACTION_HOTPLUG 1

struct firmware_s {
  size_t size;
  const uint8_t *data;
  struct page **pages;

  /* firmware loader private fields */
  void *priv;
};

struct builtin_fw_s {
  char *name;
  void *data;
  unsigned long size;
};

/* We have to play tricks here much like stringify() to get the
   __COUNTER__ macro to be expanded as we want it */
#define __fw_concat1(x, y) x##y
#define __fw_concat(x, y) __fw_concat1(x, y)
#define DECLARE_BUILTIN_FIRMWARE(name, blob) DECLARE_BUILTIN_FIRMWARE_SIZE(name, &(blob), sizeof(blob))

#define DECLARE_BUILTIN_FIRMWARE_SIZE(name, blob, size)                                                                \
  static const struct builtin_fw __fw_concat(__builtin_fw, __COUNTER__)                                                \
      __used __section(.builtin_fw) = {name, blob, size}

static inline int request_firmware(const struct firmware_s **fw, const char *name, struct device_s *device) {
  return -EINVAL;
}

static inline int firmware_request_nowarn(const struct firmware_s **fw, const char *name, struct device_s *device) {
  return -EINVAL;
}

static inline int request_firmware_nowait(struct module_s *module, bool uevent, const char *name,
                                          struct device_s *device, gfp_t gfp, void *context,
                                          void (*cont)(const struct firmware_s *fw, void *context)) {
  return -EINVAL;
}

static inline void release_firmware(const struct firmware_s *fw) {}

static inline int request_firmware_direct(const struct firmware_s **fw, const char *name, struct device_s *device) {
  return -EINVAL;
}

static inline int request_firmware_into_buf(const struct firmware_s **firmware_p, const char *name,
                                            struct device_s *device, void *buf, size_t size) {
  return -EINVAL;
}

int firmware_request_cache(struct device_s *device, const char *name);

#endif
