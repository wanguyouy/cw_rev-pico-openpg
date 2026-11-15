/*
 * Firmware image stored in the local "fw" flash partition, used by offline
 * programming.  Layout: 4 KB header sector followed by the raw .bin image.
 */

#ifndef FW_STORE_H
#define FW_STORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FW_NAME_LEN 16

#define FW_HEADER_OFF 0x0000
#define FW_DATA_OFF   0x1000

int fw_store_init(void);

bool fw_store_valid(void);
uint32_t fw_store_size(void);
uint32_t fw_store_crc(void);
/* Vendor-tool style "auth code" (MD5 of the .hex text, folded); falls back to
 * the image CRC when the image was downloaded by the vendor tool itself. */
uint32_t fw_store_auth(void);
const char *fw_store_name(void);

/* Streaming upload: begin -> write* -> finish. */
int fw_store_begin(uint32_t size, const char *name);
int fw_store_write(const void *data, size_t len);
int fw_store_finish(uint32_t auth);

/* Download driven by the vendor tool: chunks arrive at absolute offsets and
 * the total size is only known at the end. */
int fw_store_dl_begin(void);
int fw_store_dl_chunk(uint32_t off, const void *data, uint32_t len);
int fw_store_dl_finish(const char *name);

/* Random read of the stored image (offset 0 = first byte of the .bin). */
int fw_store_read(uint32_t off, void *buf, size_t len);

#endif /* FW_STORE_H */
