/*
 * Firmware image stored in the local "fw" flash partition (offline flashing).
 */

#include "fw_store.h"

#include <string.h>

#include "esp_log.h"
#include "esp_partition.h"
#include "esp_rom_crc.h"

#define FW_MAGIC 0x32495743u /* "CWI2" */

typedef struct {
    uint32_t magic;
    uint32_t size;
    uint32_t crc32;
    uint32_t auth;
    char name[FW_NAME_LEN];
} fw_hdr_t;

static const char *TAG = "fwstore";

static const esp_partition_t *s_part;
static fw_hdr_t s_hdr;
static uint32_t s_write_off;
static uint32_t s_crc;
static uint32_t s_upload_size;
static bool s_uploading;

static uint32_t s_dl_erased;   /* data-area bytes already erased */
static uint32_t s_dl_seq;      /* next sequential offset expected */
static uint32_t s_dl_size;     /* furthest byte written */
static bool s_dl_active;
static bool s_dl_crc_ok;

static uint32_t erase_len(uint32_t size) {
    return (size + 0xFFFu) & ~0xFFFu;
}

int fw_store_init(void) {
    if (s_part != NULL) {
        return 0;
    }
    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, "fw");
    if (s_part == NULL) {
        ESP_LOGE(TAG, "partition 'fw' not found");
        return -1;
    }
    memset(&s_hdr, 0, sizeof(s_hdr));
    esp_partition_read(s_part, FW_HEADER_OFF, &s_hdr, sizeof(s_hdr));
    s_hdr.name[FW_NAME_LEN - 1] = '\0';
    if (s_hdr.magic != FW_MAGIC || s_hdr.size == 0 ||
        s_hdr.size > (s_part->size - FW_DATA_OFF)) {
        memset(&s_hdr, 0, sizeof(s_hdr));
        ESP_LOGW(TAG, "no valid image stored");
    }
    else {
        ESP_LOGI(TAG, "image: %s, %u bytes, crc 0x%08X", s_hdr.name,
                 (unsigned)s_hdr.size, (unsigned)s_hdr.crc32);
    }
    return 0;
}

bool fw_store_valid(void) {
    return s_hdr.magic == FW_MAGIC && s_hdr.size > 0;
}

uint32_t fw_store_size(void) {
    return fw_store_valid() ? s_hdr.size : 0;
}

uint32_t fw_store_crc(void) {
    return fw_store_valid() ? s_hdr.crc32 : 0;
}

uint32_t fw_store_auth(void) {
    if (!fw_store_valid()) {
        return 0;
    }
    return s_hdr.auth != 0 ? s_hdr.auth : s_hdr.crc32;
}

const char *fw_store_name(void) {
    return fw_store_valid() ? s_hdr.name : "";
}

int fw_store_begin(uint32_t size, const char *name) {
    if (fw_store_init() != 0) {
        return -1;
    }
    if (size == 0 || size > (s_part->size - FW_DATA_OFF)) {
        ESP_LOGE(TAG, "bad image size %u", (unsigned)size);
        return -1;
    }
    if (esp_partition_erase_range(s_part, FW_HEADER_OFF, FW_DATA_OFF + erase_len(size)) != ESP_OK) {
        ESP_LOGE(TAG, "erase failed");
        return -1;
    }
    memset(&s_hdr, 0, sizeof(s_hdr));
    s_write_off = 0;
    s_crc = 0;
    s_upload_size = size;
    s_uploading = true;

    strncpy(s_hdr.name, name ? name : "", FW_NAME_LEN - 1);
    return 0;
}

int fw_store_write(const void *data, size_t len) {
    if (!s_uploading) {
        return -1;
    }
    if (s_write_off + len > s_upload_size) {
        ESP_LOGE(TAG, "write overflow");
        return -1;
    }
    if (esp_partition_write(s_part, FW_DATA_OFF + s_write_off, data, len) != ESP_OK) {
        ESP_LOGE(TAG, "write failed at %u", (unsigned)s_write_off);
        return -1;
    }
    s_crc = esp_rom_crc32_le(s_crc, (const uint8_t *)data, len);
    s_write_off += len;
    return 0;
}

int fw_store_finish(uint32_t auth) {
    if (!s_uploading) {
        return -1;
    }
    s_uploading = false;
    if (s_write_off != s_upload_size) {
        ESP_LOGE(TAG, "short image: %u of %u", (unsigned)s_write_off, (unsigned)s_upload_size);
        return -1;
    }
    s_hdr.magic = FW_MAGIC;
    s_hdr.size = s_upload_size;
    s_hdr.crc32 = s_crc;
    s_hdr.auth = auth;
    if (esp_partition_write(s_part, FW_HEADER_OFF, &s_hdr, sizeof(s_hdr)) != ESP_OK) {
        memset(&s_hdr, 0, sizeof(s_hdr));
        return -1;
    }
    ESP_LOGI(TAG, "stored %s: %u bytes, crc 0x%08X", s_hdr.name,
             (unsigned)s_hdr.size, (unsigned)s_hdr.crc32);
    return 0;
}

/*
 * Vendor-tool download.  The official CW-Programmer replays the same ISP frames
 * it would send during an online flash, one frame per APDU, so the image arrives
 * as 0x28 WriteData frames carrying (offset, 240 bytes).  Nothing tells us the
 * total size up front, therefore sectors are erased on demand and the header is
 * written once the tool signals the end of the script.
 */
int fw_store_dl_begin(void) {
    if (fw_store_init() != 0) {
        return -1;
    }
    /* Drop the old header first so an interrupted download cannot look valid. */
    if (esp_partition_erase_range(s_part, FW_HEADER_OFF, FW_DATA_OFF) != ESP_OK) {
        ESP_LOGE(TAG, "erase header failed");
        return -1;
    }
    memset(&s_hdr, 0, sizeof(s_hdr));
    s_dl_erased = 0;
    s_dl_seq = 0;
    s_dl_size = 0;
    s_crc = 0;
    s_dl_crc_ok = true;
    s_dl_active = true;
    return 0;
}

int fw_store_dl_chunk(uint32_t off, const void *data, uint32_t len) {
    if (!s_dl_active || len == 0 || data == NULL) {
        return -1;
    }
    if (off + len > s_part->size - FW_DATA_OFF) {
        ESP_LOGE(TAG, "chunk out of range: %u+%u", (unsigned)off, (unsigned)len);
        return -1;
    }
    if (off + len > s_dl_erased) {
        uint32_t need = erase_len(off + len);
        if (esp_partition_erase_range(s_part, FW_DATA_OFF + s_dl_erased,
                                      need - s_dl_erased) != ESP_OK) {
            ESP_LOGE(TAG, "erase failed at %u", (unsigned)s_dl_erased);
            return -1;
        }
        s_dl_erased = need;
    }
    if (esp_partition_write(s_part, FW_DATA_OFF + off, data, len) != ESP_OK) {
        ESP_LOGE(TAG, "write failed at %u", (unsigned)off);
        return -1;
    }
    if (off == s_dl_seq) {
        s_crc = esp_rom_crc32_le(s_crc, (const uint8_t *)data, len);
        s_dl_seq += len;
    }
    else if (off > s_dl_seq) {
        s_dl_crc_ok = false; /* gap in the stream: crc no longer trustworthy */
    }
    if (off + len > s_dl_size) {
        s_dl_size = off + len;
    }
    return 0;
}

int fw_store_dl_finish(const char *name) {
    if (!s_dl_active) {
        return -1;
    }
    s_dl_active = false;
    if (s_dl_size == 0 || !s_dl_crc_ok || s_dl_seq != s_dl_size) {
        ESP_LOGE(TAG, "download incomplete: %u of %u", (unsigned)s_dl_seq,
                 (unsigned)s_dl_size);
        return -1;
    }
    memset(&s_hdr, 0, sizeof(s_hdr));
    s_hdr.magic = FW_MAGIC;
    s_hdr.size = s_dl_size;
    s_hdr.crc32 = s_crc;
    strncpy(s_hdr.name, name ? name : "", FW_NAME_LEN - 1);
    if (esp_partition_write(s_part, FW_HEADER_OFF, &s_hdr, sizeof(s_hdr)) != ESP_OK) {
        memset(&s_hdr, 0, sizeof(s_hdr));
        return -1;
    }
    ESP_LOGI(TAG, "downloaded %s: %u bytes, crc 0x%08X", s_hdr.name,
             (unsigned)s_hdr.size, (unsigned)s_hdr.crc32);
    return 0;
}

int fw_store_read(uint32_t off, void *buf, size_t len) {
    if (!fw_store_valid() || off + len > s_hdr.size) {
        return -1;
    }
    return esp_partition_read(s_part, FW_DATA_OFF + off, buf, len) == ESP_OK ? 0 : -1;
}
