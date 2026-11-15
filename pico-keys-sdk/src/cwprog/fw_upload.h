/*
 * Firmware upload over the CDC ACM interface.
 *
 * Binary protocol, one command per request, one byte reply ('K' / 'E'):
 *   'Q'                                    -> "CWU1 <size> <crc32 hex> <name>\n"
 *   'B' <u32 size> <u8 name_len> <name>    -> start of image
 *   'D' <u16 len> <data>                   -> one chunk
 *   'E' <u32 crc32> <u32 auth>             -> commit, crc checked
 */

#ifndef FW_UPLOAD_H
#define FW_UPLOAD_H

void fw_upload_task(void);

#endif /* FW_UPLOAD_H */
