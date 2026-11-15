/*
 * Offline programming: flashes the image stored in the "fw" partition into the
 * target, without a PC.  Triggered by holding BOOT for one second.
 */

#ifndef CW_OFFLINE_H
#define CW_OFFLINE_H

int cw_offline_run(void);
void cw_offline_task(void);

#endif /* CW_OFFLINE_H */
