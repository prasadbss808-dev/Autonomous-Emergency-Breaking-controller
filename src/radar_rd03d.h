#ifndef RADAR_RD03D_H
#define RADAR_RD03D_H
#include <stdint.h>
#include <stdbool.h>
int radar_init(const char *dev, int baud);
void radar_ingest(const uint8_t *buf, int n, uint64_t t_rx_ns);
void radar_housekeeping(uint64_t now_ns);
void *radar_thread(void *arg);
void radar_set_verbose(bool v);
void radar_set_hexdump(bool v);
void radar_close(void);
#endif
