#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H
#include "aeb_types.h"
extern _Atomic float g_sim_ego_speed;
const char *ego_src_name(ego_src_t s);
int ego_src_parse(const char *s, ego_src_t *out);
void *vehicle_thread(void *arg);
#endif
