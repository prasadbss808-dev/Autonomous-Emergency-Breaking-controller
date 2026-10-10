#ifndef SIM_RT_H
#define SIM_RT_H
#include <stdatomic.h>
int sim_rt_init(int scenario_id);
void *sim_thread(void *arg);
extern atomic_int g_sim_result;
#endif
