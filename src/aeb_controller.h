#ifndef AEB_CONTROLLER_H
#define AEB_CONTROLLER_H
#include <stdatomic.h>
void *prediction_thread(void *arg);
extern atomic_int g_inject_hang_ms;
#endif
