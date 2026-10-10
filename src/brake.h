#ifndef AEB_BRAKE_H
#define AEB_BRAKE_H
int brake_init(void);
int brake_coid(void);
void *brake_thread(void *arg);
void brake_stop(void);
#endif
