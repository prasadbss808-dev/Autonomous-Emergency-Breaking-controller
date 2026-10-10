#ifndef GPIO_RPI_H
#define GPIO_RPI_H
#include <stdbool.h>
int gpio_open(const char *path);
bool gpio_is_open(void);
int gpio_set_output(int pin);
int gpio_set_input(int pin);
int gpio_set_pull(int pin, int pud);
int gpio_write(int pin, int level);
int gpio_read(int pin, int *level);
int gpio_pwm_setup(int pin, unsigned freq_hz);
int gpio_pwm_duty(int pin, float percent);
void gpio_close(void);
#endif
