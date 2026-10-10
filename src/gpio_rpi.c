#include "gpio_rpi.h"
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <sys/neutrino.h>
#include <sys/iomsg.h>
#include <sys/iomgr.h>
#if defined(__has_include)
#if __has_include(<sys/rpi_gpio.h>)
#include <sys/rpi_gpio.h>
#define AEB_HAVE_SYS_RPI_GPIO 1
#endif
#endif
#ifndef AEB_HAVE_SYS_RPI_GPIO
#define RPI_GPIO_IOMGR (_IOMGR_PRIVATE_BASE + 35)
enum {
    RPI_GPIO_SET_SELECT,
    RPI_GPIO_GET_SELECT,
    RPI_GPIO_WRITE,
    RPI_GPIO_READ,
    RPI_GPIO_ADD_EVENT,
    RPI_GPIO_PWM_SETUP,
    RPI_GPIO_PWM_DUTY,
    RPI_GPIO_PUD
};
enum { RPI_PWM_MODE_PWM = 0, RPI_PWM_MODE_MS = 1 };
enum { RPI_GPIO_FUNC_IN = 0, RPI_GPIO_FUNC_OUT = 1 };
typedef struct {
    struct _io_msg hdr;
    unsigned gpio;
    unsigned value;
} rpi_gpio_msg_t;
typedef struct {
    struct _io_msg hdr;
    unsigned gpio;
    unsigned frequency;
    unsigned range;
    unsigned mode;
} rpi_gpio_pwm_t;
#endif
#define PWM_RANGE 1024u
static int g_fd = -1;
int gpio_open(const char *path)
{
    if (g_fd >= 0) return 0;
    g_fd = open(path, O_RDWR);
    return (g_fd >= 0) ? 0 : -1;
}
bool gpio_is_open(void)
{
    return g_fd >= 0;
}
static void hdr_init(struct _io_msg *h, unsigned subtype)
{
    memset(h, 0, sizeof(*h));
    h->type = _IO_MSG;
    h->mgrid = RPI_GPIO_IOMGR;
    h->subtype = (uint16_t)subtype;
}
static int send_simple(unsigned subtype, int pin, unsigned value, rpi_gpio_msg_t *reply)
{
    if (g_fd < 0) {
        errno = EBADF;
        return -1;
    }
    if (pin < 0 || pin > 27) {
        errno = EINVAL;
        return -1;
    }
    rpi_gpio_msg_t m;
    memset(&m, 0, sizeof m);
    hdr_init(&m.hdr, subtype);
    m.gpio = (unsigned)pin;
    m.value = value;
    long rc =
        reply ? MsgSend(g_fd, &m, sizeof m, reply, sizeof *reply) : MsgSend(g_fd, &m, sizeof m, NULL, 0);
    return (rc == -1) ? -1 : 0;
}
int gpio_set_output(int pin)
{
    return send_simple(RPI_GPIO_SET_SELECT, pin, RPI_GPIO_FUNC_OUT, NULL);
}
int gpio_set_input(int pin)
{
    return send_simple(RPI_GPIO_SET_SELECT, pin, RPI_GPIO_FUNC_IN, NULL);
}
int gpio_set_pull(int pin, int pud)
{
    return send_simple(RPI_GPIO_PUD, pin, (unsigned)pud, NULL);
}
int gpio_write(int pin, int level)
{
    return send_simple(RPI_GPIO_WRITE, pin, level ? 1u : 0u, NULL);
}
int gpio_read(int pin, int *level)
{
    rpi_gpio_msg_t r;
    memset(&r, 0, sizeof r);
    if (send_simple(RPI_GPIO_READ, pin, 1u, &r) != 0) return -1;
    *level = r.value ? 1 : 0;
    return 0;
}
int gpio_pwm_setup(int pin, unsigned freq_hz)
{
    if (g_fd < 0) {
        errno = EBADF;
        return -1;
    }
    rpi_gpio_pwm_t m;
    memset(&m, 0, sizeof m);
    hdr_init(&m.hdr, RPI_GPIO_PWM_SETUP);
    m.gpio = (unsigned)pin;
    m.frequency = freq_hz;
    m.range = PWM_RANGE;
    m.mode = RPI_PWM_MODE_MS;
    return (MsgSend(g_fd, &m, sizeof m, NULL, 0) == -1) ? -1 : 0;
}
int gpio_pwm_duty(int pin, float percent)
{
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    return send_simple(RPI_GPIO_PWM_DUTY, pin, (unsigned)(percent * (float)PWM_RANGE / 100.0f + 0.5f), NULL);
}
void gpio_close(void)
{
    if (g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }
}
