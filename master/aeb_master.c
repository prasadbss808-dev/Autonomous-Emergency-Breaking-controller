#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
extern char **environ;
#define DEFAULT_DIR "/data/home/qnxuser/aeb"
static int exists(const char *p)
{
    struct stat st;
    return stat(p, &st) == 0;
}
static int is_exec(const char *p)
{
    return access(p, X_OK) == 0;
}
static int run(const char *cmd)
{
    printf("[master] $ %s\n", cmd);
    fflush(stdout);
    int rc = system(cmd);
    return (rc == -1) ? -1 : (WIFEXITED(rc) ? WEXITSTATUS(rc) : -1);
}
static pid_t spawn_bg(const char *log, char *const argv[])
{
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_addopen(&fa, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
    posix_spawn_file_actions_addopen(&fa, STDOUT_FILENO, log, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    posix_spawn_file_actions_adddup2(&fa, STDOUT_FILENO, STDERR_FILENO);
    pid_t pid = -1;
    int e = posix_spawn(&pid, argv[0], &fa, NULL, argv, environ);
    posix_spawn_file_actions_destroy(&fa);
    if (e != 0) {
        printf("[master] cannot start %s: %s\n", argv[0], strerror(e));
        return -1;
    }
    printf("[master] started %-14s pid %d  (log: %s)\n", argv[0], (int)pid, log);
    return pid;
}
static void ip_hint(void)
{
    FILE *p =
        popen("ifconfig 2>/dev/null | grep 'inet ' | grep -v 127.0.0.1 | head -n 1 | awk '{print $2}'", "r");
    char ip[64] = "";
    if (p) {
        if (!fgets(ip, sizeof ip, p)) ip[0] = '\0';
        pclose(p);
    }
    ip[strcspn(ip, "\r\n")] = '\0';
    printf("\n[master] ===== DASHBOARD: http://%s:8080 =====\n\n", ip[0] ? ip : "<pi-ip>");
}
int main(int argc, char **argv)
{
    int soft = 0, vision = 1, dash = 1;
    const char *camera = "real";
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "soft"))
            soft = 1;
        else if (!strcmp(argv[i], "novision"))
            vision = 0;
        else if (!strcmp(argv[i], "nodash"))
            dash = 0;
        else if (!strcmp(argv[i], "sim"))
            camera = "sim";
        else
            printf("[master] unknown argument '%s' (use: soft novision sim nodash)\n", argv[i]);
    }
    setvbuf(stdout, NULL, _IOLBF, 0);
    const char *dir = getenv("AEB_DIR") ? getenv("AEB_DIR") : DEFAULT_DIR;
    if (chdir(dir) != 0) {
        printf("[master] cannot cd to %s: %s\n"
               "[master] deploy first:  make deploy-all TARGET=qnxuser@<pi-ip>  (or set AEB_DIR)\n",
               dir, strerror(errno));
        return 1;
    }
    printf("[master] QNX AEB master launcher - folder %s\n", dir);
    if (geteuid() == 0 && getuid() != 0 && setuid(0) != 0)
        printf("[master] setuid(0) failed: %s\n", strerror(errno));
    if (geteuid() != 0)
        printf("[master] WARNING: not running as root - GPIO/UART/priorities will fail.\n"
               "[master]          In Momentics connect to the target as root, or run: su -c ./aeb_master\n");
    const char *need[] = {"./aeb", "./aeb_guard", "./start_uart.sh", "./aeb.conf"};
    for (unsigned i = 0; i < sizeof need / sizeof need[0]; i++)
        if (!exists(need[i])) {
            printf("[master] MISSING %s/%s - run make deploy-all\n", dir, need[i] + 2);
            return 1;
        }
    run("chmod +x aeb aeb_guard aeb_dash aeb_vision *.sh 2>/dev/null");
    run("slay -f aeb aeb_guard aeb_vision aeb_dash >/dev/null 2>&1");
    unlink("stop.flag");
    sleep(1);
    run("gpio-rp1 set 2,3 a3 >/dev/null 2>&1");
    if (run("./start_uart.sh") != 0) {
        printf("[master] radar UART failed - check start_uart.sh output\n");
        return 1;
    }
    if (vision && !(is_exec("./aeb_vision") && exists("./models/mobilenet_iter_73000.caffemodel"))) {
        printf("[master] camera files missing - running without camera detection\n");
        vision = 0;
    }
    if (vision && is_exec("./camera_setup.sh")) {
        char cmd[64];
        snprintf(cmd, sizeof cmd, "./camera_setup.sh %s", camera);
        run(cmd);
    }
    if (dash) {
        if (is_exec("./aeb_dash") && exists("./www/index.html")) {
            char *a[] = {"./aeb_dash", "-w", "./www", NULL};
            spawn_bg("dash.log", a);
        } else {
            printf("[master] dashboard files missing (aeb_dash, www/index.html) - no dashboard\n");
            dash = 0;
        }
    }
    if (vision) {
        char *a[] = {"/bin/sh", "./run_vision.sh", "-q", NULL};
        if (spawn_bg("vision.log", a) > 0)
            printf("[master] camera detections (OBJECT DETECTED ...) also in vision.log\n");
    }
    char *g[] = {"./aeb_guard", NULL};
    pid_t guard = spawn_bg("guard.log", g);
    sleep(1);
    int guard_ok = guard > 0 && waitpid(guard, NULL, WNOHANG) == 0;
    if (dash) ip_hint();
    char *aeb[12];
    int n = 0;
    aeb[n++] = "./aeb";
    aeb[n++] = "-m";
    aeb[n++] = "full";
    if (guard_ok) aeb[n++] = "-G";
    aeb[n++] = "-c";
    aeb[n++] = "aeb.conf";
    aeb[n++] = "-l";
    aeb[n++] = "aeb_log.csv";
    if (soft) {
        aeb[n++] = "-p";
        aeb[n++] = "soft";
    }
    aeb[n] = NULL;
    printf("[master] starting AEB:");
    for (int i = 0; i < n; i++) printf(" %s", aeb[i]);
    printf(
        "\n[master] commands in this console: go | stop | estop | status | quit   (Terminate/Ctrl-C = safe stop)\n\n");
    fflush(stdout);
    execv("./aeb", aeb);
    printf("[master] exec ./aeb failed: %s\n", strerror(errno));
    return 1;
}
