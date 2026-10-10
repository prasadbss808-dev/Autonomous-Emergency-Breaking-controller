#ifndef AEB_IPC_H
#define AEB_IPC_H
#include <stdint.h>
#include <sys/neutrino.h>
#include <sys/iomsg.h>
#include "aeb_types.h"
#define PC_TIMER (_PULSE_CODE_MINAVAIL + 1)
#define PC_EMERGENCY (_PULSE_CODE_MINAVAIL + 2)
#define PC_FAULT_SAFE (_PULSE_CODE_MINAVAIL + 3)
#define PC_SHUTDOWN (_PULSE_CODE_MINAVAIL + 4)
#define PC_DEADLINE (_PULSE_CODE_MINAVAIL + 5)
#define PC_GUARD_HB (_PULSE_CODE_MINAVAIL + 10)
#define PC_GUARD_TICK (_PULSE_CODE_MINAVAIL + 11)
#define EMERG_VALUE(seq, st) ((int)((((uint32_t)(seq) & 0x07FFFFFFu) << 4) | ((uint32_t)(st) & 0xFu)))
#define EMERG_SEQ(v) (((uint32_t)(v)) >> 4)
#define EMERG_STATE(v) ((aeb_state_t)(((uint32_t)(v)) & 0xFu))
#define MSG_BRAKE_DECISION (_IO_MAX + 0x100)
#define MSG_GUARD_REGISTER (_IO_MAX + 0x200)
#define MSG_GUARD_CLEAN_EXIT (_IO_MAX + 0x201)
typedef struct {
    uint16_t type;
    uint16_t reserved;
    uint32_t seq;
    uint64_t t_decision_ns;
    uint64_t t_sensor_ns;
    int32_t state;
    motor_cmd_t cmd;
    float v_ref_mps;
    int32_t imu_ok;
} brake_decision_msg_t;
typedef struct {
    uint64_t t_applied_ns;
    uint32_t seq;
    int32_t status;
} brake_reply_t;
typedef struct {
    uint16_t type;
    uint16_t reserved;
    int32_t pid;
} guard_msg_t;
#endif
