#include <stdint.h>

#ifndef _PID_CONTROLLER_HEADER
#define _PID_CONTROLLER_HEADER

typedef struct pid_controller {
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t tf;
    int32_t iterm;
    uint32_t time_old;
    int32_t error_old;
} pid_cont_t;

int32_t update_pid(pid_cont_t* cont, int32_t ref, int32_t feedback, uint32_t millisec);


#endif