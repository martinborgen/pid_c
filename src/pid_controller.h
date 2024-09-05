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
    int32_t( *integrating_method)(int32_t, int32_t, uint32_t, uint32_t);
} pid_cont_t;

int32_t euler_forward(int32_t u, int32_t u_old, uint32_t t, uint32_t t_old);
int32_t euler_backward(int32_t u, int32_t u_old, uint32_t t, uint32_t t_old);
int32_t trapz(int32_t u, int32_t u_old, uint32_t t, uint32_t t_old);

int32_t update_pid(pid_cont_t* cont, int32_t ref, int32_t feedback, uint32_t millisec);
pid_cont_t make_pid_from_params(int32_t kp, int32_t ki, int32_t kd, int32_t tf, int32_t(*integrating_method)(int32_t, int32_t, uint32_t, uint32_t));


#endif