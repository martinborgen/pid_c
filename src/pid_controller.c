#include <stdint.h>
#include <limits.h>
#include "pid_controller.h"

int32_t euler_forward(int32_t u_old, uint32_t t, uint32_t t_old) {
    return (t - t_old) * u_old;
}
int32_t euler_backward(int32_t u, int32_t u_old, uint32_t t, uint32_t t_old) {
    return (t - t_old) * u;
}

int32_t trapz(int32_t u, int32_t u_old, uint32_t t, uint32_t t_old) {
    return (t - t_old) * (u + u_old) * 0.5;
}

int32_t update_pid(pid_cont_t* cont, int32_t ref, int32_t feedback, uint32_t millisec) {
    uint32_t millisec_delta = millisec - cont->time_old;
    int32_t error = ref - feedback;
    int32_t output = 0;

    int32_t p_term = cont->kp * error;
    int32_t i_term = cont->ki * euler_forward(cont->u_old, millisec, cont->time_old);
    int32_t d_term = cont->kd * error / millisec_delta; // TODO Check for zero division

    output += cont->iterm;
    output += d_term;

    cont->iterm += i_term; // TODO Handle overflow here too
    cont->time_old = millisec;
    cont->u_old = output;
    return output;
}
