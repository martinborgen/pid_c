#include "pid_controller.h"
#include <limits.h>
#include <stdint.h>

int32_t euler_forward(int32_t u, int32_t u_old, uint32_t t, uint32_t t_old) {
    return (t - t_old) * u_old;
}
int32_t euler_backward(int32_t u, int32_t u_old, uint32_t t, uint32_t t_old) {
    return (t - t_old) * u;
}

int32_t trapz(int32_t u, int32_t u_old, uint32_t t, uint32_t t_old) {
    return (t - t_old) * (u + u_old) * 0.5;
}

int32_t update_pid(pid_cont_t* cont,
                   int32_t ref,
                   int32_t feedback,
                   uint32_t millisec) {
    uint32_t millisec_delta = millisec - cont->time_old;
    int32_t error = ref - feedback;
    int32_t output = 0;

    if (millisec_delta == 0 || error == 0) {
        return cont->i_term;
    }

    int32_t p_term = cont->kp * error;
    int32_t i_term = cont->ki *
                     (*cont->integrating_method)(error, cont->error_old,
                                                 millisec, cont->time_old) /
                     1000;  // Because of millisec vs sec
    int32_t d_term = cont->kd * (error - cont->error_old) / millisec_delta;

    if (cont->kp > INT32_MAX / error) {
        p_term = INT32_MAX;
    }
    if (cont->ki > INT32_MAX / error) {
        i_term = INT32_MAX;
    }
    if (cont->kd > INT32_MAX / error) {
        d_term = INT32_MAX;
    }

    if ((p_term > 0 && p_term < INT32_MAX - output) ||
        (p_term < 0 && p_term > INT32_MIN + output)) {
        output += p_term;
    } else if (p_term > 0) {
        output = INT32_MAX;
    } else {
        output = INT32_MIN;
    }

    if ((d_term > 0 && d_term < INT32_MAX - output) ||
        (d_term < 0 && d_term > INT32_MIN + output)) {
        output += d_term;
    } else if (d_term > 0) {
        output = INT32_MAX;
    } else {
        output = INT32_MIN;
    }

    // This return handles integrator windup
    if (output == INT32_MAX || output == INT32_MIN) {
        return output;
    }

    if ((i_term > 0 && i_term < INT32_MAX - cont->i_term) ||
        (i_term < 0 && i_term > INT32_MIN + cont->i_term)) {
        cont->i_term += i_term;
    } else if (i_term > 0) {
        cont->i_term = INT32_MAX;
    } else {
        cont->i_term = INT32_MIN;
    }

    if ((cont->i_term > 0 && i_term < INT32_MAX - output) ||
        (cont->i_term < 0 && i_term > INT32_MIN + output)) {
        output += i_term;
    } else if (cont->i_term > 0) {
        return INT32_MAX;
    } else {
        return INT32_MIN;
    }

    cont->time_old = millisec;
    cont->error_old = error;
    return output;
}

pid_cont_t make_pid_from_params(
    int32_t kp,
    int32_t ki,
    int32_t kd,
    int32_t tf,
    int32_t (*integrating_method)(int32_t, int32_t, uint32_t, uint32_t)) {
    pid_cont_t output = {
        kp, ki, kd, tf, 0, 0, 0, integrating_method,
    };
    return output;
}
