#include <stdint.h>
#include <limits.h>
#include "pid_controller.h"

int32_t update_pid(pid_cont_t* cont, int32_t ref, int32_t feedback, uint32_t millisec) {
    uint32_t delta_t = millisec - cont->old_t;
    int32_t error = ref - feedback;
    int32_t output = 0;

    int32_t p_term = cont->kp * error;
    int32_t i_term = cont->ki * error;
    int32_t d_term = cont->kd * error;

    cont->iterm += i_term;
    cont->old_t = millisec;

    // Overflow handling. Always int max? TODO: HOw to detect underflow?
    if (error != 0 && p_term / error != cont->kp) {
        return INT32_MAX;
    }

    if (error != 0 && i_term / error != cont->ki) {
        return INT32_MAX;
    }

    if (error != 0 && d_term / error != cont->kd) {
        return INT32_MAX;
    }

    if (p_term < INT32_MAX - output) {
        output += p_term;
    } else {
        return INT32_MAX;
    }

    if (i_term < INT32_MAX - output) {
        output += i_term;
    } else {
        return INT32_MAX;
    }

    if (d_term < INT32_MAX - output) {
        output += d_term;
    } else {
        return INT32_MAX;
    }

    return output;
}
