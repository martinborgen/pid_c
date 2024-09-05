#include <stdint.h>
#include <limits.h>
#include "pid_controller.h"

int32_t update_pid(pid_cont_t* cont, int32_t ref, int32_t feedback, uint32_t millisec) {
    uint32_t millisec_delta = millisec - cont->time_old;
    int32_t error = ref - feedback;
    int32_t output = 0;

    int32_t p_term = cont->kp * error;
    int32_t i_term = cont->ki * error * millisec_delta;
    int32_t d_term = cont->kd * error / millisec_delta; // TODO Check for zero division


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

    // iterm = ki * iterm old 
    output += cont->iterm;
    // if (cont->iterm < INT32_MAX - output) {
    // } else {
    //     return INT32_MAX;
    // }

    output += d_term;
    // if (d_term < INT32_MAX - output) {
    // } else {
    //     return INT32_MAX;
    // }

    cont->iterm += i_term; // TODO Handle overflow here too
    cont->time_old = millisec;

    return output;
}
