#include <stdint.h>

#ifndef _PID_CONTROLLER_HEADER
#define _PID_CONTROLLER_HEADER

// Struct that holds the parameters for the controller.
// `kp, ki, kd`, the usual gains in parallel form
// `tf` is the filter time constant CURRENTLY NOT IMPLEMENTED
// `iterm, time_old, error_old` are not suppoased to be set by the user
// `integrating_method` is a function pointer to the function desired for
// integration for instance any of the `euler_forward`, `euler_backward` or
// `trapz` functions
typedef struct pid_controller {
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t tf;
    int32_t i_term;
    uint32_t time_old;
    int32_t error_old;
    int32_t (*integrating_method)(int32_t, int32_t, uint32_t, uint32_t);
} pid_cont_t;

// Calculates the integral as `(t - t_old) * u_old`
int32_t euler_forward(int32_t u, int32_t u_old, uint32_t t, uint32_t t_old);

// Calculates the integral as `(t - t_old) * u`
int32_t euler_backward(int32_t u, int32_t u_old, uint32_t t, uint32_t t_old);

// Calculates the integral as `(t - t_old) * (u - u_old) / 2`
int32_t trapz(int32_t u, int32_t u_old, uint32_t t, uint32_t t_old);

// Updates the controller, where:
// `cont` is the controller to be updated
// `ref` is the reference value
// `feedback` is the value from the feedback (positive sign)
// `millisec` is the absolute time in milliseconds
int32_t update_pid(pid_cont_t* cont,
                   int32_t ref,
                   int32_t feedback,
                   uint32_t millisec);

// Creates a pid from params, returns the struct by-value
// `kp, ki, kd` the usual gains in parallel form
// `tf` is filter time constant NOT IMPLEMENTED
// `integrating_method` is a function pointer to the function desired for
// integration for instance any of the `euler_forward`, `euler_backward` or
// `trapz` functions
pid_cont_t make_pid_from_params(
    int32_t kp,
    int32_t ki,
    int32_t kd,
    int32_t tf,
    int32_t (*integrating_method)(int32_t, int32_t, uint32_t, uint32_t));

#endif