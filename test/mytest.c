#include <stdio.h>
// #include "acutest.h"
#include "pid_controller.h"

// Currently, unittesting seems unfeasable for the controllers.
// ATM, the author is cross checking versus matlab for controller behaviour

void basic_test() {
    // pid_cont_t test = {
    //     1,
    //     2,
    //     3,
    //     3,
    // };

    // int32_t res = update_pid(&test, 1, 1, 1000);
    // TEST_CHECK(res == 0);
    // res = update_pid(&test, 1, 0, 2000);
    // TEST_CHECK(res == 6);
}

float dummy_load(float volt, float ang_vel) {
    /* Discrete version of an electric motor w. flywheel

    w     0.026009141693124
    - = -----------------------
    U    1 - 0.997503122397460

    where w is angular vel, U is voltage
    Transfer function from voltage to angular vel
    For Ts = 0.001 secods. */
    const float Ts = 0.001;

    float num0 = 0.026009141693124;
    float den0 = 1;
    float den1 = -0.997503122397460;

    return (ang_vel * den1 * -1) + num0 * volt;
}

void test_w_dummy_load() {
    pid_cont_t test = make_pid_from_params(1, 2, 1, 0, &euler_forward);

    float w = 0;
    float res[200];
    float scale_factor = 100000;
    float ref = 1;
    for (int i = 0; i < 200; i++) {
        int u = update_pid(&test, ref * scale_factor, w * scale_factor, i + 1);
        float u_scaled = u / scale_factor;
        w = dummy_load(u_scaled, w);
        res[i] = w;
        printf("%f\n", w);
    }
}

void main() {
    test_w_dummy_load();
}

// TEST_LIST = {
//     {"Initial test", basic_test},
//     {NULL, NULL},
// };