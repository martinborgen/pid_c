#include <stdio.h>
#include "acutest.h"
#include "pid_controller.h"

void basic_test() {
    pid_cont_t test =  {
        1,
        2,
        3,
        3,
    };

    int32_t res = update_pid(&test, 1, 1, 1000);
    TEST_CHECK(res == 0);
    res = update_pid(&test, 1, 0, 2000);
    TEST_CHECK(res == 6);
}

TEST_LIST = {
    {"Initial test", basic_test},
    {NULL, NULL},
};