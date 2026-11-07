#include "test_runner.hpp"

#include "unity.h"

extern "C" void setUp(void) {}
extern "C" void tearDown(void) {}

int nj_run_all_tests()
{
    UNITY_BEGIN();
    nj_test_motion();
    nj_test_stepgen();
    nj_test_encoder();
    nj_test_retarget();
    nj_test_path();
    nj_test_group();
    nj_test_cruise();
    return UNITY_END();
}
