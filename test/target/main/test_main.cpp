#include <cstdio>

#include "test_runner.hpp"

extern "C" void app_main(void)
{
    const int failures = nj_run_all_tests();
    std::printf("NJ_TESTS_DONE failures=%d\n", failures);
}
