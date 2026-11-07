#include "test_runner.hpp"

int main()
{
    return nj_run_all_tests() == 0 ? 0 : 1;
}
