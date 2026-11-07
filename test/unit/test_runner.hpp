#pragma once

void nj_test_motion();
void nj_test_stepgen();
void nj_test_encoder();
void nj_test_retarget();
void nj_test_path();
void nj_test_group();

/// Runs every unit test; returns the number of failures.
int nj_run_all_tests();
