#ifndef MJT_MULTI_LEVEL_TIMESTEP_LUT_H
#define MJT_MULTI_LEVEL_TIMESTEP_LUT_H

#define MJT_UNIT_TS 2e-6
#define TS_LUT_LEVEL0_SIZE 7
#define TS_LUT_LEVEL1_SIZE 9
#define TS_LUT_LEVEL2_SIZE 9
#define TS_LUT_LEVEL3_SIZE 9
#define TS_LUT_LEVEL4_SIZE 9
#define TS_LUT_LEVEL5_SIZE 9
#define TS_LUT_LEVEL6_SIZE 5

const double ts_lut_level0[] = {2e-06,1e-05,0.0001,0.001,0.01,0.1,1};
const double ts_lut_level1[] = {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9};
const double ts_lut_level2[] = {0.01,0.02,0.03,0.04,0.05,0.06,0.07,0.08,0.09};
const double ts_lut_level3[] = {0.001,0.002,0.003,0.004,0.005,0.006,0.007,0.008,0.009};
const double ts_lut_level4[] = {0.0001,0.0002,0.0003,0.0004,0.0005,0.0006,0.0007,0.0008,0.0009};
const double ts_lut_level5[] = {1e-05,2e-05,3e-05,4e-05,5e-05,6e-05,7e-05,8e-05,9e-05};
const double ts_lut_level6[] = {2e-06,4e-06,6e-06,8e-06,1e-05};

#endif