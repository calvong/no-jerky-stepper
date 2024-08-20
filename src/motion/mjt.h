#ifndef NO_JERKY_MJT_H
#define NO_JERKY_MJT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


typedef struct mjt_bc
{
    uint32_t x0;
    uint32_t xT;
    int32_t v0;
    int32_t vT;
    int32_t a0;
    int32_t aT;

    uint32_t T; // [s] trajectory duration
} mjt_bc_t;


typedef struct mjt_coeff
{
    double c0;
    double c1;
    double c2;
    double c3;
    double c4;
    double c5;
} mjt_coeff_t;


typedef struct mjt_data
{
    // input data
    uint32_t vmax; // [m/s or deg/s] maximum velocity <- this is more intiuitive than acceleration limit

    double dx;          // [m or deg] step size
    double unit_dt;     // [s] smallest time step unit

    // generated data
    double* dt_array;   // [s] mjt trajectory represented by varying time steps (one variable time step for each unit step distance)
    uint32_t n;         // number of points of the trajectory
    mjt_bc_t bc;        // boundary conditions
    mjt_coeff_t coeff;  // mjt coefficients
} mjt_data_t;


void gen_mjt_with_vmax_constraint(mjt_data_t* data);
void gen_mjt_with_time_constraint(mjt_data_t* data);
mjt_coeff_t compute_mjt_coeff(mjt_bc_t bc);
mjt_data_t init_mjt_data();

#ifdef __cplusplus
}
#endif

#endif  // NO_JERKY_MJT_H 