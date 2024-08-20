#include <stdlib.h>
#include <stdio.h>

#include "mjt.h"


/**
 * @brief Generate a minimum jerk trajectory with a maximum velocity constraint.
 * 
 * @param data [mj_data_t*] pointer to the mjt_data_t struct
 *                 input data:
 *                    - vmax [m/s or deg/s] maximum velocity <- this is more intiuitive than acceleration limit
 *                    - dx [m or deg] step size
 *                    - unit_dt [s] smallest time step unit   
 * 
 *                 output data:
 *                    - dt_array [s] mjt trajectory represented by varying time steps (one variable time step for each unit step distance)
 *                    - n number of points of the trajectory
 *                    - bc boundary conditions
 *                    - coeff mjt coefficients  [can be an input as well for non default boundary conditions]
 */
void gen_mjt_with_vmax_constraint(mjt_data_t* data)
{
    // calculate a T (trajectory duration) based on vmax
    double a = data->bc.xT * 1.875;
    data->bc.T = (uint32_t)(a / data->vmax);    // [s] round to the nearest second

    // calculate the mjt coefficients
    data->coeff = compute_mjt_coeff(data->bc);

    // calculate the number of points of the trajectory
    uint32_t n_allocated_pts = (uint32_t)(data->bc.T / data->unit_dt);

    // allocate memory for the dt_array
    data->dt_array = (double*)malloc(n_allocated_pts * sizeof(double));

    // generate the mjt trajectory
    double x = 0;
    uint32_t n = 0;
    uint32_t prev_i = 0;
    for (uint32_t i = 0; i < n_allocated_pts; i++)
    {
        x += data->coeff.c0 + data->coeff.c1 * i + data->coeff.c2 * i * i + data->coeff.c3 * i * i * i + data->coeff.c4 * i * i * i * i + data->coeff.c5 * i * i * i * i * i;

        if (x >= data->dx)
        {
            data->dt_array[n] = (double)(i - prev_i) * data->unit_dt;
            prev_i = i;
            n++;
            x = 0;        
        }
    }

    data->n = n;

    // shrink the dt_array to the actual number of points
    data->dt_array = (double*)realloc(data->dt_array, n * sizeof(double));
}


/**
 * @brief Generate a minimum jerk trajectory with a time constraint.
 * 
 * @param data [mj_data_t*] pointer to the mjt_data_t struct
 *                 input data:
 *                   - vmax [m/s or deg/s] maximum velocity <- this is more intiuitive than acceleration limit
 *                   - dx [m or deg] step size
 *                   - unit_dt [s] smallest time step unit
 *                   - bc.T [s] trajectory duration
 * 
 *                 output data:
 *                   - dt_array [s] mjt trajectory represented by varying time steps (one variable time step for each unit step distance)
 *                   - n number of points of the trajectory
 *                   - coeff mjt coefficients  
 */
void gen_mjt_with_time_constraint(mjt_data_t* data)
{
    // calculate the mjt coefficients
    data->coeff = compute_mjt_coeff(data->bc);

    // calculate the number of points of the trajectory
    uint32_t n_allocated_pts = (uint32_t)(data->bc.T / data->unit_dt);

    // allocate memory for the dt_array
    data->dt_array = (double*)malloc(n_allocated_pts * sizeof(double));

    // generate the mjt trajectory
    double x = 0;
    uint32_t n = 0;
    uint32_t prev_i = 0;
    double x_stepped = 0;   // x_stepped is the distance covered by the trajectory

    double udt2 = data->unit_dt * data->unit_dt;
    double udt3 = data->unit_dt * data->unit_dt * data->unit_dt;
    double udt4 = data->unit_dt * data->unit_dt * data->unit_dt * data->unit_dt;
    double udt5 = data->unit_dt * data->unit_dt * data->unit_dt * data->unit_dt * data->unit_dt;

    for (uint32_t i = 0; i < n_allocated_pts; i++)
    {
        x = (double) data->coeff.c0 + 
                     data->coeff.c1*i*data->unit_dt + 
                     data->coeff.c2*i*i*udt2 + 
                     data->coeff.c3*i*i*i*udt3 + 
                     data->coeff.c4*i*i*i*i*udt4 + 
                     data->coeff.c5*i*i*i*i*i*udt5;

        if (x - x_stepped >= data->dx)
        {
            data->dt_array[n] = (double)(i - prev_i) * data->unit_dt;
            prev_i = i;
            n++;
            x_stepped += data->dx;        
        }
    }

    // compute the final step
    data->dt_array[n] = (double)(n_allocated_pts + 1 - prev_i) * data->unit_dt;
    n++;

    data->n = n;

    // shrink the dt_array to the actual number of points
    data->dt_array = (double*)realloc(data->dt_array, n * sizeof(double));
}


/**
 * @brief Compute the mjt coefficients from the boundary conditions.
 * 
 * @param bc [mjt_bc_t] boundary conditions of the minimum jerk trajectory
 * @return mjt_coeff_t 
 */
mjt_coeff_t compute_mjt_coeff(mjt_bc_t bc)
{
    mjt_coeff_t c;

    uint32_t T = bc.T;
    uint32_t x0 = bc.x0;
    uint32_t xT = bc.xT;
    int32_t v0 = bc.v0;
    int32_t vT = bc.vT;
    int32_t a0 = bc.a0;
    int32_t aT = bc.aT;

    c.c0 = (double) x0;
    c.c1 = (double) v0;
    c.c2 = (double) a0/2.0;
    c.c3 = (double) (-3.0*T*T*a0 + T*T*aT - 12.0*T*v0 - 8.0*T*vT - 20.0*x0 + 20.0*xT)/(2.0*T*T*T);
    c.c4 = (double) (3.0*T*T*a0 - 2.0*T*T*aT + 16.0*T*v0 + 14.0*T*vT + 30.0*x0 - 30.0*xT)/(2.0*T*T*T*T);
    c.c5 = (double) (-T*T*a0 + T*T*aT - 6.0*T*v0 - 6.0*T*vT - 12.0*x0 + 12.0*xT)/(2.0*T*T*T*T*T);

    return c;
}


/**
 * @brief Output the mjt_data_t struct with default values.
 * 
 * @return mjt_data_t
 */
mjt_data_t init_mjt_data()
{
    mjt_data_t output = {
    .vmax = 9999999,
    .dx = 999,
    .unit_dt = 0.00999,
    .dt_array = NULL,
    .n = 0,
    .bc = (mjt_bc_t){
        .x0 = 0,
        .xT = 1,
        .v0 = 0,
        .vT = 0,
        .a0 = 0,
        .aT = 0,
        .T = 1
        },
    .coeff = (mjt_coeff_t){
        .c0 = 0,
        .c1 = 0,
        .c2 = 0,
        .c3 = 0,
        .c4 = 0,
        .c5 = 0
        }
    };

    return output;
}
