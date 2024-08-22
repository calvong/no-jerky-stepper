#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "mjt_mutli_level_timestep_lut.h"
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
    // // calculate a T (trajectory duration) based on vmax
    // double a = data->bc.xT * 1.875;
    // data->bc.T = (uint32_t)(a / data->vmax);    // [s] round to the nearest second





    // calculate the mjt coefficients
    data->coeff = compute_mjt_coeff(data->bc);

    // calculate the number of points of the trajectory
    uint32_t n_allocated_pts = (uint32_t)(data->bc.T / MJT_UNIT_TS);

    if (n_allocated_pts > 1000)
    {
        // making sure that the buffer is not too large to start with
        n_allocated_pts = 1000;
    }

    // allocate memory for the dt_array
    data->dt_array = (double*)malloc(n_allocated_pts * sizeof(double));
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

    // calculate the number of points of the trajectory - start with a small buffer
    uint32_t n_allocated_pts = (uint32_t)(data->bc.T / MJT_UNIT_TS)/10;

    if (n_allocated_pts > 1000)
    {
        // making sure that the buffer is not too large to start with
        n_allocated_pts = 1000;
    }

    // allocate memory for the dt_array
    data->dt_array = (double*)malloc(n_allocated_pts * sizeof(double));

    // generate the mjt trajectory
    double x = 0;
    double two_dx = 2 * data->dx;
    uint32_t n = 0;
    double x_stepped = 0;   // x_stepped is the distance covered by the trajectory
    double tt = 0;  // total time in increments of unit_dt

    while (1)
    {
        double ts = multi_stage_binary_mjt_timestep_search(data, &x_stepped, &tt);

        data->dt_array[n] = ts;

        n++;
        if (n >= n_allocated_pts)
        {
            n_allocated_pts *= 2;
            data->dt_array = (double*)realloc(data->dt_array, n_allocated_pts * sizeof(double));
        }

        if (x_stepped >= data->bc.xT)
        {
            break;
        }
    }

    data->n = n;

    // shrink the dt_array to the actual number of points
    data->dt_array = (double*)realloc(data->dt_array, n * sizeof(double));
}


double multi_stage_binary_mjt_timestep_search(mjt_data_t* data, double* x_stepped, double* tt)
{
    const double* ts_lut = NULL;
    double final_ts = 0;
    uint8_t starting_stage = 0;

    // check stage 0 to determine the starting search stage
    double x = 0;
    for (uint8_t i=0; i < TS_LUT_LEVEL0_SIZE; i++)
    {
        // calculate current position
        double ts = ts_lut_level0[i];

        double udt2 = (*tt+ts)*(*tt+ts);
        double udt3 = udt2*(*tt+ts);
        double udt4 = udt3*(*tt+ts);
        double udt5 = udt4*(*tt+ts);

        x = (double) data->coeff.c0 + 
                     data->coeff.c1*(*tt+ts) + 
                     data->coeff.c2*udt2 + 
                     data->coeff.c3*udt3 + 
                     data->coeff.c4*udt4 + 
                     data->coeff.c5*udt5;

        if (x - *x_stepped >= data->dx)
        {
            if (i == 0)
            {
                // this should not be possible??
                printf("multi stage binary search failed: need to start with ts=2e-6, bad step size selection?\n");
                break;
            }
            else
            {
                starting_stage = 6 - i + 1;
                break;
            }
        }
    }

    if (starting_stage == 0)
    {
        // this should not be possible as well :(
        printf("multi stage binary search failed: no suitable timestep found\n");
        final_ts = ts_lut_level0[0];
        *tt += ts_lut_level0[0];
    }
    else
    {
        for (uint8_t stage = starting_stage; stage <= 6; stage++)
        {
            switch (stage)
            {
                case 1:
                    ts_lut = ts_lut_level1;
                    break;
                case 2:
                    ts_lut = ts_lut_level2;
                    break;
                case 3:
                    ts_lut = ts_lut_level3;
                    break;
                case 4:
                    ts_lut = ts_lut_level4;
                    break;
                case 5:
                    ts_lut = ts_lut_level5;
                    break;
                case 6:
                    ts_lut = ts_lut_level6;
                    break;
                default:
                    printf("multi stage binary search not possible to be here 1\n");    // invalid stage
            }

            uint8_t idx = binary_mjt_timestep_index_search(stage, data, *x_stepped, *tt);

            if (idx == 255)
            {
                // skip this stage - no solution found
            }
            else
            {
                final_ts += ts_lut[idx];
                *tt += ts_lut[idx];
            }
        }
    }

    *x_stepped += data->dx;

    return final_ts;
}


uint8_t binary_mjt_timestep_index_search(uint8_t stage, mjt_data_t* data, double x_stepped, double tt)
{
    double x = 0;
    double two_dx = 2 * data->dx;
    double one_and_half_dx = data->dx + data->dx / 2.0;

    int low = 0;
    int high = 0;
    int mid = 0;
    const double* timestep_lut = NULL;
    switch (stage)
    {
        case 1:
            high = TS_LUT_LEVEL1_SIZE - 1;
            timestep_lut = ts_lut_level1;
            break;
        case 2:
            high = TS_LUT_LEVEL2_SIZE - 1;
            timestep_lut = ts_lut_level2;
            break;
        case 3:
            high = TS_LUT_LEVEL3_SIZE - 1;
            timestep_lut = ts_lut_level3;
            break;
        case 4:
            high = TS_LUT_LEVEL4_SIZE - 1;
            timestep_lut = ts_lut_level4;
            break;
        case 5:
            high = TS_LUT_LEVEL5_SIZE - 1;
            timestep_lut = ts_lut_level5;
            break;
        case 6:
            high = TS_LUT_LEVEL6_SIZE - 1;
            timestep_lut = ts_lut_level6;
            break;
        default:
            return 254;    // invalid stage
    }

    while (low <= high)
    {
        // calculate current position
        mid = (int) low + (high - low) / 2;
        double ts = timestep_lut[mid];

        double udt2 = (tt+ts)*(tt+ts);
        double udt3 = udt2*(tt+ts);
        double udt4 = udt3*(tt+ts);
        double udt5 = udt4*(tt+ts);

        x = (double) data->coeff.c0 + 
                     data->coeff.c1*(tt+ts) + 
                     data->coeff.c2*udt2 + 
                     data->coeff.c3*udt3 + 
                     data->coeff.c4*udt4 + 
                     data->coeff.c5*udt5;

        if ((x - x_stepped >= data->dx) && (x - x_stepped <= two_dx))
        {
            high = mid - 1;
        }
        else if (x - x_stepped < data->dx)
        {
            low = mid + 1;
        }
        else
        {
            high = mid - 1;
        }
    }

    if (x - x_stepped < data->dx)
    {
        // making sure that the solution is within the required range
        return mid;
    }
    else
    {
        // if not, choose the previous solution or -1 if not found in this stage
        return mid - 1; // if -1 (solution not found) becomes 255 of uint8_t
    }
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

