#ifndef NO_JERKY_MJT_H
#define NO_JERKY_MJT_H

#ifdef __cplusplus
extern "C" {
#endif

 #include <stdint.h>


typedef struct mjt_data
{
    int32_t x0;
    int32_t xT;
    int32_t v0;
    int32_t vT;
    int32_t a0;
    int32_t aT;

    uint32_t T; // [s] trajectory duration
} mjt_data_t;


void compute_mjt_coeff();


#ifdef __cplusplus
}
#endif

#endif  // NO_JERKY_MJT_H 