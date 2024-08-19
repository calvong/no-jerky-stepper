#include "mjt.h"

void gen_mjt()
{
    // c0 = x0
    // c1 = v0
    // c2 = a0/2
    // c3 = (-3*T*T*a0 + T*T*aT - 12*T*v0 - 8*T*vT - 20*x0 + 20*xT)/(2*T*T*T)
    // c4 = (3*T*T*a0 - 2*T*T*aT + 16*T*v0 + 14*T*vT + 30*x0 - 30*xT)/(2*T*T*T*T)
    // c5 = (-T*T*a0 + T*T*aT - 6*T*v0 - 6*T*vT - 12*x0 + 12*xT)/(2*T*T*T*T*T)

    // t = np.linspace(0, T, 100)
    // x = c0 + c1*t + c2*t*t + c3*t*t*t + c4*t*t*t*t + c5*t*t*t*t*t
    // v = c1 + 2*c2*t + 3*c3*t*t + 4*c4*t*t*t + 5*c5*t*t*t*t
    // a = 2*c2 + 6*c3*t + 12*c4*t*t + 20*c5*t*t*t
    // j = 6*c3 + 24*c4*t + 60*c5*t*t
}