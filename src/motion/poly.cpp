#include "motion/poly.hpp"

#include <algorithm>
#include <cmath>

namespace nj {

PolySegment make_hold(double p, double T)
{
    PolySegment s;
    s.T = T;
    s.c[0] = p;
    return s;
}

namespace {

double horner(const double* k, int deg, double x)
{
    double r = k[deg];
    for (int i = deg - 1; i >= 0; --i) {
        r = r * x + k[i];
    }
    return r;
}

void add_root(double x, double* out, int& n)
{
    if (n > 0 && std::fabs(x - out[n - 1]) <= 1e-12 * (1.0 + std::fabs(x))) {
        return;
    }
    out[n++] = x;
}

}  // namespace

int roots_in(const double* k, int deg, double lo, double hi, double* out)
{
    while (deg > 0 && k[deg] == 0.0) {
        --deg;
    }
    if (deg <= 0 || !(hi > lo)) {
        return 0;
    }
    if (deg == 1) {
        const double x = -k[0] / k[1];
        if (x >= lo && x <= hi) {
            out[0] = x;
            return 1;
        }
        return 0;
    }

    // Critical points (roots of the derivative) split [lo, hi] into monotone pieces;
    // each piece holds at most one root, found by bisection.
    double d[5];
    for (int i = 0; i < deg; ++i) {
        d[i] = (i + 1) * k[i + 1];
    }
    double crit[5];
    const int nc = roots_in(d, deg - 1, lo, hi, crit);

    double pts[7];
    int np = 0;
    pts[np++] = lo;
    for (int i = 0; i < nc; ++i) {
        if (crit[i] > pts[np - 1] && crit[i] < hi) {
            pts[np++] = crit[i];
        }
    }
    pts[np++] = hi;

    int n = 0;
    for (int i = 0; i + 1 < np; ++i) {
        double a = pts[i];
        double b = pts[i + 1];
        double fa = horner(k, deg, a);
        const double fb = horner(k, deg, b);
        if (fa == 0.0) {
            add_root(a, out, n);
            continue;
        }
        if ((fa < 0.0) == (fb < 0.0) || fb == 0.0) {
            continue;  // no sign change inside (an fb == 0 root is picked up as the next fa)
        }
        for (int it = 0; it < 200; ++it) {
            const double m = 0.5 * (a + b);
            if (m <= a || m >= b) {
                break;
            }
            const double fm = horner(k, deg, m);
            if (fm == 0.0) {
                a = b = m;
                break;
            }
            if ((fm < 0.0) == (fa < 0.0)) {
                a = m;
                fa = fm;
            } else {
                b = m;
            }
        }
        add_root(0.5 * (a + b), out, n);
    }
    if (horner(k, deg, hi) == 0.0) {
        add_root(hi, out, n);
    }
    return n;
}

Peaks peaks(const PolySegment& s)
{
    Peaks pk;
    const double T = s.T;
    double r[5];

    // |v| is extreme at the ends or where a(t) = 0.
    pk.v = std::max(std::fabs(s.v(0)), std::fabs(s.v(T)));
    const double ka[4] = {2 * s.c[2], 6 * s.c[3], 12 * s.c[4], 20 * s.c[5]};
    for (int i = 0, n = roots_in(ka, 3, 0.0, T, r); i < n; ++i) {
        pk.v = std::max(pk.v, std::fabs(s.v(r[i])));
    }

    // |a| is extreme at the ends or where j(t) = 0.
    pk.a = std::max(std::fabs(s.a(0)), std::fabs(s.a(T)));
    const double kj[3] = {6 * s.c[3], 24 * s.c[4], 60 * s.c[5]};
    for (int i = 0, n = roots_in(kj, 2, 0.0, T, r); i < n; ++i) {
        pk.a = std::max(pk.a, std::fabs(s.a(r[i])));
    }

    // |j| is extreme at the ends or where snap(t) = 0.
    pk.j = std::max(std::fabs(s.j(0)), std::fabs(s.j(T)));
    const double ks[2] = {24 * s.c[4], 120 * s.c[5]};
    for (int i = 0, n = roots_in(ks, 1, 0.0, T, r); i < n; ++i) {
        pk.j = std::max(pk.j, std::fabs(s.j(r[i])));
    }
    return pk;
}

double limit_ratio(const Peaks& pk, const Limits& lim)
{
    double r = 0.0;
    if (lim.v_max > 0.0) {
        r = std::max(r, pk.v / lim.v_max);
    }
    if (lim.a_max > 0.0) {
        r = std::max(r, pk.a / lim.a_max);
    }
    if (lim.j_max > 0.0) {
        r = std::max(r, pk.j / lim.j_max);
    }
    return r;
}

Limits limits_for_start(const Limits& lim, const State& start)
{
    Limits out = lim;
    if (out.v_max > 0.0) {
        out.v_max = std::max(out.v_max, std::fabs(start.v) * (1.0 + 1e-9));
    }
    if (out.a_max > 0.0) {
        out.a_max = std::max(out.a_max, std::fabs(start.a) * (1.0 + 1e-9));
    }
    return out;
}

}  // namespace nj
