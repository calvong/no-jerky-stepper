#include "motion/path_planner.hpp"

#include <algorithm>
#include <cmath>

#include "motion/min_jerk.hpp"

namespace nj {

namespace {

// Jerk and snap at both ends of the min-jerk quintic with boundary vector
// b = [p0, v0, a0, p1, v1, a1]: out = [j(0), s(0), j(T), s(T)]. Linear in b.
void end_derivs(double T, const double b[6], double out[4])
{
    const State s0{b[0], b[1], b[2]};
    const State s1{b[3], b[4], b[5]};
    const PolySegment s = min_jerk(s0, s1, T);
    out[0] = s.j(0.0);
    out[1] = s.snap(0.0);
    out[2] = s.j(T);
    out[3] = s.snap(T);
}

// 4x6 matrix of the linear map above.
void end_deriv_matrix(double T, double M[4][6])
{
    for (int i = 0; i < 6; ++i) {
        double b[6] = {0, 0, 0, 0, 0, 0};
        b[i] = 1.0;
        double o[4];
        end_derivs(T, b, o);
        for (int r = 0; r < 4; ++r) {
            M[r][i] = o[r];
        }
    }
}

// 2x2 helpers, row-major m = [m00, m01, m10, m11].
bool inv2(const double m[4], double out[4])
{
    const double det = m[0] * m[3] - m[1] * m[2];
    const double scale = std::fabs(m[0] * m[3]) + std::fabs(m[1] * m[2]);
    if (!(std::fabs(det) > 1e-14 * scale) || !std::isfinite(det)) {
        return false;
    }
    out[0] = m[3] / det;
    out[1] = -m[1] / det;
    out[2] = -m[2] / det;
    out[3] = m[0] / det;
    return true;
}
void mul2(const double a[4], const double b[4], double out[4])
{
    const double r0 = a[0] * b[0] + a[1] * b[2];
    const double r1 = a[0] * b[1] + a[1] * b[3];
    const double r2 = a[2] * b[0] + a[3] * b[2];
    const double r3 = a[2] * b[1] + a[3] * b[3];
    out[0] = r0;
    out[1] = r1;
    out[2] = r2;
    out[3] = r3;
}
void mulv2(const double a[4], const double v[2], double out[2])
{
    const double r0 = a[0] * v[0] + a[1] * v[1];
    const double r1 = a[2] * v[0] + a[3] * v[1];
    out[0] = r0;
    out[1] = r1;
}

bool knot_fixed(const PathProblem& p, int k)
{
    return !std::isnan(p.vel[k]) || !std::isnan(p.acc[k]);
}
double or_zero(double x)
{
    return std::isnan(x) ? 0.0 : x;
}

void via_stop(const PathProblem& p, PathWork& w)
{
    for (int k = 0; k < p.n - 1; ++k) {
        w.v[k] = or_zero(p.vel[k]);
        w.a[k] = or_zero(p.acc[k]);
    }
}

void via_heuristic(const PathProblem& p, const double* T, PathWork& w)
{
    for (int k = 0; k < p.n - 1; ++k) {
        const double prev = k == 0 ? p.start.p : p.pos[k - 1];
        const double s0 = (p.pos[k] - prev) / T[k];
        const double s1 = (p.pos[k + 1] - p.pos[k]) / T[k + 1];
        double v = 0.0;
        if (s0 * s1 > 0.0) {
            v = (s0 * T[k + 1] + s1 * T[k]) / (T[k] + T[k + 1]);
        }
        w.v[k] = std::isnan(p.vel[k]) ? v : p.vel[k];
        w.a[k] = or_zero(p.acc[k]);
    }
}

// Minimum total jerk^2: continuity of jerk and snap at each intermediate knot.
bool via_optimal(const PathProblem& p, const double* T, PathWork& w)
{
    const int m = p.n - 1;  // unknown knots 0..m-1, each x_k = (v_k, a_k)
    if (m <= 0) {
        return true;
    }
    double Mk[4][6];
    double Mk1[4][6];
    end_deriv_matrix(T[0], Mk);
    for (int k = 0; k < m; ++k) {
        end_deriv_matrix(T[k + 1], Mk1);
        double* A = w.A[k];
        double* B = w.B[k];
        double* C = w.C[k];
        double* d = w.d[k];
        if (knot_fixed(p, k)) {
            A[0] = A[1] = A[2] = A[3] = 0.0;
            C[0] = C[1] = C[2] = C[3] = 0.0;
            B[0] = 1.0;
            B[1] = 0.0;
            B[2] = 0.0;
            B[3] = 1.0;
            d[0] = or_zero(p.vel[k]);
            d[1] = or_zero(p.acc[k]);
        } else {
            const double p_prev = k == 0 ? p.start.p : p.pos[k - 1];
            const double p_k = p.pos[k];
            const double p_next = p.pos[k + 1];
            for (int r = 0; r < 2; ++r) {
                const double* er = Mk[2 + r];  // end of segment k
                const double* sr = Mk1[r];     // start of segment k+1
                A[2 * r + 0] = er[1];
                A[2 * r + 1] = er[2];
                B[2 * r + 0] = er[4] - sr[1];
                B[2 * r + 1] = er[5] - sr[2];
                C[2 * r + 0] = -sr[4];
                C[2 * r + 1] = -sr[5];
                const double cst = er[0] * p_prev + er[3] * p_k - sr[0] * p_k - sr[3] * p_next;
                d[r] = -cst;
            }
            if (k == 0) {
                // x_{-1} is the fixed start state.
                const double x[2] = {p.start.v, p.start.a};
                double ax[2];
                mulv2(A, x, ax);
                d[0] -= ax[0];
                d[1] -= ax[1];
                A[0] = A[1] = A[2] = A[3] = 0.0;
            }
            if (k == m - 1) {
                C[0] = C[1] = C[2] = C[3] = 0.0;  // final knot is at rest
            }
        }
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 6; ++c) {
                Mk[r][c] = Mk1[r][c];
            }
        }
    }

    // Block Thomas algorithm.
    double inv[4];
    double W[4];
    double tmp[4];
    double tv[2];
    for (int k = 1; k < m; ++k) {
        if (!inv2(w.B[k - 1], inv)) {
            return false;
        }
        mul2(w.A[k], inv, W);
        mul2(W, w.C[k - 1], tmp);
        for (int i = 0; i < 4; ++i) {
            w.B[k][i] -= tmp[i];
        }
        mulv2(W, w.d[k - 1], tv);
        w.d[k][0] -= tv[0];
        w.d[k][1] -= tv[1];
    }
    if (!inv2(w.B[m - 1], inv)) {
        return false;
    }
    double x[2];
    mulv2(inv, w.d[m - 1], x);
    w.v[m - 1] = x[0];
    w.a[m - 1] = x[1];
    for (int k = m - 2; k >= 0; --k) {
        const double xn[2] = {w.v[k + 1], w.a[k + 1]};
        mulv2(w.C[k], xn, tv);
        const double r[2] = {w.d[k][0] - tv[0], w.d[k][1] - tv[1]};
        if (!inv2(w.B[k], inv)) {
            return false;
        }
        mulv2(inv, r, x);
        w.v[k] = x[0];
        w.a[k] = x[1];
    }
    for (int k = 0; k < m; ++k) {
        if (!std::isfinite(w.v[k]) || !std::isfinite(w.a[k])) {
            return false;
        }
    }
    return true;
}

}  // namespace

void base_durations(const PathProblem& prob, double* T0)
{
    for (int k = 0; k < prob.n; ++k) {
        if (prob.duration[k] > 0.0) {
            T0[k] = prob.duration[k];
        } else {
            const double prev = k == 0 ? prob.start.p : prob.pos[k - 1];
            T0[k] = std::max(rest_to_rest_duration(prob.pos[k] - prev, prob.limits), 0.01);
        }
    }
}

void build_path(const PathProblem& prob, const double* T, PathSolution& sol, PathWork& work)
{
    const int n = prob.n;
    for (int k = 0; k < n; ++k) {
        sol.T[k] = std::max(T[k], kMinSegmentDuration);
    }
    switch (prob.policy) {
    case ViaPolicy::Stop:
        via_stop(prob, work);
        break;
    case ViaPolicy::Heuristic:
        via_heuristic(prob, sol.T, work);
        break;
    case ViaPolicy::Optimal:
        if (!via_optimal(prob, sol.T, work)) {
            via_heuristic(prob, sol.T, work);  // singular system: fall back
        }
        break;
    }
    if (n > 0) {
        work.v[n - 1] = 0.0;  // paths always end at rest
        work.a[n - 1] = 0.0;
    }
    State prev = prob.start;
    for (int k = 0; k < n; ++k) {
        const State s{prob.pos[k], work.v[k], work.a[k]};
        sol.seg[k] = min_jerk(prev, s, sol.T[k]);
        prev = s;
    }
    sol.n = n;
}

double path_limit_ratio(const PathProblem& prob, const PathSolution& sol)
{
    const Limits eff = limits_for_start(prob.limits, prob.start);
    double r = 0.0;
    for (int k = 0; k < sol.n; ++k) {
        if (prob.duration[k] > 0.0) {
            continue;  // explicit durations are not limit-checked
        }
        r = std::max(r, limit_ratio(peaks(sol.seg[k]), eff));
    }
    return r;
}

Status solve_path(const PathProblem& prob, PathSolution& sol, PathWork& work)
{
    if (prob.n <= 0 || prob.n > cfg::kPathCapacity) {
        return prob.n <= 0 ? Status::InvalidArg : Status::NoMem;
    }
    bool any_auto = false;
    for (int k = 0; k < prob.n; ++k) {
        if (!std::isfinite(prob.pos[k])) {
            return Status::InvalidArg;
        }
        any_auto = any_auto || !(prob.duration[k] > 0.0);
    }
    if (any_auto && !prob.limits.any()) {
        return Status::InvalidArg;
    }
    base_durations(prob, work.T0);
    if (!any_auto) {
        build_path(prob, work.T0, sol, work);
        return Status::Ok;
    }

    double T[cfg::kPathCapacity];
    auto apply = [&](double s) {
        for (int k = 0; k < prob.n; ++k) {
            T[k] = prob.duration[k] > 0.0 ? prob.duration[k] : s * work.T0[k];
        }
        build_path(prob, T, sol, work);
    };
    auto ratio = [&](double s) {
        apply(s);
        return path_limit_ratio(prob, sol);
    };
    bool ok = true;
    const double s = search_min_scale(ratio, 1.0, 1e-3, 1e3, &ok);
    apply(s);
    return ok ? Status::Ok : Status::Infeasible;
}

}  // namespace nj
