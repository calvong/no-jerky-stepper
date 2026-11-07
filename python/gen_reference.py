"""Symbolic derivations behind src/motion (run: python python/gen_reference.py).

1. Min-jerk quintic with full boundary conditions: coefficients used by min_jerk().
2. Free-end-position min-jerk (smooth stop / velocity change): the natural boundary condition
   for a free x(T) is x^(5)(T) = 0, and x^(6) = 0 everywhere, so c5 = 0 -> a quartic.
   Coefficients used by min_jerk_free_end(), plus the stopping distance.
3. Rest-to-rest peak factors: v_peak = 15/8 D/T, a_peak = 10/sqrt(3) D/T^2, j_peak = 60 D/T^3,
   used by rest_to_rest_duration().
4. Optimality conditions for via points (ViaPolicy::Optimal): d/dv_k and d/da_k of the total
   cost are proportional to the jumps in snap and jerk at knot k, so the optimum is the C4 spline.
"""
import sympy as sp

t, T = sp.symbols("t T", positive=True)
x0, v0, a0, xT, vT, aT = sp.symbols("x0 v0 a0 xT vT aT", real=True)
c = sp.symbols("c0:6", real=True)


def quintic():
    x = sum(c[i] * t**i for i in range(6))
    eqs = [
        x.subs(t, 0) - x0, sp.diff(x, t).subs(t, 0) - v0, sp.diff(x, t, 2).subs(t, 0) - a0,
        x.subs(t, T) - xT, sp.diff(x, t).subs(t, T) - vT, sp.diff(x, t, 2).subs(t, T) - aT,
    ]
    sol = sp.solve(eqs, c, dict=True)[0]
    print("== min-jerk quintic coefficients")
    for i in range(6):
        print(f"c{i} =", sp.simplify(sol[c[i]]))
    return x.subs(sol)


def free_end():
    # Minimise integral(j^2) with x(T) free: x^(6) = 0 and x^(5)(T) = 0  ->  c5 = 0.
    x = sum(c[i] * t**i for i in range(5))
    eqs = [
        x.subs(t, 0) - x0, sp.diff(x, t).subs(t, 0) - v0, sp.diff(x, t, 2).subs(t, 0) - a0,
        sp.diff(x, t).subs(t, T) - vT, sp.diff(x, t, 2).subs(t, T),
    ]
    sol = sp.solve(eqs, c[:5], dict=True)[0]
    print("\n== free-end quartic (a(T) = 0, v(T) = vT)")
    for i in range(5):
        print(f"c{i} =", sp.simplify(sol[c[i]]))
    xs = x.subs(sol)
    print("distance =", sp.simplify(sp.expand(xs.subs(t, T) - x0)))

    # Check: no other end-position gives a lower cost.
    xf = sp.symbols("xf", real=True)
    q = quintic_with(x0, v0, a0, xf, vT, 0)
    J = sp.integrate(sp.diff(q, t, 3) ** 2, (t, 0, T))
    xf_opt = sp.solve(sp.diff(J, xf), xf)[0]
    print("optimal free end (via cost minimisation) =", sp.simplify(xf_opt))
    print("matches quartic end:", sp.simplify(xf_opt - xs.subs(t, T)) == 0)


def quintic_with(px0, pv0, pa0, pxT, pvT, paT):
    x = sum(c[i] * t**i for i in range(6))
    eqs = [
        x.subs(t, 0) - px0, sp.diff(x, t).subs(t, 0) - pv0, sp.diff(x, t, 2).subs(t, 0) - pa0,
        x.subs(t, T) - pxT, sp.diff(x, t).subs(t, T) - pvT, sp.diff(x, t, 2).subs(t, T) - paT,
    ]
    return x.subs(sp.solve(eqs, c, dict=True)[0])


def peaks():
    D = sp.symbols("D", positive=True)
    x = quintic_with(0, 0, 0, D, 0, 0)
    v = sp.diff(x, t)
    a = sp.diff(x, t, 2)
    j = sp.diff(x, t, 3)
    v_pk = sp.simplify(v.subs(t, T / 2))
    t_a = [r for r in sp.solve(sp.diff(a, t), t) if r.is_positive]
    a_pk = sp.simplify(sp.Max(*[sp.Abs(a.subs(t, r)) for r in t_a]))
    j_pk = sp.simplify(sp.Abs(j.subs(t, 0)))
    print("\n== rest-to-rest peaks")
    print("v_peak =", v_pk, "=", sp.N(v_pk / (D / T)), "D/T")
    print("a_peak =", a_pk, "=", sp.N(a_pk / (D / T**2)), "D/T^2")
    print("j_peak =", j_pk)


def via_point_conditions():
    # Two segments meeting at knot (p1, v1, a1); cost gradient vs jumps of jerk and snap.
    T1, T2 = sp.symbols("T1 T2", positive=True)
    p0, p1, p2, v1, a1 = sp.symbols("p0 p1 p2 v1 a1", real=True)
    s1 = quintic_with(p0, 0, 0, p1, v1, a1).subs(T, T1)
    s2 = quintic_with(p1, v1, a1, p2, 0, 0).subs(T, T2)
    J = sp.integrate(sp.diff(s1, t, 3) ** 2, (t, 0, T1)) + sp.integrate(sp.diff(s2, t, 3) ** 2, (t, 0, T2))
    jump_j = sp.diff(s1, t, 3).subs(t, T1) - sp.diff(s2, t, 3).subs(t, 0)
    jump_s = sp.diff(s1, t, 4).subs(t, T1) - sp.diff(s2, t, 4).subs(t, 0)
    print("\n== via-point optimality")
    print("dJ/dv1 / jump(snap) =", sp.simplify(sp.diff(J, v1) / jump_s))
    print("dJ/da1 / jump(jerk) =", sp.simplify(sp.diff(J, a1) / jump_j))


if __name__ == "__main__":
    quintic()
    free_end()
    peaks()
    via_point_conditions()
