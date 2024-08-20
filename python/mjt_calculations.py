import sympy as sym
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit


def mjt_symbolic_coefficients():
    """Solve for MJT coefficients using symbolic math."""
    x0 = sym.symbols('x0')
    xT = sym.symbols('xT')
    v0 = sym.symbols('v0')
    vT = sym.symbols('vT')
    a0 = sym.symbols('a0')
    aT = sym.symbols('aT')
    T = sym.symbols('T')
    c0 = sym.symbols('c0')
    c1 = sym.symbols('c1')
    c2 = sym.symbols('c2')
    c3 = sym.symbols('c3')
    c4 = sym.symbols('c4')
    c5 = sym.symbols('c5')

    system_eq = [c0 - x0,
                 c5*T*T*T*T*T + c4*T*T*T*T + c3*T*T*T + c2*T*T + c1*T + c0 - xT,
                 c1 - v0,
                 5*c5*T*T*T*T + 4*c4*T*T*T + 3*c3*T*T + 2*c2*T + c1 - vT,
                 2 * c2 - a0,
                 20*c5*T*T*T + 12*c4*T*T + 6*c3*T + 2*c2 - aT]

    sol = sym.linsolve(system_eq, (c0, c1, c2, c3, c4, c5))

    # coefficients: [c0, c1, c2, c3, c4, c5]
    print(sol)
    # x0
    # v0
    # a0/2
    # (-3*T*T*a0 + T*T*aT - 12*T*v0 - 8*T*vT - 20*x0 + 20*xT)/(2*T*T*T)
    # (3*T*T*a0 - 2*T*T*aT + 16*T*v0 + 14*T*vT + 30*x0 - 30*xT)/(2*T*T*T*T)
    # (-T*T*a0 + T*T*aT - 6*T*v0 - 6*T*vT - 12*x0 + 12*xT)/(2*T*T*T*T*T)


def plot_unit_mjt_profiles():
    """Plot minimum jerk trajectory unit profiles."""
    x0 = 0
    xT = 10
    v0 = 0
    vT = 0
    a0 = 0
    aT = 0

    T = 1

    c0 = x0
    c1 = v0
    c2 = a0/2
    c3 = (-3*T*T*a0 + T*T*aT - 12*T*v0 - 8*T*vT - 20*x0 + 20*xT)/(2*T*T*T)
    c4 = (3*T*T*a0 - 2*T*T*aT + 16*T*v0 + 14*T*vT + 30*x0 - 30*xT)/(2*T*T*T*T)
    c5 = (-T*T*a0 + T*T*aT - 6*T*v0 - 6*T*vT - 12*x0 + 12*xT)/(2*T*T*T*T*T)

    t = np.linspace(0, T, 100)
    x = c0 + c1*t + c2*t*t + c3*t*t*t + c4*t*t*t*t + c5*t*t*t*t*t
    v = c1 + 2*c2*t + 3*c3*t*t + 4*c4*t*t*t + 5*c5*t*t*t*t
    a = 2*c2 + 6*c3*t + 12*c4*t*t + 20*c5*t*t*t
    j = 6*c3 + 24*c4*t + 60*c5*t*t

    # plot x(t), v(t), a(t), j(t) in subplots
    fig, axs = plt.subplots(4)
    axs[0].plot(t, x)
    axs[0].set_ylabel('x(t)')
    axs[0].set_title('Unit MJT Profiles')
    axs[1].plot(t, v)
    axs[1].set_ylabel('v(t)')
    axs[2].plot(t, a)
    axs[2].set_ylabel('a(t)')
    axs[3].plot(t, j)
    axs[3].set_ylabel('j(t)')
    axs[3].set_xlabel('t')
    plt.tight_layout()


def max_velocity_to_trajectory_duration_equations():
    """Maps maximum velocity to trajectory duration."""

    x0 = 0
    xT = 1
    v0 = 0
    vT = 0
    a0 = 0
    aT = 0

    xT = np.linspace(0.1, 10, 1000)
    T = np.linspace(0.1, 10, 1000)

    # mapping functions
    def f1(x, a):
        """Mapping function for T to Vmax."""
        return a * x**-1

    def f2(x, b, c):
        """Mapping function for xT to a."""
        return b * x + c

    a = []
    for i in range(len(xT)):
        vmax = []
        for j in range(len(T)):
            c0 = x0
            c1 = v0
            c2 = a0 / 2
            c3 = (-3 * T[j] * T[j] * a0 + T[j] * T[j] * aT - 12 * T[j] * v0 - 8 * T[j] * vT - 20 * x0 + 20 * xT[i]) / (2 * T[j] * T[j] * T[j])
            c4 = (3 * T[j] * T[j] * a0 - 2 * T[j] * T[j] * aT + 16 * T[j] * v0 + 14 * T[j] * vT + 30 * x0 - 30 * xT[i]) / (2 * T[j] * T[j] * T[j] * T[j])
            c5 = (-T[j] * T[j] * a0 + T[j] * T[j] * aT - 6 * T[j] * v0 - 6 * T[j] * vT - 12 * x0 + 12 * xT[i]) / (2 * T[j] * T[j] * T[j] * T[j] * T[j])

            # Vmax at t = 0.5*T[j]
            vmax.append(c1 + 2*c2*0.5*T[j] + 3*c3*0.5*T[j]*0.5*T[j] + 4*c4*0.5*T[j]*0.5*T[j]*0.5*T[j] + 5*c5*0.5*T[j]*0.5*T[j]*0.5*T[j]*0.5*T[j])

        f1_params, _ = curve_fit(f1, T, vmax)
        a.append(f1_params[0])

    f2_params, _ = curve_fit(f2, xT, a)

    # print the mapping functions
    print('Mapping function for T to Vmax: Vmax = a * T^-1')
    print(f'Mapping function for xT to a: a = {np.round(f2_params[0], 4)} * xT + {np.round(f2_params[1], 4)}')

    # plot Vmax vs T and a vs xT in subplots
    plt.figure()
    plt.subplot(211)
    plt.plot(T, vmax)
    plt.plot(T, f1(T, f1_params[0]))
    plt.title('Vmax to T mapping')
    plt.ylabel('Vmax')
    plt.xlabel('T')
    plt.subplot(212)
    plt.plot(xT, a)
    plt.plot(xT, f2(xT, f2_params[0], f2_params[1]))
    plt.ylabel('a')
    plt.xlabel('xT')
    plt.tight_layout()


if __name__ == '__main__':
    mjt_symbolic_coefficients()
    plot_unit_mjt_profiles()
    max_velocity_to_trajectory_duration_equations()
    plt.show()
