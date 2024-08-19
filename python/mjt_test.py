import sympy as sym
import matplotlib.pyplot as plt
import numpy as np


def main():
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

    print(c2)

    system_eq = [c0 - x0,
                 c5*T*T*T*T*T + c4*T*T*T*T + c3*T*T*T + c2*T*T + c1*T + c0 - xT,
                 c1 - v0,
                 5*c5*T*T*T*T + 4*c4*T*T*T + 3*c3*T*T + 2*c2*T + c1 - vT,
                 2 * c2 - a0,
                 20*c5*T*T*T + 12*c4*T*T + 6*c3*T + 2*c2 - aT]

    system_eq2 = [c0 - T, c1 - 2, c2 - 3, c4 - 4, c5 - 5]

    sol = sym.linsolve(system_eq, (c0, c1, c2, c3, c4, c5))
    print(sol)
    # x0
    # v0
    # a0/2
    # (-3*T*T*a0 + T*T*aT - 12*T*v0 - 8*T*vT - 20*x0 + 20*xT)/(2*T*T*T)
    # (3*T*T*a0 - 2*T*T*aT + 16*T*v0 + 14*T*vT + 30*x0 - 30*xT)/(2*T*T*T*T)
    # (-T*T*a0 + T*T*aT - 6*T*v0 - 6*T*vT - 12*x0 + 12*xT)/(2*T*T*T*T*T)


def equations():
    x0 = 0
    xT = 5
    v0 = 0
    vT = 0
    a0 = 0
    aT = 0

    T = 2

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
    axs[1].plot(t, v)
    axs[1].set_ylabel('v(t)')
    axs[2].plot(t, a)
    axs[2].set_ylabel('a(t)')
    axs[3].plot(t, j)
    axs[3].set_ylabel('j(t)')
    plt.show()


if __name__ == '__main__':
    main()
    equations()
