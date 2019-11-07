# Author: Waltteri Koskinen
# When: 07.11.2019
# Why: to practice mathematical modeling
# What it does: Calculates acceleration of a passenger car with step
# throttle input.
# Vehicle dynamics, Module 1:2, Chalmers University of Technology

import numpy as np
import matplotlib.pyplot as plt


def model(m, g, h, l_f, l_r, ccx, i, J_flywheel, r, c_p, time, T_wheel,
          omega_engine, v_x):

    if time >= 1:
        T_engine = 50
    else:
        T_engine = 0

    T_transmission = T_wheel/i
    D_omega_engine = (T_engine-T_transmission)/J_flywheel

    F_front_x = T_wheel/r
    a_x = T_wheel/(r*m)
    F_front_z = (g*l_r*m-F_front_x*h)/(l_f+l_r)
    omega_wheel = (ccx*F_front_z*v_x)/(ccx*F_front_z*r-F_front_x*r)
    omega_transmission = omega_engine / i
    D_Torque_wheel = c_p*(omega_transmission-omega_wheel)

    results = [D_omega_engine, a_x, D_Torque_wheel]
    return results


def main():
    g = 9.80665     # Gravity
    J_e = 0.5       # Flywheel inertia
    ratio = 7       # Gearbox ratio
    c_p = 2000      # Drive shaft spring stiffness
    R = 0.3         # Tyre radius
    cc_x = 10       # Tyre slip
    l_f = 1.25      # Distance between CoG and front axle
    l_r = 1.5       # Distance between CoG and rear axle
    h = 0.5         # CoG height
    m = 1500        # Vehicle mass

    # Stepping options
    time = np.linspace(0., 5, 5001)
    dt = 0.001

    # Initial conditions:
    speed = [5.56]
    Torque = [0]
    omega_engine = [130]

    for i in range(1, len(time)):
        results = model(m, g, h, l_f, l_r, cc_x, ratio, J_e, R, c_p, time[i],
                        Torque[i-1], omega_engine[i-1], speed[-1])
        omega_engine.append(results[0] * dt + omega_engine[-1])
        speed.append(results[1]*dt + speed[-1])
        Torque.append(results[2]*dt + Torque[-1])

    plt.plot(time, speed)
    plt.plot(time, omega_engine)
    plt.plot(time, Torque)
    plt.grid()
    plt.legend(['Speed', 'Rev', 'Torque'])
    plt.show()


main()
