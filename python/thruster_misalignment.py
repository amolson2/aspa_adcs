
import numpy as np
import matplotlib.pyplot as plt

from numpy.linalg import norm


def format_var(name: str, var, unit: str = ''):
    print(f'{name:>50}  :  {var:>10} {unit}')


def inch_to_meter(inch):
    return 0.0254 * inch


def lb_to_kg(lb):
    return 0.453592 * lb


def torque(t, c_o, c_f, F, offset):
    # Initial COM-to-thruster moment arm (Also thrust vector)
    t_o = t - c_o
    l_o = norm(t_o)
    # Final COM-to-thruster moment arm
    t_f = t - c_f
    l_f = norm(t_f)
    # Final angle between thrust vector and final moment arm
    # 0.00872665 is a 0.5-degree offset
    theta = np.arccos(np.dot(t_o, t_f) / (l_o * l_f)) + offset * (np.pi / 180)
    # Final instantaneous torque on spacecraft COM
    T = F * l_f * np.sin(theta)
    print(T)
    T2 = np.cross(t_f, -F * (t_o / norm(t_o)))
    # print(T2, norm(T2))
    return theta, t_o, t_f, norm(T2)


# Thruster Location on Spacecraft (m)
thruster = np.array([0, 0.771493377, 0.6858609854])
# thruster = np.array([1, 0, 0])
# Initial (Wet) Center of mass    (m)
c_o = inch_to_meter(np.array([5.3279459e+01, 3.0405055e+01, 2.5733602e+01]))
# c_f = inch_to_meter(np.array([5.3977153e+01, 2.9710242e+01,
# 2.3760950e+01]))    # Final (Dry) Center of mass      (m)
c_f = c_o + (c_o - thruster) * 0.1
# Nominal Thruster Force          (N)
F = 425
# Burn Time                       (s)
time_burn = 3492
offset = 0    # Thruster offset (deg)

theta, t_o, t_f, torque_f = torque(thruster, c_o, c_f, F, offset)
total_mom = (1 / 2) * time_burn * torque_f

format_var('Theta', theta * (180 / np.pi), 'deg')
format_var('thrust', str(thruster), 'm')
format_var('t_o', str(t_o), 'm')
format_var('t_f', str(t_f), 'm')
format_var('C_o', str(c_o), 'm')
format_var('C_f', str(c_f), 'm')
format_var('Initial Moment Arm', str(t_o), 'm')
format_var('Final Moment Arm', str(t_f), 'm')
format_var('Final Torque on Spacecraft After Burn', torque_f, 'Nm')
format_var('Total Momentum Accumulation from Burn', total_mom, 'Nms')
