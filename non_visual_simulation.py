import numpy as np
import matplotlib.pyplot as plt


from tkinter import Tk, Scale, HORIZONTAL

import keyboard
import math

def f(t, w, s, v, a, b, c):

    calc_f = v/c - (a/c)*w - (b/c)*s
    return s, calc_f


def f2(t, theta, acceleration, pendconst):
    a = pendconst[0]
    b = pendconst[1]
    c = pendconst[2]
    calc_f2 = a/c*math.sin(theta) + b/c*math.cos(theta)*acceleration
    return calc_f2


def runge_kutta4(t0, h, voltage_slider, consts, w0, s0, theta0, wheel_radius, pendconst, is_plot):

    plt.ion()
    fig, ax = plt.subplots(3, 1, figsize=(10, 6))

    a = consts[0]
    b = consts[1]
    c = consts[2]

    t_vector = [t0]
    w_vector = [w0]
    s_vector = [s0]
    a_vector = [s0*wheel_radius]
    d_vector = [0]
    theta_vector = [theta0]

    ts = t0
    pre_dist = 0
    while True:
        w = w_vector[-1]
        s = s_vector[-1]
        v = voltage_slider.get()

        k1w, k1s = f(ts, w, s, v, a, b, c)
        k2w, k2s = f(ts + h/2, w + h/2 * k1w, s+ h/2 * k1s, v, a, b, c)
        k3w, k3s = f(ts + h/2, w + h/2 * k2w, s + h/2 * k2s, v, a, b, c)
        k4w, k4s = f(ts + h, w + h * k3w, s + h * k3s, v, a, b, c)

        w_new = w + (h/6) * (k1w + 2*k2w + 2*k3w + k4w)
        s_new = s + (h/6) * (k1s + 2*k2s + 2*k3s + k4s)
        
        w_vector.append(w_new)
        s_vector.append(s_new)
        a_new = s_new*wheel_radius
        a_vector.append(a_new)
        dist_new = w_new*wheel_radius*h + pre_dist
        d_vector.append(dist_new)
        pre_dist = dist_new

        theta = theta_vector[-1]
        k1thet = f2(ts, theta, a_new, pendconst)
        k2thet = f2(ts + h/2, theta + h/2 * k1thet, a_new, pendconst)
        k3thet = f2(ts + h/2, theta + h/2 * k2thet, a_new, pendconst)
        k4thet = f2(ts + h, theta + h * k3thet, a_new, pendconst)

        theta_new = theta + (h/6) * (k1thet + 2*k2thet + 2*k3thet + k4thet)
        theta_vector.append(theta_new)

        ts = ts + h 
        t_vector.append(ts)

        if is_plot:
            plot_briefly(ax, fig, t_vector, theta_vector, a_vector, d_vector)

        if keyboard.is_pressed("q"):
            break



def plot_briefly(ax, fig, t, theta, a, distance):

    ax[0].clear()
    ax[0].plot(t, theta, label="theta(t)", color='b')
    ax[0].set_xlabel("t")
    ax[0].set_ylabel("theta")
    ax[0].set_title("theta(t)")
    ax[0].grid(True)
    ax[0].legend()

    ax[1].clear()
    ax[1].plot(t, a, label="a(t)", color='r')
    ax[1].set_xlabel("t")
    ax[1].set_ylabel("a")
    ax[1].set_title("a(t)")
    ax[1].grid(True)
    ax[1].legend() 


    ax[2].clear()
    ax[2].plot(t, distance, label="d(t)", color='r')
    ax[2].set_xlabel("t")
    ax[2].set_ylabel("d")
    ax[2].set_title("d(t)")
    ax[2].grid(True)
    ax[2].legend() 

    plt.draw()
    plt.pause(0.05)



w0 = 1.0 
s0 = 0.0  
t0 = 0.0
h = 0.1 
theta0 = 0

root = Tk()
root.title("Simulator")

k = 2
R = 1
L = 0.1
wheel_mass = 20
wheel_radius = 0.1
I1 = 0.5*wheel_mass*(wheel_radius)**2
gamma = 0.001

a = k + R*gamma/k
b = R*I1/k + L*gamma/k
c = L*I1/k

I2 = 0.03
bp = 0.001
l = 0.7
mr = 0.2
g = 9.81

A = I2 + bp
B = (l/2)*mr*g
C = (l/2)*mr


consts = [a, b, c]
pendconst = [A, B, C]


voltage_slider = Scale(root, from_=-5, to=5, orient=HORIZONTAL, label="voltage", resolution=0.1)
voltage_slider.set(2)
voltage_slider.pack()

runge_kutta4(t0, h, voltage_slider, consts, w0, s0, theta0, wheel_radius, pendconst)