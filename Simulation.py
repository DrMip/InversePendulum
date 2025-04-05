import numpy as np
import matplotlib.pyplot as plt
from tkinter import Tk, Scale, HORIZONTAL
import keyboard
import math
from cart_simulator import Visualize

class Simulation():

    def __init__(self, w0, s0, t0, h, theta0):
        
        self.h = h

        self.root = Tk()
        self.root.title("Simulator")
        self.voltage_slider = Scale(self.root, from_=-20, to=20, orient=HORIZONTAL, label="voltage", resolution=0.1)
        self.voltage_slider.set(2)
        self.voltage_slider.pack()

        self.k = 2
        self.R = 1
        self.L = 0.1
        self.wheel_mass = 20
        self.wheel_radius = 0.1
        self.I1 = 0.5*self.wheel_mass*(self.wheel_radius)**2
        gamma = 0.001
        a = self.k + self.R*gamma/self.k
        b = self.R*self.I1/self.k + self.L*gamma/self.k
        c = self.L*self.I1/self.k
        self.I2 = 1
        self.bp = 0.001
        self.l = 0.7
        self. mr = 0.2
        self. g = 9.81
        A = self.I2 + self.bp
        B = (self.l/2)*self.mr*self.g
        C = (self.l/2)*self.mr
        self.consts = [a, b, c]
        self.pendconst = [A, B, C]

        self.time_vector = [t0]
        self.w_vector = [w0]
        self.s_vector = [s0]
        self.a_vector = [s0*self.wheel_radius]
        self.distance_vector = [0]
        self.theta_vector = [theta0]

        self.simulator = Visualize()


    def f(self, t, w, s, v, a, b, c):
        calc_f = v/c - (a/c)*w - (b/c)*s
        return s, calc_f


    def f2(self, t, theta, acceleration, pendconst):
        a = pendconst[0]
        b = pendconst[1]
        c = pendconst[2]
        calc_f2 = a/c*math.sin(theta) + b/c*math.cos(theta)*acceleration
        return calc_f2


    def plot_briefly(self, ax):

        ax[0].clear()
        ax[0].plot(self.time_vector, self.theta_vector, label="theta(t)", color='b')
        ax[0].set_xlabel("t")
        ax[0].set_ylabel("theta")
        ax[0].set_title("theta(t)")
        ax[0].grid(True)
        ax[0].legend()

        ax[1].clear()
        ax[1].plot(self.time_vector, self.a_vector, label="a(t)", color='r')
        ax[1].set_xlabel("t")
        ax[1].set_ylabel("a")
        ax[1].set_title("a(t)")
        ax[1].grid(True)
        ax[1].legend() 


        ax[2].clear()
        ax[2].plot(self.time_vector, self.distance_vector, label="d(t)", color='r')
        ax[2].set_xlabel("t")
        ax[2].set_ylabel("d")
        ax[2].set_title("d(t)")
        ax[2].grid(True)
        ax[2].legend() 

        plt.draw()
        plt.pause(0.05)


    def run(self, is_plot, is_simulate, simulation_length):
        plt.ion()
        fig, ax = plt.subplots(3, 1, figsize=(10, 6))
        while self.time_vector[-1] < simulation_length:
             self.runge_kutta4(self.time_vector[-1])
             if is_plot:
                  self.plot_briefly(ax)
             if is_simulate:
                  self.simulator.simulate(self.distance_vector[-1], -self.theta_vector[-1], self.time_vector[-1])
            
             self.root.update_idletasks()  # Process idle tasks (e.g., slider update)
             self.root.update()
             
    

    def runge_kutta4(self, ts):
            w = self.w_vector[-1]
            s = self.s_vector[-1]
            v = self.voltage_slider.get()

            k1w, k1s = self.f(ts, w, s, v, self.consts[0], self.consts[1], self.consts[2])
            k2w, k2s = self.f(ts + self.h/2, w + self.h/2 * k1w, s+ self.h/2 * k1s, v,self.consts[0], self.consts[1], self.consts[2])
            k3w, k3s = self.f(ts + self.h/2, w + self.h/2 * k2w, s + self.h/2 * k2s, v,self.consts[0], self.consts[1], self.consts[2])
            k4w, k4s = self.f(ts + self.h, w + self.h * k3w, s + self.h * k3s, v, self.consts[0], self.consts[1], self.consts[2])

            w_new = w + (self.h/6) * (k1w + 2*k2w + 2*k3w + k4w)
            s_new = s + (self.h/6) * (k1s + 2*k2s + 2*k3s + k4s)
            
            self.w_vector.append(w_new)
            self.s_vector.append(s_new)
            a_new = s_new*self.wheel_radius
            self.a_vector.append(a_new)
            dist_new = w_new*self.wheel_radius*self.h + self.distance_vector[-1]
            self.distance_vector.append(dist_new)

            theta = self.theta_vector[-1]
            k1thet = self.f2(ts, theta, a_new, self.pendconst)
            k2thet = self.f2(ts + self.h/2, theta + self.h/2 * k1thet, a_new, self.pendconst)
            k3thet = self.f2(ts + self.h/2, theta + self.h/2 * k2thet, a_new, self.pendconst)
            k4thet = self.f2(ts + self.h, theta + self.h * k3thet, a_new, self.pendconst)

            theta_new = theta + (self.h/6) * (k1thet + 2*k2thet + 2*k3thet + k4thet)
            self.theta_vector.append(theta_new)

            ts = ts + self.h 
            self.time_vector.append(ts)



s = Simulation(0, 0, 0, 0.02, 0)
s.run(False, True, 100)




    
        

