from pkgutil import get_loader

import numpy as np
import matplotlib.pyplot as plt
from tkinter import Tk, Scale, HORIZONTAL
import keyboard
import math
from cart_simulator import Visualize

e_i = 0
class Simulation():

    def __init__(self,h,setpoint, vel0 = 0, a0 = 0, t0 = 0, theta0 = 0, theta_dot0= 0):
        
        self.h = h

        #self.root = Tk()
        #self.root.title("Simulator")
        #self.voltage_slider = Scale(self.root, from_=-20, to=20, orient=HORIZONTAL, label="voltage", resolution=0.1)
        #self.voltage_slider.set(2)
        #self.voltage_slider.pack()

        self.k = 2
        self.R = 1
        self.L = 0.0001
        self.wheel_mass = 2
        self.wheel_radius = 0.1
        self.I1 = 1.5*self.wheel_mass*(self.wheel_radius)**2
        gamma = 0.001
        a = (self.k + self.R*gamma/self.k)/self.wheel_radius
        b = (self.R*self.I1/self.k + self.L*gamma/self.k)/self.wheel_radius
        c = self.L*self.I1/self.k/self.wheel_radius
        self.bp = 0.001
        self.l = 0.7
        self.mr = 0.2
        self.g = 9.81

        self.e_i_vel = 0
        self.e_i_angle = 0

        self.time2reach = -1
        self.time_in_tolerance = 0
        self.tolerance = 0.02


        #self.setpoint = setpoint

        self.I2 = (1/3.0)*self.mr*(self.l)**2
        
        A = self.I2
        B = (self.l/2)*self.mr*self.g
        C = (self.l/2)*self.mr
        D = -1*self.bp

        self.consts = [a, b, c]
        self.pendconst = [A, B, C, D]
        print(theta0)
        self.time_vector = [t0]
        self.velocity_vector = [vel0]
        self.accel_vector = [a0]
        self.distance_vector = [0.0]
        self.theta_vector = [theta0]
        self.thet_dot = [theta_dot0]
        self.theta_integral = [0.0]
        self.e = [self.setpoint_func()]
        self.e_precentage = [100]
        self.voltage_vec = [0.0]


    def PID_controller_volt(self, setpoint, flag = "velocity"):
        current_time = self.time_vector[-1]
        if flag == "velocity":
            e_p = setpoint - self.velocity_vector[-1]
            self.e_i_vel += e_p*self.h
            e_d = -1*self.accel_vector[-1]
        elif flag == "angle":
            setpoint = 0.714*setpoint
            e_p = -self.theta_vector[-1] + setpoint
            self.e_i_angle += e_p*self.h
            e_d = -1*self.thet_dot[-1]

        elif flag=="acceleration":
            setpoint = 0.714*setpoint/self.g
            e_p = -self.theta_vector[-1] + setpoint
            self.e_i_angle += e_p*self.h
            e_d = -1*self.thet_dot[-1]

        elif flag=="vel&ang":
            e_p_vel = setpoint - self.velocity_vector[-1]
            print(e_p_vel)
            self.e_i_vel += e_p_vel*self.h
            e_d_vel = -1*self.accel_vector[-1]

            k_p_vel = 1
            k_d_vel = 0
            k_i_vel = 0

            wanted_acceleration = (k_p_vel * e_p_vel + k_d_vel * e_d_vel + k_i_vel * self.e_i_vel)
            wanted_angle = 0.714*wanted_acceleration/self.g
            e_p = -self.theta_vector[-1] + wanted_angle
            self.e_i_angle += e_p*self.h
            e_d = -1*self.thet_dot[-1]



        else:
            e_p = 0
            e_i = 0
            e_d = 0

        if flag == "velocity":
            k_p = 0
            k_d = 0
            k_i = 7

            val = (k_p*e_p + k_d*e_d + k_i*self.e_i_vel)
        if flag == "angle" or flag=="acceleration" or flag=="vel&ang":
            k_p = 300
            k_d = 0
            k_i = 600

            val = (k_p * e_p + k_d * e_d + k_i * self.e_i_angle)

        self.e.append(e_p)
        if setpoint - self.velocity_vector[0] > 0:
            self.e_precentage.append(e_p/(setpoint - self.velocity_vector[0])*100)


        if val>240:
            val = 240
        elif val < -240:
            val = -240

        return val

    def f_vel(self, t, w, s, v, a, b, c):
        calc_f = v/c - (a/c)*w - (b/c)*s
        return s, calc_f


    def f_theta(self, t, theta, theta_dot, w, s, v):
        a2, b2, c2, d2 = self.pendconst
        theta_ddot = (b2/a2)*math.sin(theta) + (c2/a2)*math.cos(theta)*s + (d2/a2)*theta_dot
        return theta_dot, theta_ddot


    def plot_briefly(self, ax):

        ax[0].clear()
        ax[0].plot(self.time_vector, self.theta_vector, label="theta(t)", color='b')
        ax[0].set_xlabel("t")
        ax[0].set_ylabel("theta")
        ax[0].set_title("theta(t)")
        ax[0].grid(True)
        ax[0].legend()

        ax[1].clear()
        ax[1].plot(self.time_vector, self.velocity_vector, label="v(t)", color='r')
        ax[1].plot(self.time_vector, [self.setpoint_func()]*len(self.time_vector), label="setpoint", color="m")
        ax[1].plot(self.time_vector, self.e, label="error", color="b")

        ax[1].set_xlabel("t")
        ax[1].set_ylabel("v")
        ax[1].set_title("v(t)")
        ax[1].grid(True)
        ax[1].legend()

        plt.draw()
        plt.pause(0.05)


    # def run(self, is_plot, is_simulate, simulation_length):
    #     plt.ion()
    #     fig, ax = plt.subplots(4, 1, figsize=(10, 6))
    #     while self.time_vector[-1] < simulation_length:
    #          self.runge_kutta4(self.time_vector[-1], "velocity")
    #          if is_plot:
    #               self.plot_briefly(ax)
    #          if is_simulate:
    #               self.simulator.simulate(self.distance_vector[-1], -self.theta_vector[-1], self.time_vector[-1])

    #          #self.root.update_idletasks()  # Process idle tasks (e.g., slider update)
    #          #self.root.update()

    def run(self, duration):
        while self.time_vector[-1]<duration:
            self.runge_kutta4(self.time_vector[-1], "angle", self.setpoint_func("BalanceAndVel"))

        return self.time2reach, self.time_in_tolerance


    def get_system_vars(self, ax, flag = "velocity") -> (float,float,float, float, float):
        self.runge_kutta4(self.time_vector[-1], flag, self.setpoint_func("BalanceAndVel"))
        #self.plot_briefly(ax)
        return self.distance_vector[-1], self.theta_vector[-1], self.time_vector[-1], self.accel_vector[-1], self.velocity_vector[-1]

    def get_time_velocity(self):
        return self.time_vector, self.velocity_vector


    def limit_theta(self, theta):
        return math.fmod(theta + math.pi, 2 * math.pi) - math.pi


    def f_all(self, t, y, v):
        w, s, theta, theta_dot = y
        a, b, c = self.consts
        a2, b2, c2, d2 = self.pendconst
        dw_dt = s
        ds_dt = (v / c) - (a / c) * w - (b / c) * s
        dtheta_dt = theta_dot
        dtheta_dot_dt = (b2 / a2) * math.sin(theta) + (c2 / a2) * math.cos(theta) * s + (d2 / a2) * theta_dot

        return [dw_dt, ds_dt, dtheta_dt, dtheta_dot_dt]


    def runge_kutta4(self, ts, flag, setpoint):
                w = self.velocity_vector[-1]
                s = self.accel_vector[-1]
                v = self.PID_controller_volt(setpoint, flag)
                self.voltage_vec.append(v)
                #v = self.voltage_slider.get()


                k1w, k1s = self.f_vel(ts, w, s, v, self.consts[0], self.consts[1], self.consts[2])
                k2w, k2s = self.f_vel(ts + self.h/2, w + self.h/2 * k1w, s+ self.h/2 * k1s, v,self.consts[0], self.consts[1], self.consts[2])
                k3w, k3s = self.f_vel(ts + self.h/2, w + self.h/2 * k2w, s + self.h/2 * k2s, v,self.consts[0], self.consts[1], self.consts[2])
                k4w, k4s = self.f_vel(ts + self.h, w + self.h * k3w, s + self.h * k3s, v, self.consts[0], self.consts[1], self.consts[2])

                w_new = w + (self.h/6) * (k1w + 2*k2w + 2*k3w + k4w)
                s_new = s + (self.h/6) * (k1s + 2*k2s + 2*k3s + k4s)

                self.velocity_vector.append(w_new)
                self.accel_vector.append(s_new)
                dist_new = w_new*self.h + self.distance_vector[-1]
                self.distance_vector.append(dist_new)

                theta = self.theta_vector[-1]
                theta_dot = self.thet_dot[-1]
                k1thet, k1dot = self.f_theta(ts, theta, theta_dot, w, s_new,v)
                k2thet, k2dot = self.f_theta(ts + self.h/2, theta + self.h/2 * k1thet, theta_dot + self.h/2 * k1dot, w, s_new, v)
                k3thet, k3dot = self.f_theta(ts + self.h/2, theta + self.h/2 * k2thet, theta_dot + self.h/2 * k2dot,w,s_new,v)
                k4thet, k4dot = self.f_theta(ts + self.h, theta + self.h * k3thet, theta_dot + self.h*k3dot, w, s_new, v)

                theta_new = theta + (self.h/6) * (k1thet + 2*k2thet + 2*k3thet + k4thet)
                theta_new = self.limit_theta(theta_new)


                dot_new = theta_dot + (self.h/6) * (k1dot + 2*k2dot + 2*k3dot + k4dot)


                self.theta_vector.append(theta_new)
                self.thet_dot.append(dot_new)
                theta_int_new = theta_new*self.h + self.theta_integral[-1]
                self.theta_integral.append(theta_int_new)

                ts = ts + self.h
                self.time_vector.append(ts)

                dif = abs(theta_new-setpoint)
                if dif<=self.tolerance:
                    if self.time2reach==-1:
                        self.time2reach = ts
                    else:
                        self.time_in_tolerance = ts - self.time2reach

    def setpoint_func(self, setpoint_flag = "const"):
        
        if setpoint_flag == "sin":
            return 0.1*math.sin(2*self.time_vector[-1])
        if setpoint_flag == "const":
            return 0.01
        if setpoint_flag == "BalanceAndVel":
            current_vel = self.velocity_vector[-1]
            wanted_vel = 2
            vel_error = wanted_vel - current_vel

            kp = -0.5

            acc = kp * vel_error
            #return the degree
            return math.atan2(acc, self.g)



        
            
                


    def plot(self, time):
        while self.time_vector[-1]<time:
            self.runge_kutta4(self.time_vector[-1], "angle", self.setpoint_func("sin"))

        
        plt.plot(self.time_vector, self.theta_vector)
        #plt.plot(self.time_vector, self.velocity_vector)

        plt.show()


if __name__ == "__main__":
    
    # theta_start = 0.01
    # thetas_vec = []
    # t2rvec = []
    # timetvec = []
    # step = 0.01
    # dt = 0.0002
    # duration = 130
    #
    # final_theta = 0.5
    # current_theta = theta_start
    # while current_theta<final_theta:
    #     s = Simulation(dt, 0.01, theta0=current_theta)
    #     t2r, timeit = s.run(duration)
    #     print(t2r, timeit)
    #     thetas_vec.append(current_theta)
    #     t2rvec.append(t2r)
    #     timetvec.append(timeit)
    #     current_theta +=step
    #
    #
    # plt.plot(thetas_vec, t2rvec)
    # plt.plot(thetas_vec, timetvec)
    # plt.show()

    s = Simulation(0.0002, 0.01, theta0=0)
    s.plot(1)
    



    
        

