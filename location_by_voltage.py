import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import keyboard
import numpy as np

LENGTH =  0.7
M_ROD =  0.2
RADIUS = 0.1
M_WHEEL =  2
B =  0.000000001
GAMMA =  0.000000001
K =  2
L =  0.0000001
R =  1


# create constants
a_eff = K + (GAMMA * R)/K
b_eff = M_WHEEL * RADIUS * RADIUS * R/(2*K) + GAMMA * L/K
c_eff = M_WHEEL * RADIUS * RADIUS * L/(2*K)
class LocationSolver:
    """
    this object will solve for the updated location,
    give it the voltage, and it will give you the location
    """
    def __init__(self, a : float, b : float, c : float, radius : float, dt : float, init_x : float = 0, init_vel : float = 0, init_acc : float = 0):

        """
        we have to solve the equation v(t) = a*w+b*w'+c*w''
        :param a: coefficient of w
        :param b: coefficient of w'
        :param c: coefficient of w''
        """
        #initial params
        self.__a = a
        self.__b = b
        self.__c = c
        self.__dt = dt
        self.__rad = radius
        self.ang = init_x/self.__rad
        self.ang_vel = init_vel/self.__rad
        self.ang_acc = init_acc/self.__rad
        self.__time = 0


    def location(self, voltage : float) -> float:
        self.__calculate_next(voltage)
        return self.ang * self.__rad

    def get_time(self):
        return self.__time

    def __calculate_next(self, voltage):
        """
        eulers method for second order
        :param voltage: the voltage
        """
        # solve the ode using euler's method
        temp_ang_imp = self.__calc_ang_imp(self.ang_vel, self.ang_acc, voltage)
        temp_ang_vel = self.ang_vel
        self.__time = self.__time + self.__dt
        self.ang_vel = self.ang_vel + (self.__dt * self.ang_acc)
        self.ang_acc = self.ang_acc + (self.__dt * temp_ang_imp)

        # calculate the new angle
        self.ang = self.ang + (self.__dt * temp_ang_vel)

        #print(fr"The time is: {self.__time}, The angle is {self.ang}, and the velocity is {self.ang_vel}, the acceleration is {self.ang_acc}")
    def __calc_ang_imp(self, ang_vel : float, ang_acc : float, voltage :float) -> float:
        return (-self.__a/self.__c) * ang_vel + (-self.__b/self.__c) * ang_acc + voltage/self.__c



if __name__ == '__main__':
    loc_sol = LocationSolver(a_eff, b_eff, 0.00001, RADIUS, 0.00001)
    temp_voltage = 0
    i = 0
    while True:
        loc = loc_sol.location(temp_voltage)

        if i % 10000 == 0:
            print(temp_voltage, loc, loc_sol.get_time())

        if keyboard.is_pressed('e'):
            temp_voltage += 0.001
        elif keyboard.is_pressed('d'):
            temp_voltage -= 0.001

        i += 1




    # #plt.style.use('seaborn-white')
    # print(a_eff, b_eff, c_eff)
    # time_vals = []
    # loc_vals = []
    # loc_sol = LocationSolver(a_eff, b_eff,  c_eff, RADIUS,  0.0000001)
    # temp_voltage = 0
    #
    # def animate(i):
    #     global temp_voltage
    #     loc_vals.append(loc_sol.location(temp_voltage))
    #     time_vals.append(loc_sol.get_time())
    #     plt.cla()
    #     plt.plot(time_vals, loc_vals)
    #
    #     #control voltage
    #     if keyboard.is_pressed('e'):
    #         temp_voltage += 0.1
    #     elif keyboard.is_pressed('d'):
    #         temp_voltage -= 0.1
    #
    #
    #
    #
    # ani = FuncAnimation(plt.gcf(), func = animate, cache_frame_data=False, interval=0.0001)
    # plt.show()


