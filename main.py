import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.animation as animation
import time
from Simulation import Simulation

SIM_SPEED = 1 # Simulation runs 5x faster than real time
t = 0
acc = 0
vel = 0
real_start_time = 0
theta = 0
loc = 0
# --------- Set up figure and axes ---------
fig, ax = plt.subplots()
timer = ax.text(0.5, 0.9, "", bbox={'facecolor': 'w', 'alpha':0.5, 'pad':5},
                transform=ax.transAxes, ha="center")
acc_display = ax.text(0.3, 0.8, "", bbox={'facecolor': 'w', 'alpha':0.5, 'pad':5},
                transform=ax.transAxes, ha="center")
vel_display = ax.text(0.5, 0.8, "", bbox={'facecolor': 'w', 'alpha':0.5, 'pad':5},
                transform=ax.transAxes, ha="center")
ax.set_xlim(-5,5)
ax.set_ylim(-1, 5)
ax.set_aspect('equal')
plt.grid()

# Cart properties
cart_width = 1.0
cart_height = 0.5

# Pendulum properties
pendulum_length = 2.0

# Draw cart as a Rectangle
cart_patch = plt.Rectangle((0, 0), cart_width, cart_height, color='black')
ax.add_patch(cart_patch)

# Draw pendulum as a Line
pendulum_line, = ax.plot([], [], 'r-', lw=3)

dt = 0.0002  # frame time step
# fig2, ax2 = plt.subplots(4, 1, figsize=(10, 6))



sim = Simulation(dt, 0, theta0 = 0.01)
# --------- Animation functions ---------
def init():
    global real_start_time,ax
    real_start_time = time.time()

    cart_patch.set_xy((-cart_width / 2, 0))
    pendulum_line.set_data([], [])
    timer.set_text('2.5')
    acc_display.set_text(f"Acc: {acc:.4f} m/s^2")
    vel_display.set_text(f"Vel: {vel:.4f} m/s")

    #ax = sim.init_plot()

    return cart_patch, pendulum_line, timer, acc_display, vel_display


def update(frame):
    global t, real_start_time, acc, vel, theta,ax, loc

    real_elapsed = time.time() - real_start_time  # How much real time passed
    target_sim_time = real_elapsed * SIM_SPEED  # We want to simulate faster
    
    while t < target_sim_time:
        real_elapsed = time.time() - real_start_time
        target_sim_time = real_elapsed * SIM_SPEED
        loc, theta, t, acc, vel = sim.get_system_vars(ax, "angle")

    #sim.plot_briefly(ax)
    # Update cart position
    cart_patch.set_xy((loc - cart_width / 2, 0))

    # Calculate pendulum end position
    pivot_x = loc
    pivot_y = cart_height
    pendulum_x = pivot_x - pendulum_length * np.sin(theta)
    pendulum_y = pivot_y + pendulum_length * np.cos(theta)

    # Update pendulum line
    pendulum_line.set_data([pivot_x, pendulum_x], [pivot_y, pendulum_y])

    # Add the timer
    timer.set_text(f"Time: {t:.2f} s")
    acc_display.set_text(f"Acc: {acc:.2f} m/s^2")
    vel_display.set_text(f"Vel: {vel:.2f} m/s")

    return cart_patch, pendulum_line, timer, acc_display, vel_display



# --------- Run the animation ---------



ani = animation.FuncAnimation(fig, update, init_func=init, frames=10000,  interval=dt*1000, blit=True)
plt.show()
