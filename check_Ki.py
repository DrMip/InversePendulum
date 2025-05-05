from scipy import signal
import matplotlib.pyplot as plt
import numpy as np

# Define transfer function K(s)
num_K = [0, 0, 0, 0.07, 0]
den_K = [4.9e-9, 4.9e-5, 0.006536, -0.00083, -0.1374]

# Define controller transfer function ctrl(s) = (s + 2)/s
num_ctrl = [8.5, 17]
den_ctrl = [1, 0]

# Multiply numerator and denominator polynomials to get open-loop transfer function
num_ol = np.polymul(num_ctrl, num_K)
den_ol = np.polymul(den_ctrl, den_K)

# Closed-loop transfer function: OL / (1 + OL)
# So denominator becomes: den_ol + num_ol
num_cl = num_ol
den_cl = np.polyadd(den_ol, num_ol)

# Create closed-loop system
closed_loop = signal.TransferFunction(num_cl, den_cl)

# Simulate step response
t = np.linspace(0, 5, 100)
t, y = signal.step(closed_loop, T=t)

# Plot
plt.plot(t, y)
plt.title('Closed-Loop Step Response')
plt.xlabel('Time [s]')
plt.ylabel('Output')
plt.grid(True)
plt.show()
