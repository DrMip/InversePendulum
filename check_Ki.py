from scipy import signal
import matplotlib.pyplot as plt
import numpy as np

# Paste your MATLAB-exported numerator and denominator here
num = [0, 0, 1 ]  # from tfdata
den = [1.5e-07, 0.0015, 0.2001]

sys = signal.TransferFunction(num, den)
dc_gain = num[-1] / den[-1]
print("DC Gain:", dc_gain)
t = np.linspace(0, 0.2, 100)
t, y = signal.step(sys, T=t)




plt.plot(t, y)
plt.grid()
plt.title("Python Closed-Loop Step Response")
plt.xlabel("Time (s)")
plt.ylabel("Output")
plt.show()