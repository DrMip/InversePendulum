import control as ctrl
from transfer_consts import TransferConsts as ts
import numpy as np
import matplotlib.pyplot as plt

class RootLocus():
    
    def __init__(self, P, I, D):
        self.P = P
        self.I = I
        self.D = D
        
    def calc_GH(self):
        numerator = [0, 0, 0, 1, 0]  # corresponds to 0*s^4 + 0*s^3 + 0*s^2 + 1*s + 0
        denominator = [ts.a * self.D,
                       ts.a * self.P + ts.b * self.D,
                       ts.a * self.I + ts.b * self.P + ts.c * self.D,
                       ts.b * self.I + ts.c * self.P + 1,
                       ts.c *self.I]
        return numerator, denominator


    def calc_KH(self):
        numerator = [0, 0, ts.K, 0, 0, 0, 0]  # → K * s^2
        d_coeffs = np.polymul(np.polymul([ts.a, ts.b, ts.c], [ts.A, ts.B, ts.C]), [self.D, self.P, self.I])
        d_coeffs[4] +=ts.K
        denominator = d_coeffs
        return numerator, denominator
    

    def poles_zeros(self, n, d):
        T = ctrl.TransferFunction(n, d)
        zeros = ctrl.zeros(T)
        roots = ctrl.poles(T)
        return zeros, roots



    def plot_root_locus(self, n, d):
        z, p = self.poles_zeros(n, d)
        fig,ax = plt.subplots()

        ax.scatter(z.real, z.imag, s=80, facecolors='none', edgecolors='hotpink')
        ax.scatter(p.real, p.imag, color = '#88c999', marker="x")

        ax.set_xlabel("real")
        ax.set_ylabel("imaginary")
        ax.set_title("Poles and Zeros Map")

        ax.legend(["zeros", "poles"])

        plt.show()


r = RootLocus(1, 0, 0.3)
n, d = r.calc_KH()
r.plot_root_locus(n, d)