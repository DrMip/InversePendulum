import control as ctrl
from transfer_consts import TransferConsts as ts
import numpy as np
import matplotlib.pyplot as plt

class RootLocus():
    
    def __init__(self, P, I, D):
        self.P = P
        self.I = I
        self.D = D

    def PID_Transfer(self):
        numerator = [0, 1, 0]  # corresponds 0*s^2 + 1*s + 0
        denominator = [self.D, self.P, self.I]
        return numerator, denominator

    def H_Transfer(self):
        numerator = [0, 0, 1]  # corresponds 0*s^2 + 1*s + 0
        denominator = [ts.a, ts.b, ts.c]
        return numerator, denominator

    def system_transfer(self) -> ctrl.TransferFunction:
        numG, denG = self.PID_Transfer()
        G = ctrl.TransferFunction(numG, denG)
        numH, denH = self.H_Transfer()
        H = ctrl.TransferFunction(numH, denH)

        # System transfer function: E(s) = (G(s)*H(s) / (1 + G(s)*H(s)))
        T_s = (G*H / (1 + G*H))

        return T_s


    def calc_GH(self):
        numerator = [0, 0, 0, 1, 0]  # corresponds to 0*s^4 + 0*s^3 + 0*s^2 + 1*s + 0
        denominator = [ts.a * self.D,
                       ts.a * self.P + ts.b * self.D,
                       ts.a * self.I + ts.b * self.P + ts.c * self.D,
                       ts.b * self.I + ts.c * self.P + 1,
                       ts.c * self.I]
        return numerator, denominator


    def calc_KH(self):
        numerator = [0, 0, ts.K, 0, 0, 0, 0]  # → K * s^2
        d_coeffs = np.polymul(np.polymul([ts.a, ts.b, ts.c], [ts.A, ts.B, ts.C]), [self.D, self.P, self.I])
        d_coeffs[4] +=ts.K
        denominator = d_coeffs
        return numerator, denominator
    

    def poles_zeros(self, T_s):
        zeros = ctrl.zeros(T_s)
        roots = ctrl.poles(T_s)

        return zeros, roots

    def calc_OS(self):
        z,p = self.poles_zeros(self.system_transfer())
        real_poles = self.find_complex_differences(z,p)
        omega = np.sqrt(real_poles[0]*real_poles[1])
        b = real_poles[0] + real_poles[1]
        xi = b/(2*omega)
        OS = np.exp(-np.pi * xi / np.sqrt(1-xi**2))
        print(np.real(OS))


    def plot_root_locus(self, T_s):
        z, p = self.poles_zeros(T_s)
        fig,ax = plt.subplots()


        ax.scatter(z.real, z.imag, s=80, facecolors='none', edgecolors='hotpink')
        ax.scatter(p.real, p.imag, color = '#88c999', marker="x")

        ax.set_xlabel("real")
        ax.set_ylabel("imaginary")
        ax.set_title("Poles and Zeros Map")

        ax.legend(["zeros", "poles"])

        plt.grid()

        plt.show()


    def calculate_overshoot(self):
        # Get numerator and denominator
        num, den = self.calc_GH()
        system = ctrl.TransferFunction(num, den)

        # Step response
        t, y = ctrl.step_response(system)

        # Handle unstable / divergent response
        if np.any(np.isnan(y)) or np.isinf(y[-1]):
            return None, None, None

        final_value = y[-1]
        peak_value = np.max(y)

        if final_value == 0:
            overshoot_percent = 0
        else:
            overshoot_percent = ((peak_value - final_value) / final_value) * 100

        return overshoot_percent, peak_value, final_value

    def calculate_steady_state_error(self, input_type="step"):
        """
        Calculates the steady-state error using the Final Value Theorem.
        input_type: "step", "ramp", or "parabola"
        Returns: steady-state error (float) or None if can't compute
        """

        try:
            # Get numerator and denominator for open-loop transfer function GH(s)
            numG, denG = self.PID_Transfer()
            G = ctrl.TransferFunction(numG, denG)
            numH, denH = self.H_Transfer()
            H = ctrl.TransferFunction(numH, denH)

            # Error transfer function: E(s) = (1 / (1 + G(s))) * R(s)
            E_s = (1 / (1 + G*H))

            # Final value theorem: lim_{s→0} E(s)
            s_val = 1e-6  # near-zero for numerical stability
            E_val = ctrl.evalfr(E_s, s_val)
            return np.real(E_val)

        except Exception as e:
            print(f"⚠️ Error calculating SSE: {e}")
            return None

    def plot_sse_vs_P(self, P_range, input_type="step"):
        """
        Plots steady-state error (ess) as a function of P values.

        Parameters:
        - P_range: iterable (e.g., np.linspace)
        - input_type: "step", "ramp", or "parabola"
        """
        P_values = []
        SSE_values = []

        original_P = self.P  # Save original value to restore later

        for P_val in P_range:
            self.P = P_val
            ess = self.calculate_steady_state_error(input_type=input_type)

            if ess is not None:
                P_values.append(P_val)
                SSE_values.append(ess)

        self.P = original_P  # Restore original P

        # Plot
        plt.figure()
        plt.plot(P_values, SSE_values, marker='o', linestyle='-')
        plt.xlabel("Proportional Gain P")
        plt.ylabel("Steady-State Error")
        plt.title(f"Steady-State Error vs P (input: {input_type})")
        plt.grid(True)
        plt.show()

    @staticmethod
    def find_complex_differences(list1, list2, tol=1e-2):
        """
        Returns the complex numbers that are different between list1 and list2.
        Two numbers are considered equal if their absolute difference is below `tol`.

        Parameters:
            list1, list2: lists of complex numbers
            tol: tolerance for comparison (default: 1e-8)

        Returns:
            List of values that are not common between the two (from both lists)
        """
        diffs = []
        for z1 in list1:
            if not any(abs(z1 - z2) < tol for z2 in list2):
                diffs.append(z1)
        for z2 in list2:
            if not any(abs(z2 - z1) < tol for z1 in list1):
                diffs.append(z2)
        return diffs




r = RootLocus(10, 0,0)
T_s = r.system_transfer()
r.plot_root_locus(T_s)
r.calc_OS()
# overshoot, peak, final = r.calculate_overshoot()
# P_values = np.linspace(0.1, 100, 50)
# r.plot_sse_vs_P(P_values)

# if overshoot is not None:
#     print(f"ℹ️ For P={r.P}: Overshoot = {overshoot:.2f}%, Peak = {peak:.2f}, Final Value = {final:.2f}")
# else:
#     print("⚠️ System might be unstable or diverging.")
#
# print(f"Steady-State Error (Step Input): {sse:.4f}")

# r = RootLocus(P=0, I=0, D=0)  # P will be updated inside
# P_vals = np.linspace(0, 10, 50)  # or a list like [0, 1, 2, 3, 4]
# r.plot_all_zeros_poles_for_P_range(P_vals)
