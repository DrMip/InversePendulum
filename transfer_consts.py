class TransferConsts:

        k = 2
        R = 1
        L = 0.1
        wheel_mass = 2
        wheel_radius = 0.1
        I1 = 0.5*wheel_mass*(wheel_radius)**2
        gamma = 0.001
        c = (k + R*gamma/k)*wheel_radius
        b = (R*I1/k + L*gamma/k)*wheel_radius
        a = (L*I1/k)*wheel_radius
        bp = 0.001
        l = 0.7
        mr = 0.2
        g = 9.81


        I2 = (1/3.0)*mr*(l)**2
        A = I2
        B = bp
        C = -(l/2)*mr*g
        K = (l/2)*mr
