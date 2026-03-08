using System.Diagnostics;
using Unity.Mathematics;

namespace GravityEngine2 {
    public class PoweredExplicitGuidance {
        // """
        // powered_explicit_guidance.py
        // Created on Mon Feb 22 13:47:41 2016

        // Copy and recreation of matlab code from https://github.com/Noiredd/PEGAS

        // Implementation of Powered Explicit Guidance major loop.
        // http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19660006073.pdf
        // http://www.orbiterwiki.org/wiki/Powered_Explicit_Guidance

        // Dependencies: lib_physicalconstants.py

        // @author: sig
        // """
        // Converted to C# with Cursor and then moved to Unity.Mathematics by nbodyphysics

        // # dt     - length of the major computer cycle (time between PEG calculations) [s]
        // # alt    - current altitude as distance from body center [m]
        // # vt     - tangential velocity (horizontal - orbital) [m/s]
        // # vr     - radial velocity (veritcal - away from the Earth) [m/s]
        // # tgt    - target altitude (measured as current) [m]
        // # acc    - current vehicle acceleration [m/s^2]
        // # ve     - effective exhaust velocity (isp*g0) [m/s]
        // # basing on current state vector, vehicle params and given ABT estimates
        // # new A and B, and calculates new T from them
        // # also outputs a C component for the purposes of guidance
        // # passing A=B=0 causes estimation of those directly from the given oldT

        // This treatment seems to be following the naming conventions from the orbiter wiki
        // https://www.orbiterwiki.org/wiki/Powered_Explicit_Guidance
        public static (double A, double B, double C, double T) CalculateABCT(
            double mu, double alt, double vt, double vr, double tgt,
            double acc, double ve, double oldA, double oldB, double oldT, double dtSec)
        {
            // A sort of normalized mass - time to burn the vehicle completely as if it were all propellant [s]
            double tau = ve / acc;

            if (oldA == 0 && oldB == 0) {
                if (oldT > tau)  // Prevent NAN from logarithm due to bad estimate of T
                    oldT = 0.9 * tau;

                double b0 = -ve * math.log(1.0 - oldT / tau);
                double b1 = b0 * tau - ve * oldT;
                double c0 = b0 * oldT - b1;
                double c1 = c0 * tau - ve * oldT * oldT / 2.0;

                // Solve 2x2 matrix equation
                double[,] MA = new double[,] { { b0, b1 }, { c0, c1 } };
                double[] MB = new double[] { -vr, tgt - alt - vr * oldT };
                double[] MX = SolveMatrix2x2(MA, MB);

                oldA = MX[0];
                oldB = MX[1];
            }

            // Calculate angular momentum vectors
            double3 h_vec = math.cross(new double3(0, 0, alt), new double3(vt, 0, vr));
            double h = math.length(h_vec);

            double v_tgt = math.sqrt(mu / tgt);
            double3 ht_vec = math.cross(new double3(0, 0, tgt), new double3(v_tgt, 0, 0));
            double ht = math.length(ht_vec);

            double dh = ht - h;
            double rbar = (alt + tgt) / 2.0;

            // Vehicle performance
            double C = (mu / (alt * alt) - vt * vt / alt) / acc;
            double fr = oldA + C;

            // Estimation
            // CT is typically 0, not sure why that works out
            double CT = (mu / (tgt * tgt) - v_tgt * v_tgt / tgt) / (acc / (1.0 - oldT / tau));
            double frT = oldA + oldB * oldT + CT;
            double frdot = (frT - fr) / oldT;
            double ftheta = 1 - fr * fr / 2.0;
            double fthetadot = -(fr * frdot);
            double fthetadotdot = -frdot * frdot / 2.0;

            // Calculate velocity to gain
            double tLeft = oldT - dtSec;
            double dv = (dh / rbar + ve * tLeft * (fthetadot + fthetadotdot * tau)
                + fthetadotdot * ve * tLeft * tLeft / 2.0)
                / (ftheta + fthetadot * tau + fthetadotdot * tau * tau);


            // Estimate updated burnout time
            double T = tau * (1 - math.exp(-dv / ve));

            if (T >= 7.5) {
                // these are the constant thrust terms from p4-8 of NASA UPEG paper (equations (7a)-(7d) in Teren paper)
                double b0 = -ve * math.log(1 - T / tau);    // paper calls this L_i. Integral(a(t), 0..T)
                double b1 = b0 * tau - ve * T;              // J_i
                double c0 = b0 * T - b1;                    // S_i
                double c1 = c0 * tau - ve * T * T / 2.0;      // Q_i
                                                              // no eqn for Q_i. Why?
                double[,] MA = new double[,] { { b0, b1 }, { c0, c1 } };
                double[] MB = new double[] { -vr, tgt - alt - vr * T };
                double[] MX = SolveMatrix2x2(MA, MB);

                oldA = MX[0];
                oldB = MX[1];
            }

            return (oldA, oldB, C, T);
        }

        public static (double A, double B, double C, double T) Calculate(
            double cycleTime, double mu, double alt, double vt, double vr, double tgt,
            double acc, double ve, double oldA, double oldB, double oldT)
        {
            // A sort of normalized mass - time to burn the vehicle completely as if it were all propellant [s]
            double tau = ve / acc;

            if (oldA == 0 && oldB == 0) {
                if (oldT > tau)  // Prevent NAN from logarithm due to bad estimate of T
                    oldT = 0.9 * tau;

                double b0 = -ve * math.log(1 - oldT / tau);
                double b1 = b0 * tau - ve * oldT;
                double c0 = b0 * oldT - b1;
                double c1 = c0 * tau - ve * oldT * oldT / 2;

                // Solve 2x2 matrix equation
                double[,] MA = new double[,] { { b0, b1 }, { c0, c1 } };
                double[] MB = new double[] { -vr, tgt - alt - vr * oldT };
                double[] MX = SolveMatrix2x2(MA, MB);

                oldA = MX[0];
                oldB = MX[1];
            }

            // Calculate angular momentum vectors
            double3 h_vec = math.cross(new double3(0, 0, alt), new double3(vt, 0, vr));
            double h = math.length(h_vec);

            double v_tgt = math.sqrt(mu / tgt);
            double3 ht_vec = math.cross(new double3(0, 0, tgt), new double3(v_tgt, 0, 0));
            double ht = math.length(ht_vec);

            double dh = ht - h;
            double rbar = (alt + tgt) / 2;

            // Vehicle performance
            double C = (mu / (alt * alt) - vt * vt / alt) / acc;
            double fr = oldA + C;

            // Estimation
            double CT = (mu / (tgt * tgt) - v_tgt * v_tgt / tgt) / (acc / (1 - oldT / tau));
            double frT = oldA + oldB * oldT + CT;
            double frdot = (frT - fr) / oldT;
            double ftheta = 1 - fr * fr / 2;
            double fthetadot = -(fr * frdot);
            double fthetadotdot = -frdot * frdot / 2;

            // Calculate velocity to gain
            double dv = (dh / rbar + ve * (oldT - cycleTime) * (fthetadot + fthetadotdot * tau)
                + fthetadotdot * ve * (oldT - cycleTime) * (oldT - cycleTime) / 2)
                / (ftheta + fthetadot * tau + fthetadotdot * tau * tau);


            // Estimate updated burnout time
            double T = tau * (1 - math.exp(-dv / ve));

            if (T >= 7.5) {
                // these are the constant thrust terms from p4-8 of NASA UPEG paper
                double b0 = -ve * math.log(1 - T / tau);    // paper calls this L_i
                double b1 = b0 * tau - ve * T;              // J_i
                double c0 = b0 * T - b1;                    // S_i
                double c1 = c0 * tau - ve * T * T / 2;      // Q_i
                                                            // no eqn for Q_i. Why?
                double[,] MA = new double[,] { { b0, b1 }, { c0, c1 } };
                double[] MB = new double[] { -vr, tgt - alt - vr * T };
                double[] MX = SolveMatrix2x2(MA, MB);

                oldA = MX[0];
                oldB = MX[1];
            }

            return (oldA, oldB, C, T);
        }

        private static double[] SolveMatrix2x2(double[,] A, double[] b)
        {
            double det = A[0, 0] * A[1, 1] - A[0, 1] * A[1, 0];
            return new double[]
            {
            (A[1,1] * b[0] - A[0,1] * b[1]) / det,
            (-A[1,0] * b[0] + A[0,0] * b[1]) / det
            };
        }
    }
}
