using System;

namespace GravityEngine2 {
    /// <summary>
    /// Secant root find method. 
    /// 
    /// Implementation based on Code 5.3 from Gezerlis (2020) "Numerical Methods in Physics with Python"
    /// 
    /// </summary>
    public class SecantRootFind_GE2 {

        public delegate double Function(double x);

        /// <summary>
        /// Secant root finder. 
        /// </summary>
        /// <param name="f">Function to find root of (delegate)</param>
        /// <param name="x0">An point on the function</param>
        /// <param name="x1">A second point on the function (near x0)</param>
        /// <param name="kmax">Max iterations</param>
        /// <param name="tol">Tolerance for solution</param>
        /// <returns></returns>
        public static double Secant(Function f, double x0, double x1, int kmax = 200, double tol = 1E-8)
        {
            double f0 = f(x0);
            double f1, xdiff, ratio;
            double x2 = double.NaN;
            for (int k = 0; k < kmax; k++) {
                f1 = f(x1);
                // NB - bail if function becomes ill behaved
                if (double.IsNaN(f1))
                    return double.NaN;
                // NB - added denom check 
                if (Math.Abs(f1 - f0) < tol)
                    break;
                ratio = (x1 - x0) / (f1 - f0);
                x2 = x1 - f1 * ratio;

                xdiff = Math.Abs(x2 - x1);
                x0 = x1;
                x1 = x2;
                f0 = f1;
                if (Math.Abs(xdiff / x2) < tol)
                    break;
            }

            return x2;
        }
    }
}
