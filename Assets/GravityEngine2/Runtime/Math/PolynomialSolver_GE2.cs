using System.Collections.Generic;
using System.Numerics;
using System;

namespace GravityEngine2 {
    public class PolynomialSolver_GE2 {

        /// <summary>
        /// Solve the equation ax^2 + bx + c = 0 for two real roots. 
        /// If solution is complex, return NaN. 
        /// 
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="c"></param>
        /// <returns></returns>
        public static double[] Quadratic(double a, double b, double c)
        {
            double[] soln = new double[] { double.NaN, double.NaN };

            double radical = b * b - 4.0 * a * c;
            if (radical > 0) {
                soln[0] = 0.5 * (-b + Math.Sqrt(radical)) / a;
                soln[1] = 0.5 * (-b - Math.Sqrt(radical)) / a;
            }

            return soln;
        }

        /// <summary>
        /// Solutions of the cubic y^3 + p y^2 + q y + r = 0
        /// 
        /// From CRC Standard Math Tables, 28th Edition p9.
        /// 
        /// </summary>
        /// <param name="p"></param>
        /// <param name="q"></param>
        /// <param name="r"></param>
        /// <returns></returns>
        /// DOES not work. Something to do with the choice of cube roots maybe?
        public static Complex[] BrokenCubic(double p, double q, double r)
        {
            //Debug.LogFormat("p={0} q={1} r={2}", p, q, r);
            Complex[] root = new Complex[3];

            double a = (1.0 / 3.0) * (3.0 * q - p * p);
            double b = (1.0 / 27.0) * (2.0 * p * p * p - 9.0 * p * q + 27.0 * r);

            Complex D = Complex.Sqrt(0.25 * b * b + (1.0 / 27.0) * a * a * a);
            Complex A = Complex.Pow(-0.5 * b + D, (1.0 / 3.0));
            Complex B = -Complex.Pow(0.5 * b + D, (1.0 / 3.0));
            //Debug.LogFormat("a={0} b={1} A={2} B={3} D={4} A-B={5}", a, b, A, B, D, A-B);
            Complex sqrtm3 = Complex.Sqrt(-3.0);
            double pover3 = -p / 3.0;
            root[0] = pover3 + A + B;
            root[1] = pover3 - 0.5 * (A + B) + 0.5 * (A - B) * sqrtm3;
            root[2] = pover3 - 0.5 * (A + B) - 0.5 * (A - B) * sqrtm3;
            //Debug.LogFormat("roots {0} {1} {2}", root[0], root[1], root[2]);
            return root;
        }


        /// <summary>
        /// Solve ax^3+bx^2+cx+d=0 for x.
        /// Calculation of the 3 roots of a cubic equation according to
        /// https://en.wikipedia.org/wiki/Cubic_equation#General_cubic_formula
        /// Using the complex struct from System.Numerics
        /// Visual Studio 2010, .NET version 4.0
        /// (Initial Implementation from 
        /// https://www.daniweb.com/programming/software-development/code/454493/solving-the-cubic-equation-using-the-complex-struct
        /// (added fix for initial C=0)
        /// </summary>
        /// <param name="a">real coefficient of x to the 3th power</param>
        /// <param name="b">real coefficient of x to the 2nd power</param>
        /// <param name="c">real coefficient of x to the 1th power</param>
        /// <param name="d">real coefficient of x to the zeroth power</param>
        /// <returns>A list of 3 complex numbers</returns>
        public static Complex[] SolveCubic(double a, double b, double c, double d)
        {
            const int NRoots = 3;
            Complex[] root = new Complex[3];
            double SquareRootof3 = Math.Sqrt(3);
            // the 3 cubic roots of 1
            List<Complex> CubicUnity = new List<Complex>(NRoots)
                            { new Complex(1, 0), new Complex(-0.5, -SquareRootof3 / 2.0), new Complex(-0.5, SquareRootof3 / 2.0) };
            // intermediate calculations
            double DELTA = 18 * a * b * c * d - 4 * b * b * b * d + b * b * c * c - 4 * a * c * c * c - 27 * a * a * d * d;
            double DELTA0 = b * b - 3 * a * c;
            double DELTA1 = 2 * b * b * b - 9 * a * b * c + 27 * a * a * d;
            Complex DELTA2 = -27 * a * a * DELTA;
            Complex C = Complex.Pow((DELTA1 + Complex.Pow(DELTA2, 0.5)) / 2, 1 / 3.0);
            if (C.Magnitude < 1E-6) {
                C = Complex.Pow((DELTA1 - Complex.Pow(DELTA2, 0.5)) / 2, 1 / 3.0);
            }
            for (int i = 0; i < NRoots; i++) {
                Complex M = CubicUnity[i] * C;
                Complex r = -1.0 / (3 * a) * (b + M + DELTA0 / M);
                root[i] = r;
            }
            return root;
        }

        /// <summary>
        /// Solve a Quartic of the form: x^4 + a x^3 + b x^2 + c x + d = 0 
        /// 
        /// CRC Math Handbook. 28th Ed. p 12
        /// 
        /// <returns>array of solutions</returns>
        public static Complex[] Quartic(double a, double b, double c, double d)
        {

            Complex[] root = new Complex[4];

            Complex[] cubicRoots = SolveCubic(1.0, -b, (a * c - 4.0 * d), (4.0 * b * d - c * c - a * a * d));
            // "let y be any root"
            Complex y = cubicRoots[0];

            Complex R = Complex.Sqrt(0.25 * a * a - b + y);

            Complex D, E;
            if (R.Magnitude > 1E-5) {
                D = Complex.Sqrt(0.75 * (a * a) - R * R - 2.0 * b + (4.0 * a * b - 8.0 * c - a * a * a) / (4.0 * R));
                E = Complex.Sqrt(0.75 * (a * a) - R * R - 2.0 * b - (4.0 * a * b - 8.0 * c - a * a * a) / (4.0 * R));
            } else {
                D = Complex.Sqrt(0.75 * a * a - 2.0 * b + 2.0 * Complex.Sqrt(y * y - 4.0 * d));
                E = Complex.Sqrt(0.75 * a * a - 2.0 * b - 2.0 * Complex.Sqrt(y * y - 4.0 * d));
            }
            root[0] = -0.25 * a + 0.5 * R + 0.5 * D;
            root[1] = -0.25 * a + 0.5 * R - 0.5 * D;
            root[2] = -0.25 * a - 0.5 * R + 0.5 * E;
            root[3] = -0.25 * a - 0.5 * R - 0.5 * E;
            return root;
        }

    }

}
