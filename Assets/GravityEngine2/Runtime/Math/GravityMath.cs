
using Unity.Mathematics;
using UnityEngine;
using System;
using System.Runtime.CompilerServices;

namespace GravityEngine2 {
    public class GravityMath {
        // madness that these are not in C# Math
        public static readonly double DEG2RAD = System.Math.PI / 180.0;
        public static readonly double RAD2DEG = 180.0 / System.Math.PI;
        public static readonly double TWO_PI = 2.0 * Math.PI;

        public enum AxisV3 { X_AXIS, Y_AXIS, Z_AXIS };

        public static readonly Vector3 xAxis = Vector3.right;
        public static readonly Vector3 yAxis = Vector3.up;
        public static readonly Vector3 zAxis = Vector3.forward;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Axis(AxisV3 axis)
        {
            if (axis == AxisV3.X_AXIS) return xAxis;
            if (axis == AxisV3.Y_AXIS) return yAxis;
            else return zAxis;
        }

        /// <summary>
        /// Cotangent function.
        ///
        /// Adapted from Vallado's astmath CPP functions.
        ///
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Cot(double x)
        {
            double temp = Math.Tan(x);
            if (Math.Abs(temp) < 1E-8) {
                return double.NaN;
            } else {
                return 1.0 / temp;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Acosh(double x)
        {
            return Math.Log(x + Math.Sqrt(x * x - 1));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Asinh(double x)
        {
            return Math.Log(x + Math.Sqrt(x * x + 1));
        }

        public static double AngleDeltaDegreesCW(double from, double to)
        {
            if (from > to) {
                return (Math.Abs(360 - from) + to);
            } else {
                return to - from;
            }
        }

        public static double AngleDeltaRadianCW(double from, double to)
        {
            if (from > to) {
                return (Math.Abs(2.0 * Math.PI - from) + to);
            } else {
                return to - from;
            }
        }

        // TODO: Can tidy this up
        public static double MinAngleDeltaDegrees(double from, double to)
        {
            double d;
            if (to > from)
                d = to - from;
            else
                d = from - to;
            return Math.Min(d, 360.0 - d);
        }

        // public static double AngleRadiansPrograde(double3 r1, double3 v1, double3 r2)
        // {
        //     double3 r1_unit = math.normalize(r1);
        //     double3 r2_unit = math.normalize(r2);
        //     double3 v1_unit = math.normalize(v1);
        //     double3 n_orbit = math.cross(r1_unit, v1_unit);
        //     double3 n = math.cross(r1_unit, r2_unit);
        //     double n_align = math.dot(n_orbit, n);
        //     if (n_align < 0) {
        //         n *= -1.0;
        //     }
        //     double x2 = math.dot(r2_unit, r1_unit);
        //     double3 y_axis = math.cross(n, r1_unit);
        //     double y2 = math.dot(r2_unit, y_axis);
        //     double angle = math.atan2(y2, x2);
        //     if (angle < 0.0) {
        //         angle += 2.0 * Math.PI;
        //     }
        //     return angle;
        // }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double AngleRadiansPrograde(double3 r1, double3 v1, double3 r2)
        {
            double3 r1_unit = math.normalize(r1);
            double3 r2_unit = math.normalize(r2);
            double3 v1_unit = math.normalize(v1);

            double angle = math.acos(math.dot(r1_unit, r2_unit));

            double3 n_rr = math.cross(r1_unit, r2_unit);
            double3 n_rv = math.cross(r1_unit, v1_unit);
            double n_align = math.dot(n_rv, n_rr);
            if (n_align < 0) {
                angle = 2.0 * math.PI - angle;
            }
            return angle;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double AngleRadians(double3 r1, double3 r2)
        {
            double3 r1_unit = math.normalize(r1);
            double3 r2_unit = math.normalize(r2);

            return math.acos(math.dot(r1_unit, r2_unit));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double MinAngleDeltaRadians(double from, double to)
        {
            double d;
            if (to > from)
                d = to - from;
            else
                d = from - to;
            return Math.Min(d, 2.0 * Math.PI - d);
        }

        /// <summary>
        /// Return the rotation of the vector v by angleRad radians around the x axis.
        /// </summary>
        /// <param name="v"></param>
        /// <param name="angleRad"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double3 Rot1(double3 v, double angleRad)
        {
            double c = Math.Cos(angleRad);
            double s = Math.Sin(angleRad);
            return new double3(v.x, c * v.y + s * v.z, c * v.z - s * v.y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double3 Rot1Y(double3 v, double angleRad)
        {
            double c = Math.Cos(angleRad);
            double s = Math.Sin(angleRad);
            return new double3(c * v.x + s * v.z, v.y, c * v.z - s * v.x);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double3 Rot1Z(double3 v, double angleRad)
        {
            double c = Math.Cos(angleRad);
            double s = Math.Sin(angleRad);
            return new double3(c * v.x + s * v.y, c * v.y - s * v.x, v.z);
        }

        /// <summary>
        /// Return the rotation of the vector v by angleRad radians around the z axis.
        /// </summary>
        /// <param name="v"></param>
        /// <param name="angleRad"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double3 Rot3(double3 v, double angleRad)
        {
            double c = Math.Cos(angleRad);
            double s = Math.Sin(angleRad);
            return new double3(c * v.x + s * v.y, c * v.y - s * v.x, v.z);
        }

        /// <summary>
        /// Return rotation of vector by angleRad around Z in a clockwise (CW) direction.
        /// </summary>
        /// <param name="v"></param>
        /// <param name="angleRad"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double3 Rot3CW(double3 v, double angleRad)
        {
            double c = Math.Cos(angleRad);
            double s = Math.Sin(angleRad);
            return new double3(c * v.x - s * v.y, c * v.y + s * v.x, v.z);
        }

        // Double3 Utilities

        public static double3 ApplyMatrix(ref double3x3 m, double3 v)
        {
            return new double3(v.x * m.c0.x + v.y * m.c1.x + v.z * m.c2.x,
                v.x * m.c0.y + v.y * m.c1.y + v.z * m.c2.y,
                v.x * m.c0.z + v.y * m.c1.z + v.z * m.c2.z);

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Equal(double3 a, double3 b, double tol)
        {
            return (Math.Abs(a.x - b.x) < tol) && (Math.Abs(a.y - b.y) < tol) && (Math.Abs(a.z - b.z) < tol);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool EqualRelError(double3 a, double3 b, double relError)
        {
            double denom = -1;
            for (int i = 0; i < 3; i++) {
                denom = math.max(denom, math.abs(a[i]));
                denom = math.max(denom, math.abs(b[i]));
            }
            const double small = 1E-10;
            if (denom < small)
                denom = small;

            for (int i = 0; i < 3; i++) {
                if ((math.abs(a[i] - b[i]) / denom) > relError)
                    return false;
            }
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool EqualRelError(double a, double b, double relError)
        {
            double denom;
            const double small = 1E-10;
            denom = a;
            if (math.abs(a) < small)
                denom = math.max(denom, b);
            if ((math.abs(a - b) / denom) > relError)
                return false;
            return true;
        }

        // from Vector3d
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Angle(double3 from, double3 to)
        {
            return Math.Acos(Math.Clamp(math.dot(math.normalize(from), math.normalize(to)), -1d, 1d));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double AngleDeg(double3 from, double3 to)
        {
            return Math.Acos(Math.Clamp(math.dot(math.normalize(from), math.normalize(to)), -1d, 1d)) * 57.29578d;
        }

        /// <summary>
        /// Convert an angle in radian range 0..2 Pi into -Pi..Pi
        /// </summary>
        /// <param name="angle"></param>
        /// <returns></returns>
        public static double AngleNegPiToPi(double angle)
        {
            double theta = angle;
            if (angle > Math.PI)
                theta = angle - 2.0 * Math.PI;
            return theta;
        }

        /// <summary>
        /// Create a Vector3 holding the values of a double3
        /// </summary>
        /// <param name="r"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Double3ToVector3(double3 r)
        {
            return new Vector3((float)r[0], (float)r[1], (float)r[2]);
        }

        /// <summary>
        /// Convert a double3 and fill in values in an existing Vector3 (avoids a new() )
        /// </summary>
        /// <param name="r"></param>
        /// <param name="v3"></param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Double3IntoVector3(double3 r, ref Vector3 v3)
        {
            v3.x = (float)r[0];
            v3.y = (float)r[1];
            v3.z = (float)r[2];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double3 Vector3ToDouble3(Vector3 r)
        {
            return new double3(r.x, r.y, r.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Vector3IntoDouble3(Vector3 r, ref double3 d3)
        {
            d3.x = r.x;
            d3.y = r.y;
            d3.z = r.z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Double3ExchangeYZ(ref double3 v)
        {
            double tmp = v.y;
            v.y = v.z;
            v.z = tmp;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Vector3ExchangeYZ(ref Vector3 v)
        {
            float tmp = v.y;
            v.y = v.z;
            v.z = tmp;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Equals(double3 a, double3 b, double within)
        {
            return (Math.Abs(a.x - b.x) < within) &&
                (Math.Abs(a.y - b.y) < within) &&
                (Math.Abs(a.z - b.z) < within);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool HasNaN(Vector3 v)
        {
            return float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool HasNaN(double3 v)
        {
            return double.IsNaN(v.x) || double.IsNaN(v.y) || double.IsNaN(v.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double3 Matrix3x3TimesVector(double3x3 m, double3 v)
        {
            return new double3(m.c0.x * v.x + m.c1.x * v.y + m.c2.x * v.z,
                m.c0.y * v.x + m.c1.y * v.y + m.c2.y * v.z,
                m.c0.z * v.x + m.c1.z * v.y + m.c2.z * v.z);
        }

        /// <summary>
        /// Convert latitude, longitude and radius to a vector.
        /// 
        /// This has the XY plane as the equator (standard right-handed
        /// physics spherical coordinates)
        /// 
        /// If want display XZ orbital plane will need to flip YZ with
        /// one of the '*ExchangeYZ' methods above.
        /// 
        /// </summary>
        /// <param name="latitude">assume North is positive</param>
        /// <param name="longitude">assume east is positive</param>
        /// <param name="radius"></param>
        /// <returns></returns>         
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double3 LatLongToR(double latitudeDeg, double longitudeDeg, double radius)
        {
            // standard spherical coordinates
            double thetaRad = math.radians(90.0 - latitudeDeg);
            double phiRad = math.radians(longitudeDeg);
            double x = radius * math.cos(phiRad) * math.sin(thetaRad);
            double y = radius * math.sin(phiRad) * math.sin(thetaRad);
            double z = radius * math.cos(thetaRad);
            return new double3(x, y, z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]

        public static double RtoLongitude(double3 r)
        {
            return math.degrees(math.atan2(r.y, r.x));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]

        public static double RtoLatitude(double3 r)
        {
            return 90.0 - math.degrees(math.acos(r.z / math.length(r)));
        }
    }
}
