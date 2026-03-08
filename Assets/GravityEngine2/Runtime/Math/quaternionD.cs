using System;
using System.Runtime.CompilerServices;
using Unity.Mathematics;
using static Unity.Mathematics.math;

namespace GravityEngine2 {
    // Cut & Paste just enough from 
    // to get Angle Axis and rotation of double3 to work
    // based on: 
    // https://github.com/Unity-Technologies/Unity.Mathematics/blob/master/src/Unity.Mathematics/quaternion.cs

    //[Il2CppEagerStaticClassConstruction]
    [Serializable]
    public struct quaternionD : IFormattable {
        /// <summary>
        /// The quaternion component values.
        /// </summary>
        public double4 value;

        /// <summary>A quaternion representing the identity transform.</summary>
        public static readonly quaternionD identity = new quaternionD(0.0f, 0.0f, 0.0f, 1.0f);

        /// <summary>Constructs a quaternion from four double values.</summary>
        /// <param name="x">The quaternion x component.</param>
        /// <param name="y">The quaternion y component.</param>
        /// <param name="z">The quaternion z component.</param>
        /// <param name="w">The quaternion w component.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public quaternionD(double x, double y, double z, double w) { value.x = x; value.y = y; value.z = z; value.w = w; }

        /// <summary>Constructs a quaternion from double4 vector.</summary>
        /// <param name="value">The quaternion xyzw component values.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public quaternionD(double4 value) { this.value = value; }

        /// <summary>Implicitly converts a double4 vector to a quaternion.</summary>
        /// <param name="v">The quaternion xyzw component values.</param>
        /// <returns>The quaternion constructed from a double4 vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator quaternionD(double4 v) { return new quaternionD(v); }

        /// <summary>
        /// Returns a quaternion representing a rotation around a unit axis by an angle in radians.
        /// The rotation direction is clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="axis">The axis of rotation.</param>
        /// <param name="angle">The angle of rotation in radians.</param>
        /// <returns>The quaternion representing a rotation around an axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternionD AxisAngle(double3 axis, double angle)
        {
            double sina = math.sin(0.5 * angle);
            double cosa = math.cos(0.5 * angle);
            return new quaternionD(double4(axis * sina, cosa));
        }

        // modify based on https://stackoverflow.com/questions/77132252/unity-quaternion-lookrotationx-and-quaternion-fromtorotationvector3-forward
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternionD FromToRotation(double3 from, double3 to)
        {
            double3 fromN = math.normalize(from);
            double3 toN = math.normalize(to);
            double ftdot = math.dot(fromN, toN);
            if (math.abs(ftdot) > 1.0 - 1E-6) {
                return identity;
            }
            double3 axis = math.normalize(math.cross(fromN, toN));
            double halfangle = 0.5 * math.acos(ftdot);
            return new quaternionD(double4(axis * math.sin(halfangle), math.cos(halfangle)));
        }

        /// <summary>Returns a normalized version of a quaternion q by scaling it by 1 / length(q).</summary>
        /// <param name="q">The quaternion to normalize.</param>
        /// <returns>The normalized quaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternionD Normalize(quaternionD q)
        {
            double4 x = q.value;
            return new quaternionD(rsqrt(dot(x, x)) * x);
        }

        /// <summary>Returns the result of transforming a vector by a quaternion.</summary>
        /// <param name="q">The quaternion transformation.</param>
        /// <param name="v">The vector to transform.</param>
        /// <returns>The transformation of vector v by quaternion q.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double3 mul(quaternionD q, double3 v)
        {
            double3 t = 2 * cross(q.value.xyz, v);
            return v + q.value.w * t + cross(q.value.xyz, t);
        }

        /// <summary>Returns a string representation of the quaternion.</summary>
        /// <returns>The string representation of the quaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return string.Format("quaternion({0}f, {1}f, {2}f, {3}f)", value.x, value.y, value.z, value.w);
        }

        /// <summary>Returns a string representation of the quaternion using a specified format and culture-specific format information.</summary>
        /// <param name="format">The format string.</param>
        /// <param name="formatProvider">The format provider to use during string formatting.</param>
        /// <returns>The formatted string representation of the quaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public string ToString(string format, IFormatProvider formatProvider)
        {
            return string.Format("quaternion({0}f, {1}f, {2}f, {3}f)", value.x.ToString(format, formatProvider), value.y.ToString(format, formatProvider), value.z.ToString(format, formatProvider), value.w.ToString(format, formatProvider));
        }
    }
}
