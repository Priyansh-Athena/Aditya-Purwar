using Unity.Mathematics;
using Unity.Burst;
using UnityEngine;
using Unity.Collections;


namespace GravityEngine2 {
    [BurstCompile]
    public class GravityJ2 : MonoBehaviour, GSExternalAcceleration {
        public double j2 = 1.0;  // J2 is DL ??
        public double radius = 10.0;
        public double3 axis;

        public int AddToGE(int id, GECore ge, GBUnits.Units units)
        {
            // scale radius to internal GE units
            GBUnits.GEScaler geScaler = ge.GEScaler();
            if (units != geScaler.WorldUnits()) {
                radius *= GBUnits.DistanceConversion(units, geScaler.WorldUnits());
            }
            radius = ge.GEScaler().ScaleLenWorldToGE(radius);
            double3[] data = new double3[2];
            data[0] = new double3(j2, radius, 0);
            data[1] = axis;
            return ge.ExternalAccelerationAdd(id,
                                        ExternalAccel.ExtAccelType.ON_OTHER,
                                        ExternalAccel.AccelType.GRAVITY_J2,
                                        data);
        }

        /// <summary>
        /// Apply the addition force change due to oblate gravity modeled by a J2 dipole term.
        ///
        /// The equation for the perturbing force is taken from Gravity (Poisson) (3.90) p166
        ///
        /// This routine is called for each massive (i,j) pair. Those that have a "from body"
        /// with a non-zero J2 will apply the non-spherical perturbing force to the other.
        ///
        /// There really should be an equal and opposite force on the source of the J2 force but since this
        /// is most often used for a planet force on a satellite, this is neglected since it's so small.
        /// 
        /// </summary>
        /// <param name="fromIndex"></param>
        /// <param name="toIndex"></param>
        /// <param name="rsq"></param>
        /// <param name="rvmu_from"></param>
        /// <param name="rvmu_to"></param>
        /// <param name="parmFrom"></param>
        /// <param name="t"></param>
        /// <param name="afrom_out"></param>
        /// <param name="a"></param>
        public static (int status, double3 a_out) J2Accel(ref ExternalAccel.EAStateData eaState,
                               ref ExternalAccel.EADesc eaDesc,
                               double t,
                               double dt,
                               double3 a_in,
                               NativeArray<double3> data)
        {
            double3 a_out = double3.zero;
            double j2 = data[0].x;
            int b = eaDesc.paramBase;
            if (j2 > 0) {
                // e is the axis direction
                double R = data[b + 0].y;
                double3 e = math.normalize(data[b + 1]);
                double3 r = eaState.r_from - eaState.r_to;
                double3 n = math.normalize(r);
                double3 Gm = eaState.mu_from;
                double eDotn = math.dot(e, n);
                double3 rji = eaState.r_to - eaState.r_from;
                double rsq = math.lengthsq(rji);
                a_out = 1.5 * j2 * Gm * R * R / (rsq * rsq) * ((5 * eDotn * eDotn - 1) * n - 2.0 * (eDotn) * e);
            }
            return (0, a_out);
        }
    }
}
