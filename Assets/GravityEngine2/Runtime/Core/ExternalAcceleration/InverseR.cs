using Unity.Mathematics;
using Unity.Burst;
using UnityEngine;
using Unity.Collections;

namespace GravityEngine2 {
    [BurstCompile]
    public class InverseR : MonoBehaviour, GSExternalAcceleration {

        public int AddToGE(int id, GECore ge, GBUnits.Units units)
        {
            return ge.ExternalAccelerationAdd(id,
                                    ExternalAccel.ExtAccelType.ON_OTHER,
                                    ExternalAccel.AccelType.INVERSE_R,
                                    null);
        }

        /// <summary>
        /// J2 force due to simple model of oblate planet.
        ///
        /// 
        /// </summary>
        /// <param name="t"></param>
        /// <param name="r"></param>
        /// <param name="v"></param>
        /// <param name="parms"></param>
        /// <param name="a_out"></param>
        public static int InvRAccel(ref ExternalAccel.EAStateData eaState,
                               ref ExternalAccel.EADesc eaDesc,
                               double t,
                               double dt,
                               ref double3 a_in,
                               NativeArray<double3> data,
                               out double3 a_out)
        {
            // first we need to undo the standard gravitational force
            a_out = -(a_in);

            // Now do the 1/R version
            double3 rji = eaState.r_to - eaState.r_from;
            double rsq = math.lengthsq(rji);
            a_out += -eaState.mu_from * rji / rsq;
            return 0;
        }
    }
}
