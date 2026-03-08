using Unity.Mathematics;
using Unity.Burst;
using UnityEngine;
using Unity.Collections;

namespace GravityEngine2 {
    [BurstCompile]
    public class IonDrive : MonoBehaviour, GSExternalAcceleration {
        [Header("Engine Accel (m/s^2)")]
        public double accelSI = 1.0;  // scaling ??
        [Header("Engine times (in world time)")]
        public double timeStart = 0.0;
        public double timeEnd = 1000.0;

        private int extId = -1;
        private GECore ge;

        public int AddToGE(int id, GECore ge, GBUnits.Units units)
        {
            this.ge = ge;
            double3[] data = new double3[1];
            double scaleT = ge.GEScaler().ScaleTimeWorldToGE(1.0);
            double accelGE = ge.GEScaler().ScaleAccelWorldToGE(accelSI);
            double timeStartGE = timeStart * scaleT;
            double timeEndGE = timeEnd * scaleT;
            data[0] = new double3(accelGE, timeStartGE, timeEndGE);
            extId = ge.ExternalAccelerationAdd(id,
                                                ExternalAccel.ExtAccelType.SELF,
                                                ExternalAccel.AccelType.ION_DRIVE,
                                                data);
            return extId;
        }

        /// <summary>
        /// IonDrive
        ///
        /// Apply an acceleration of magnitude params.x IF the current physics time is between
        /// parms.y and parms.z (i.e. burn start and end time)
        ///
        /// The direction of the burn in in the direction of the current velocity vector.
        ///
        /// 
        /// </summary>
        /// <param name="t"></param>
        /// <param name="r"></param>
        /// <param name="v"></param>
        /// <param name="parms"></param>
        /// <param name="a_out"></param>
        public static (int status, double3 a_out) IonDriveAccel(ref ExternalAccel.EAStateData eaState,
                                ref ExternalAccel.EADesc eaDesc,
                                double t,
                                double dt,
                                double3 a_in,
                                NativeArray<double3> data)
        {
            double3 a_out = double3.zero;
            int b = eaDesc.paramBase;
            double tStart = data[b + 0].y;
            double tEnd = data[b + 0].z;
            if (tStart < 0)
                return (0, a_out);

            if ((t >= tStart) && (t <= tEnd)) {
                a_out = data[b + 0].x * eaState.v_from;
            }
            return (0, a_out);
        }

    }
}
