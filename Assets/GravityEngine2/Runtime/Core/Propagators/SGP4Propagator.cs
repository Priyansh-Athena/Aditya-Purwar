using Unity.Mathematics;
using Unity.Burst;
using Unity.Collections;

namespace GravityEngine2 {
    /// <summary>
    /// Code to propagate SGP4 satellites i.e. satellites in Earth orbit with modeling for atmosphere, oblate gravity, moon, sun etc. 
    /// 
    /// This code assumes the Earth is at the center of physics space. 
    /// 
    /// Not hard to C&P the center body code from Kepler but seems like an uncommon use-case, so wait until someone asks....
    /// </summary>
    public class SGP4Propagator {

        private static double BLEND_TIME_PERIOD_FRACTION = 0.05;
        public struct PropInfo {
            public SGP4SatData satData;
            public int centerId;
            // 0=ok. > 0 a new error code has been reported.
            // -1 This body has been inactivated and should not evolve
            public int status;

            // Blending: when an SGP4 is created from RVT (or due to a maneuver) the immediate position from 
            // the newly inited satData may be discontinuous with the previous position. To avoid a jump in this
            // and the velocity, we init a Kepler RVT and lerp between the Kepler state and the SGP4 state over
            // a short period, typically about 2% of the orbit period.
            // If blend time is negative, blending is skipped.
            public double blendTimeSec;
            public KeplerPropagator.RVT keplerBlendRVTSIkm;

            public PropInfo(SGP4SatData satData, int centerId)
            {
                this.satData = satData;
                this.centerId = centerId;
                status = 0;
                blendTimeSec = -1.0;
                keplerBlendRVTSIkm = new KeplerPropagator.RVT();
            }

            public PropInfo(PropInfo copyFrom)
            {
                satData = copyFrom.satData;
                centerId = copyFrom.centerId;
                status = copyFrom.status;
                blendTimeSec = copyFrom.blendTimeSec;
                keplerBlendRVTSIkm = copyFrom.keplerBlendRVTSIkm;
            }

            public string LogString()
            {
                return string.Format("SGP4PropInfo: satData={0} centerId={1} status={2} blendTimeSec={3} keplerBlendRVTSIkm={4}",
                    satData.LogString(), centerId, status, blendTimeSec, keplerBlendRVTSIkm.ToString());
            }

        }

        // PropInfoError codes
        public const int OK = 0;
        public const int SGP4_ERROR = 1;
        public const int EARLY_PROPAGATION_ERROR = 2;

        /// <summary>
        /// GE entry point, to evolve a set of SGP4 bodies as defined by the indices index list. 
        /// 
        /// Individual bodies may have different outcomes (decay, early propagation error) etc. This
        /// is indicated by setting the status field of the PropInfo struct and setting a return flag. The
        /// caller must then check all bodies. 
        /// </summary>
        /// <param name="tSec">time in seconds</param>
        /// <param name="bodies"></param>
        /// <param name="propInfo"></param>
        /// <param name="patchInfo"></param>
        /// <param name="indices"></param>
        /// <param name="scaleLtoKm"></param>
        /// <param name="scaleKmsecToV"></param>
        /// <param name="startTimeJD"></param>
        [BurstCompile(CompileSynchronously = true)]
        public static bool EvolveAll(double tSec,
                 ref GEPhysicsCore.GEBodies bodies,
                 ref NativeArray<PropInfo> propInfo,
                 in NativeArray<GEPhysicsCore.PatchInfo> patchInfo,
                 in NativeArray<int> indices,
                 int lenIndices,
                 double scaleLtoKm,
                 double scaleKmsecToV,
                 double startTimeJD)
        {
            bool checkStatus = false;
            double3 r, v;
            Orbital.COEStruct coe;
            int p;
            int status;
            double scaleInv = 1.0 / scaleLtoKm;
            double t_since;
            for (int j = 0; j < lenIndices; j++) {
                int i = indices[j];
                if (bodies.self_integrating[i]) {
                    continue;
                }
                if (bodies.patchIndex[i] >= 0) {
                    p = patchInfo[bodies.patchIndex[i]].propIndex;
                } else {
                    p = bodies.propIndex[i];
                }

                // Prop needs minutes from start of SatRec epoch
                t_since = tSec * GBUnits.MIN_PER_SEC + (startTimeJD - propInfo[p].satData.jdsatepoch) * 24.0 * 60.0;

                if (t_since < 0) {
                    if (bodies.earlyPropagation[i] == GEPhysicsCore.EarlyPropagation.ERROR_INACTIVATE) {
                        PropInfo pInfo = new PropInfo(propInfo[p].satData, propInfo[p].centerId);
                        pInfo.status = EARLY_PROPAGATION_ERROR;
                        propInfo[p] = pInfo;
                        checkStatus = true;
                        continue;
                    } else if (bodies.earlyPropagation[i] == GEPhysicsCore.EarlyPropagation.HOLD_VALUE) {
                        t_since = 0.0;
                    }
                }

                (status, r, v, coe) = SGP4unit.SGP4Prop(ref propInfo, p, t_since);
                if (status != 0) {
                    // UnityEngine.Debug.LogErrorFormat("error={0} {1}", status, SGP4SatData.ErrorString(status));
                    // need to copy struct to set status field
                    PropInfo pInfo = new PropInfo(propInfo[p].satData, propInfo[p].centerId);
                    pInfo.status = SGP4_ERROR;
                    propInfo[p] = pInfo;
                    checkStatus = true;
                    // leave r and v as is, so we can see where the satellite decayed
                } else {
                    if (propInfo[p].blendTimeSec > 0.0) {
                        if ((tSec - propInfo[p].keplerBlendRVTSIkm.t0) < propInfo[p].blendTimeSec) {
                            double blendFrac = System.Math.Min(1.0, (tSec - propInfo[p].keplerBlendRVTSIkm.t0) / propInfo[p].blendTimeSec);
                            (int statusBlend, double3 rKm, double3 vKm) = KeplerPropagator.RVforTime(propInfo[p].keplerBlendRVTSIkm, tSec);
                            r = (1.0 - blendFrac) * rKm + blendFrac * r;
                            v = (1.0 - blendFrac) * vKm + blendFrac * v;
                        }
                    }
                    r *= scaleInv;
                    v *= scaleKmsecToV;
                    coe.ScaleLength(scaleInv);
                    bodies.r[i] = r + bodies.r[propInfo[p].centerId];
                    bodies.v[i] = v + bodies.v[propInfo[p].centerId];
                    bodies.coe[i] = coe;
                }
            }
            return checkStatus;
        }

        /// <summary>
        /// Evolve an SGP4 propagator to a given time. 
        /// 
        /// This entry point is used by StateForIdAtTime to evolve an SGP4 propagator to a given time.
        /// 
        /// </summary>
        /// <param name="tSec"></param>
        /// <param name="propInfo"></param>
        /// <param name="i"></param>
        /// <param name="state"></param>
        /// <param name="scaleLtoKm"></param>
        /// <param name="scaleKmsecToV"></param>
        /// <param name="startTimeJD"></param>
        /// <returns></returns>
        [BurstCompile(CompileSynchronously = true)]
        public static bool Evolve(double tSec,
         ref NativeArray<PropInfo> propInfo,
         int i,
         ref GEBodyState state,
         double scaleLtoKm,
         double scaleKmsecToV,
         double startTimeJD)
        {
            state.r = new double3(double.NaN, double.NaN, double.NaN);
            state.v = new double3(double.NaN, double.NaN, double.NaN);
            state.t = double.NaN;
            bool checkStatus = false;
            double3 r, v;
            Orbital.COEStruct coe;
            int status;
            double scaleInv = 1.0 / scaleLtoKm;
            double t_since;

            // if a previous cycle had an error, or status set to -1 to skip then don't evolve this body
            if (propInfo[i].status != 0)
                return false;

            // Prop needs minutes from start of SatRec epoch
            t_since = tSec * GBUnits.MIN_PER_SEC + (startTimeJD - propInfo[i].satData.jdsatepoch) * 24.0 * 60.0;

            (status, r, v, coe) = SGP4unit.SGP4Prop(ref propInfo, i, t_since);
            if (status != 0) {
                UnityEngine.Debug.LogErrorFormat("error={0} {1}", status, SGP4SatData.ErrorString(status));
                // need to copy struct to set status field
                PropInfo pInfo = new PropInfo(propInfo[i].satData, propInfo[i].centerId);
                pInfo.status = status;
                propInfo[i] = pInfo;
                checkStatus = true;
            }
            if (propInfo[i].blendTimeSec > 0.0) {
                if ((tSec - propInfo[i].keplerBlendRVTSIkm.t0) < propInfo[i].blendTimeSec) {
                    double blendFrac = System.Math.Min(1.0, (tSec - propInfo[i].keplerBlendRVTSIkm.t0) / propInfo[i].blendTimeSec);
                    (int statusBlend, double3 rKm, double3 vKm) = KeplerPropagator.RVforTime(propInfo[i].keplerBlendRVTSIkm, tSec);
                    r = (1.0 - blendFrac) * r + blendFrac * rKm;
                    v = (1.0 - blendFrac) * v + blendFrac * vKm;
                }
            }
            r *= scaleInv;
            v *= scaleKmsecToV;
            state.r = r;
            state.v = v;
            state.t = tSec;
            return checkStatus;
        }

        /// <summary>
        /// Used in direct propagation mode, typically with an explicitly created SGP4 propagator that
        /// was created in test or user code (and not from within GE).
        /// 
        /// Evolution for time earlier than the epoch is allowed.
        /// 
        /// </summary>
        /// <param name="satData"></param>
        /// <param name="timeJD"></param>
        /// <param name="scaleLToKm"></param>
        /// <param name="scaleKmsecToV"></param>
        /// <returns></returns>
        [BurstCompile(CompileSynchronously = true)]
        public static (int error, double3 r, double3 v, Orbital.COEStruct coe) RVforTime(
                                                            ref SGP4SatData satData,
                                                            double timeJD,
                                                            double scaleLToKm,
                                                            double scaleKmsecToV)
        {
            // time since SatRec epoch in minutes
            double minutesSinceEpoch = (timeJD - satData.jdsatepoch) * 24.0 * 60.0;

            (int status, double3 r, double3 v, Orbital.COEStruct coe) =
                                SGP4unit.SGP4Prop(ref satData, minutesSinceEpoch);
            r /= scaleLToKm;
            v *= scaleKmsecToV;
            coe.ScaleLength(1.0 / scaleLToKm);
            return (status, r, v, coe);
        }

        /// <summary>
        /// Change the state of an SGP4 propagator from a maneuver. Current r, v have been updated, so
        /// re-calculate the SGP4 state. Blend the result back to the current SGP4 state over a short
        /// period, typically 2% of the orbit period.
        /// </summary>
        /// <param name="bodyIndex"></param>
        /// <param name="timeJD"></param>
        /// <param name="bodies"></param>
        /// <param name="sgp4Props"></param>
        /// <param name="scaleLtoKm"></param>
        [BurstCompile(CompileSynchronously = true)]
        public static void ManeuverPropagator(int bodyIndex,
                        double tSec,
                        double timeJD,
                        ref GEPhysicsCore.GEBodies bodies,
                        ref NativeArray<PropInfo> sgp4Props,
                        double scaleLtoKm,
                        double scaleKmSecToV)
        {
            int p = bodies.propIndex[bodyIndex];
            int centerId = sgp4Props[p].centerId;
            double3 r = bodies.r[bodyIndex] - bodies.r[centerId];
            double3 v = bodies.v[bodyIndex] - bodies.v[centerId];
            SGP4SatData satData = sgp4Props[p].satData;
            double3 rKm = r * scaleLtoKm;
            double3 vKm = v / scaleKmSecToV;
            Orbital.COE coe = Orbital.RVtoCOE(rKm, vKm, GBUnits.earthMuKm);
            satData.InitFromCOE(coe, timeJD);
            PropInfo pInfo = new PropInfo(satData, sgp4Props[p].centerId);
            pInfo.blendTimeSec = BLEND_TIME_PERIOD_FRACTION * coe.GetPeriod();
            pInfo.keplerBlendRVTSIkm = new KeplerPropagator.RVT(rKm, vKm, t0: tSec, mu: GBUnits.earthMuKm);
            sgp4Props[p] = pInfo;
        }

        /// <summary>
        /// Re-initialize an SGP4 propagator from RV info. 
        /// 
        /// Setup blending to the current SGP4 state over a short period, typically 5% of the orbit period.
        /// 
        /// PropInfo struct is copied by value, so return after updating.
        /// </summary>
        /// <param name="tSec"></param>
        /// <param name="timeJD"></param>
        /// <param name="rKm"></param>
        /// <param name="vKm"></param>
        /// <param name="coeKm"></param>
        /// <param name="satData"></param>
        /// <param name="pInfo"></param>
        [BurstCompile(CompileSynchronously = true)]
        public static PropInfo ReInit(
                        double tSec,
                        double timeJD,
                        double3 rKm,
                        double3 vKm,
                        Orbital.COE coeKm,
                        ref SGP4SatData satData,
                        PropInfo pInfo)
        {
            satData.InitFromCOE(coeKm, timeJD);
            pInfo.blendTimeSec = BLEND_TIME_PERIOD_FRACTION * coeKm.GetPeriod();
            pInfo.keplerBlendRVTSIkm = new KeplerPropagator.RVT(rKm, vKm, t0: tSec, mu: GBUnits.earthMuKm);
            return pInfo;
        }

    }
}
