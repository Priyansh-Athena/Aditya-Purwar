using Unity.Mathematics;
using Unity.Collections;
using System;

namespace GravityEngine2 {
    /// <summary>
    /// Propagation using classical Kepler (two-body) dynamics, with state kept in COE form.
    /// 
    /// The code here is based on the KeplerCOE algorithm from Vallado.
    /// 
    /// This propogator has several differences from the KeplerPropagator:
    /// 1) It is computationally simpler than the universal variable approach. 
    /// 2) It allow propagation of the COE to times before the start time.
    /// 3) It is best used for eccentricities that are not close to 1 
    ///    (the universal variable approach is better for this).
    /// 
    /// </summary>
    public class KeplerCOEPropagator {
        public struct PropInfo {
            public Orbital.COEStruct coeGE;        // initial COE at t_start (GE units)

            public double3 r0;                     // only computed for parabolic
            public double t_start;                 // start time in GE units
            public int centerId;                   // id of central body
            public int status;                     // STATUS_OK or error

            public PropInfo(Orbital.COE coeGE, double t_start, int centerId)
            {
                this.coeGE = new Orbital.COEStruct(coeGE);  // mean anomoly is computed here
                this.t_start = t_start;
                this.centerId = centerId;
                status = STATUS_OK;
                if (IsParabolic(coeGE.e)) {
                    double3 v0 = double3.zero;
                    (r0, v0) = Orbital.COEtoRVatTime(coeGE, 0.0);
                } else {
                    r0 = double3.zero;
                }
            }

            public PropInfo(PropInfo copyFrom)
            {
                coeGE = copyFrom.coeGE;
                t_start = copyFrom.t_start;
                centerId = copyFrom.centerId;
                status = copyFrom.status;
                r0 = copyFrom.r0;
            }

            internal string LogString()
            {
                return string.Format("KEPLER_COE PropInfo t_start={0} centerId={1} COE={2}", t_start, centerId, coeGE.LogStringDegrees());
            }
        }

        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
        public static bool IsParabolic(double e)
        {
            return math.abs(e - 1.0) < 1E-3;
        }

        public const int STATUS_OK = 0;
        public const int STATUS_EARLY_PROPAGATION_ERROR = 1;

        public static int EvolveAll(double t_to,
                        ref GEPhysicsCore.GEBodies bodies,
                        ref NativeArray<PropInfo> propInfo,
                        in NativeArray<GEPhysicsCore.PatchInfo> patchInfo,
                        in NativeArray<int> indices,
                        int lenIndices)
        {
            double3 r, v;
            int p;
            int status = STATUS_OK;
            for (int j = 0; j < lenIndices; j++) {
                int i = indices[j];
                if (bodies.self_integrating[i])
                    continue;
                if (bodies.patchIndex[i] >= 0) {
                    p = patchInfo[bodies.patchIndex[i]].propIndex;
                } else {
                    p = bodies.propIndex[i];
                }
                double dtsec = t_to - propInfo[p].t_start;
                if (dtsec < 0) {
                    if (bodies.earlyPropagation[i] == GEPhysicsCore.EarlyPropagation.ERROR_INACTIVATE) {
                        PropInfo pInfo = new PropInfo(propInfo[p]);
                        pInfo.status = STATUS_EARLY_PROPAGATION_ERROR;
                        propInfo[p] = pInfo;
                        status = STATUS_EARLY_PROPAGATION_ERROR;
                        continue;
                    } else if (bodies.earlyPropagation[i] == GEPhysicsCore.EarlyPropagation.HOLD_VALUE) {
                        dtsec = 0.0;
                    }
                }
                (r, v) = KeplerProp(ref propInfo, p, dtsec);
                bodies.r[i] = r + bodies.r[propInfo[p].centerId];
                bodies.v[i] = v + bodies.v[propInfo[p].centerId];
            }
            return status;
        }

        public static void EvolveRelative(double t_to, int propId, ref NativeArray<PropInfo> propInfo, ref GEBodyState state)
        {
            double dtsec = t_to - propInfo[propId].t_start;
            (double3 r, double3 v) = KeplerProp(ref propInfo, propId, dtsec);
            state.r = r;
            state.v = v;
            state.t = t_to;
        }

        public static void ManeuverPropagator(int bodyIndex,
                       double time,
                       ref GEPhysicsCore.GEBodies bodies,
                       ref NativeArray<PropInfo> kProps)
        {
            int p = bodies.propIndex[bodyIndex];
            int centerId = kProps[p].centerId;
            double3 r = bodies.r[bodyIndex] - bodies.r[centerId];
            double3 v = bodies.v[bodyIndex] - bodies.v[centerId];
            Orbital.COEStruct coe = new Orbital.COEStruct(Orbital.RVtoCOE(r, v, kProps[p].coeGE.mu));
            PropInfo kProp = new PropInfo {
                coeGE = coe,
                centerId = kProps[p].centerId,
                t_start = time,
                status = STATUS_OK
            };
            kProps[p] = kProp;
        }

        // Pure Kepler two-body propagation from COE, advancing mean anomaly by n*dt and solving for nu.
        public static (double3 r, double3 v) KeplerProp(
                                                    ref NativeArray<PropInfo> propInfo,
                                                    int index,
                                                    double dtsec)
        {
            double e = propInfo[index].coeGE.e;
            double nu = 0.0;
            if (math.abs(e - 1.0) < 1E-3) {
                // parabolic
                double B = Orbital.KeplerEqnParabolicForB(dtsec, propInfo[index].coeGE.p, propInfo[index].coeGE.mu);
                nu = Orbital.ConvertBToTrueAnomoly(B, propInfo[index].coeGE.p, math.length(propInfo[index].r0));

            } else if (e < 1.0) {
                // elliptical
                double m = propInfo[index].coeGE.meanAnom + propInfo[index].coeGE.n * dtsec;
                nu = Orbital.ConvertMeanToTrueAnomoly(m, e);
            } else {
                // hyperbolic
                double m = propInfo[index].coeGE.meanAnom + propInfo[index].coeGE.n * dtsec;
                double H = Orbital.KeplerEqnHyperbolicForH(m, e);
                nu = Orbital.ConvertHToTrueAnomoly(H, e);
            }
            (double3 r2, double3 v2) = Orbital.COEStructWithPhasetoRVRelative(propInfo[index].coeGE, nu);
            return (r2, v2);
        }

    }
}





