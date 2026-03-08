using MathNet.Numerics.Interpolation;
using Unity.Collections;
using Unity.Mathematics;

namespace GravityEngine2 {
    /// <summary>
    /// Code used by GEPhysicsCore to evolve according to an ephemeris table. 
    /// The data values have been repackaged by GECore and converted into GE units
    /// by the time this code acts on them.
    /// </summary>
    public class EphemerisPropagator {

        public enum Interpolate { LINEAR, CUBIC_SPLINE, KEPLER };
        public struct PropInfo {
            public int baseIndex;
            public int numPoints;
            public double tStart;
            // if there is no regular interval, then set this negative
            public double tInterval;
            // ephem data may be relative to some other object
            public bool relative;
            public int centerId;
            public Interpolate interpolate;
        }

        public static void Evolve(double t,
                                  ref GEPhysicsCore.GEBodies bodies,
                                  ref NativeArray<PropInfo> propInfo,
                                  ref NativeArray<int> indices,
                                  int lenEphemBodies,
                                  ref NativeArray<GEBodyState> ephemerisData)
        {
            int pIndex;
            // TODO: Needs to be a struct
            CubicSpline[] splines = new CubicSpline[6];
            for (int j = 0; j < lenEphemBodies; j++) {
                int i = indices[j];
                pIndex = bodies.propIndex[i];

                (double3 r, double3 v) = InterpolateOneBody(t, pIndex, ref propInfo, ref ephemerisData, bodies.mu[propInfo[pIndex].centerId]);
                bodies.r[i] = r;
                bodies.v[i] = v;
                if (propInfo[pIndex].relative) {
                    bodies.r[i] += bodies.r[propInfo[pIndex].centerId];
                    bodies.v[i] += bodies.v[propInfo[pIndex].centerId];
                }

            }
        }


        private static (double3 r, double3 v) InterpolateOneBody(double t,
                                                          int pIndex,
                                                          ref NativeArray<PropInfo> propInfo,
                                                          ref NativeArray<GEBodyState> ephemerisData,
                                                          double mu)
        {
            CubicSpline[] splines = new CubicSpline[6];
            double3 r = new double3(0, 0, 0);
            double3 v = new double3(0, 0, 0);
            int bIndex = propInfo[pIndex].baseIndex;
            int tIndex;
            if (propInfo[pIndex].tInterval > 0) {
                tIndex = (int)((t - propInfo[pIndex].tStart) / propInfo[pIndex].tInterval);

                // hold on last entry for now (could flip to Kepler if relative)
                bool interpolate = true;
                if (tIndex >= propInfo[pIndex].numPoints - 1) {
                    tIndex = propInfo[pIndex].numPoints - 1;
                    if (propInfo[pIndex].interpolate == Interpolate.KEPLER) {
                        // use last point for Kepler interpolation
                        interpolate = true;
                    } else {
                        interpolate = false;
                    }
                } else if (tIndex < 0) {
                    tIndex = 0;
                    interpolate = false;
#if UNITY_EDITOR
                    UnityEngine.Debug.LogWarningFormat("tIndex < 0 for ephemeris body {0} at t={1}", pIndex, t);
#endif
                }
                // HACK - GMAT intervals are not quite exact. Maybe rounding to seconds or something??
                if (t < ephemerisData[bIndex + tIndex].t && tIndex > 0) {
                    tIndex -= 1;
                }
                // TODO: Cache splines and RVT
                if (interpolate) {
                    int numPoints = propInfo[pIndex].numPoints;
                    switch (propInfo[pIndex].interpolate) {
                        case Interpolate.CUBIC_SPLINE:
                            int baseIndex = bIndex + tIndex - 1;
                            if (baseIndex < 0)
                                baseIndex = 0;
                            else if (baseIndex + 4 >= numPoints)
                                baseIndex = numPoints - 5;

                            baseIndex += bIndex;
                            double[] x = new double[] {ephemerisData[baseIndex].t, ephemerisData[baseIndex+1].t,
                                       ephemerisData[baseIndex + 2].t, ephemerisData[baseIndex + 3].t };
                            for (int k = 0; k < 3; k++) {
                                double[] y = new double[] {ephemerisData[baseIndex].r[k], ephemerisData[baseIndex+1].r[k],
                                       ephemerisData[baseIndex + 2].r[k], ephemerisData[baseIndex + 3].r[k] };
                                double[] dy = new double[] {ephemerisData[baseIndex].v[k], ephemerisData[baseIndex+1].v[k],
                                       ephemerisData[baseIndex + 2].v[k], ephemerisData[baseIndex + 3].v[k] };
                                splines[k] = CubicSpline.InterpolateHermiteSorted(x, y, dy);
                                // don't have derivs for V
                                splines[k + 3] = CubicSpline.InterpolatePchipSorted(x, dy);
                            }
                            r = new double3(splines[0].Interpolate(t), splines[1].Interpolate(t), splines[2].Interpolate(t));
                            v = new double3(splines[3].Interpolate(t), splines[4].Interpolate(t), splines[5].Interpolate(t));
                            break;

                        case Interpolate.LINEAR:
                            double f = (t - propInfo[pIndex].tStart - propInfo[pIndex].tInterval * tIndex) / propInfo[pIndex].tInterval;
                            r = (1.0 - f) * ephemerisData[bIndex + tIndex].r +
                                            f * ephemerisData[bIndex + tIndex + 1].r;
                            v = (1.0 - f) * ephemerisData[bIndex + tIndex].v +
                                            f * ephemerisData[bIndex + tIndex + 1].v;
                            break;

                        case Interpolate.KEPLER:
                            KeplerPropagator.RVT rvt = new KeplerPropagator.RVT(ephemerisData[bIndex + tIndex].r, ephemerisData[bIndex + tIndex].v, ephemerisData[bIndex + tIndex].t, mu);
                            (int status, double3 r_v, double3 v_v) = KeplerPropagator.RVforTime(rvt, t);
                            r = r_v;
                            v = v_v;
                            break;
                    }

                } else {
                    r = ephemerisData[bIndex + tIndex].r;
                    v = ephemerisData[bIndex + tIndex].v;
                }
            }
            return (r, v);
        }

        public static void EvolveRelative(double t,
                                          int propId,
                                          ref NativeArray<PropInfo> propInfo,
                                          ref NativeArray<GEBodyState> ephemerisData,
                                          ref GEBodyState state,
                                          double mu = 0)
        {
            state.t = t;
            (state.r, state.v) = InterpolateOneBody(t, propId, ref propInfo, ref ephemerisData, mu);
        }

        /// <summary>
		/// Create a PropInfo instance and fill it in.
		///
		/// Manage the one flat ephemData array used for all ephem propagators. This is sized on demand, so an
		/// add forces a grow and a copy.
		///
		/// Implementation assumes adding bodies with ephemeris data is not a common run-time activity.
		/// </summary>
		/// <param name="ephemData">Ephemeris data in world units</param>
		/// <param name="gePhysicsJob"></param>
		/// <param name="geScaler"></param>
		/// <param name="centerId"></param>
		/// <param name="pIndex"></param>
        public static void EphemDataAlloc(EphemerisData ephemData,
                                           ref GEPhysicsCore.GEPhysicsJob gePhysicsJob,
                                           GBUnits.GEScaler geScaler,
                                           int centerId,
                                           int pIndex)
        {
            int numPoints = ephemData.NumPoints();

            // Fill in prop info and copy raw data into "all in one" ephemData in GEPhysicsCore
            int size = gePhysicsJob.ephemerisData.Length;
            int newSize = size + numPoints;
            NativeArray<GEBodyState> oldEphem = gePhysicsJob.ephemerisData;
            gePhysicsJob.ephemerisData = new NativeArray<GEBodyState>(newSize, Allocator.Persistent);
            // copyFrom requires same length, so loop
            NativeArray<GEBodyState>.Copy(oldEphem, 0, gePhysicsJob.ephemerisData, 0, size);
            oldEphem.Dispose();
            GEBodyState eState = new GEBodyState();
            double rScale = geScaler.ScaleLenWorldToGE(1.0);
            double vScale = geScaler.ScaleVelocityWorldToGE(1.0);
            double tScale = geScaler.ScaleTimeWorldToGE(1.0);
            // add new ephem data, converting to ge scale as we go
            for (int i = 0; i < numPoints; i++) {
                eState.r = rScale * ephemData.data[i].r;
                eState.v = vScale * ephemData.data[i].v;
                eState.t = tScale * ephemData.data[i].t;
                gePhysicsJob.ephemerisData[size + i] = eState;
            }

            PropInfo propInfo = new PropInfo();
            propInfo.baseIndex = size;
            propInfo.numPoints = numPoints;
            propInfo.relative = ephemData.relative;
            propInfo.centerId = centerId;
            propInfo.tInterval = (gePhysicsJob.ephemerisData[size + numPoints - 1].t - gePhysicsJob.ephemerisData[size].t) / (numPoints - 1);
            propInfo.tStart = gePhysicsJob.ephemerisData[size].t;
            propInfo.interpolate = ephemData.interpolate;
            //Debug.LogFormat("tend={0} tstart={1} interval={2}", gePhysicsJob.ephemerisData[numPoints - 1].t,
            //    gePhysicsJob.ephemerisData[0].t, propInfo.tInterval);
            gePhysicsJob.ephemPropInfo[pIndex] = propInfo;

        }

        /// <summary>
		/// Free the memory in the global flat ephemData array.
		///
		/// This is a klunky, greedy implementation:
		/// - find the entry range
		/// - copy down everything higher
		/// 
		/// </summary>
		/// <param name="gePhysicsJob"></param>
		/// <param name="pIndex"></param>
        public static void EphemDataFree(ref GEPhysicsCore.GEPhysicsJob gePhysicsJob, int pIndex)
        {
            NativeArray<GEBodyState> oldEphem = gePhysicsJob.ephemerisData;
            int n = gePhysicsJob.ephemPropInfo[pIndex].numPoints;
            int newSize = gePhysicsJob.ephemerisData.Length - n;
            int baseIndex = gePhysicsJob.ephemPropInfo[pIndex].baseIndex;
            gePhysicsJob.ephemerisData = new NativeArray<GEBodyState>(newSize, Allocator.Persistent);
            UnityEngine.Debug.LogFormat("Freeing {0} points, new size {1} base={2}", n, newSize, baseIndex);
            if (newSize > 0) {
                // copy over up to the gap
                NativeArray<GEBodyState>.Copy(oldEphem, 0, gePhysicsJob.ephemerisData, 0, baseIndex);
                // copy from end of gap to the end
                if (baseIndex < newSize) {
                    NativeArray<GEBodyState>.Copy(oldEphem, baseIndex + n, gePhysicsJob.ephemerisData, baseIndex, newSize - baseIndex - n);
                }
            }
            oldEphem.Dispose();
            // now fix up the base index value for entries that were shuffled down
            for (int i = 0; i < gePhysicsJob.ephemPropInfo.Length; i++) {
                if (gePhysicsJob.ephemPropInfo[i].numPoints > 0 && gePhysicsJob.ephemPropInfo[i].baseIndex > baseIndex) {
                    PropInfo propInfo = gePhysicsJob.ephemPropInfo[i];
                    propInfo.baseIndex -= n;
                    gePhysicsJob.ephemPropInfo[i] = propInfo;
                }
            }
        }

    }
}
