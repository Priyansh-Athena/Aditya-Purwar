using Unity.Mathematics;
using Unity.Collections;

namespace GravityEngine2 {
    public class Integrators {
        public enum Type { LEAPFROG, RK4 };

        public struct LeapfrogData {
            public NativeArray<double3> a;

            public void Init(int size)
            {
                a = new NativeArray<double3>(size, Allocator.Persistent);
            }

            public void InitFrom(LeapfrogData from)
            {
                a = new NativeArray<double3>(from.a.Length, Allocator.Persistent);
                a.CopyFrom(from.a);
            }

            public void Dispose()
            {
                a.Dispose();
            }

            public void GrowBy(int growBy)
            {
                NativeArray<double3> old_a = a;
                a = new NativeArray<double3>(a.Length + growBy, Allocator.Persistent);
                NativeArray<double3>.Copy(old_a, a, old_a.Length);
                old_a.Dispose();
            }
        }


        public static double LeapfrogStep(ref GEPhysicsCore.GEPhysicsJob geJob)
        {
            double dt = geJob.parms[GECore.DT_PARAM];
            geJob.parms[GECore.T_PARAM] += dt;
            double r2;
            double r3;
            double3 rji;
            double t = geJob.parms[GECore.T_PARAM];
            int i;

            if (geJob.lenMassiveBodies.Value > 0) {
                for (int a = 0; a < geJob.lenMassiveBodies.Value; a++) {
                    i = geJob.massiveBodies[a];
                    geJob.bodies.v[i] += geJob.leapfrogData.a[i] * 0.5 * dt;
                    // position updated at the top of the loop only!
                    geJob.bodies.r[i] += geJob.bodies.v[i] * dt;
                    geJob.leapfrogData.a[i] = 0.0;
                }

                int n = geJob.lenMassiveBodies.Value;
                // calc a
                for (int a = 0; a < n; a++) {
                    i = geJob.massiveBodies[a];
                    for (int b = a + 1; b < n; b++) {
                        int j = geJob.massiveBodies[b];
                        rji = geJob.bodies.r[j] - geJob.bodies.r[i];
                        r2 = rji.x * rji.x + rji.y * rji.y + rji.z * rji.z;
                        r3 = r2 * System.Math.Sqrt(r2);
                        // G is already included in mu
                        geJob.leapfrogData.a[i] += geJob.bodies.mu[j] * rji / r3;
                        geJob.leapfrogData.a[j] -= geJob.bodies.mu[i] * rji / r3;
                    }
                }
                // add in any acceleration from Kepler & fixed bodies 
                for (int a = 0; a < geJob.lenMassiveBodies.Value; a++) {
                    i = geJob.massiveBodies[a];
                    for (int b = 0; b < geJob.lenKeplerMassive.Value; b++) {
                        int j = geJob.keplerMassive[b];
                        rji = geJob.bodies.r[j] - geJob.bodies.r[i];
                        r2 = rji.x * rji.x + rji.y * rji.y + rji.z * rji.z;
                        r3 = r2 * System.Math.Sqrt(r2);
                        geJob.leapfrogData.a[i] += geJob.bodies.mu[j] * rji / r3;
                    }
                    for (int b = 0; b < geJob.lenFixedBodies.Value; b++) {
                        int j = geJob.fixedBodies[b];
                        rji = geJob.bodies.r[j] - geJob.bodies.r[i];
                        r2 = rji.x * rji.x + rji.y * rji.y + rji.z * rji.z;
                        r3 = r2 * System.Math.Sqrt(r2);
                        geJob.leapfrogData.a[i] += geJob.bodies.mu[j] * rji / r3;
                    }
                }

            }

            /////////////////////////////////////////////////////////////
            /// NBody Massless
            /////////////////////////////////////////////////////////////

            // Update v and r (half-step for velocity, full step for position)
            if (geJob.lenMasslessBodies.Value > 0) {
                // first half-step and clear a
                for (int a = 0; a < geJob.lenMasslessBodies.Value; a++) {
                    i = geJob.masslessBodies[a];
                    geJob.bodies.v[i] += geJob.leapfrogData.a[i] * 0.5 * dt;
                    geJob.bodies.r[i] += geJob.bodies.v[i] * dt;
                    geJob.leapfrogData.a[i] = 0.0;
                }
                // calc a from all massive bodies. 
                for (int a = 0; a < geJob.lenMasslessBodies.Value; a++) {
                    i = geJob.masslessBodies[a];
                    for (int b = 0; b < geJob.lenMassiveBodies.Value; b++) {
                        int j = geJob.massiveBodies[b];
                        // O(N^2) in here, unpack loops to optimize	
                        rji = geJob.bodies.r[j] - geJob.bodies.r[i];
                        r2 = rji.x * rji.x + rji.y * rji.y + rji.z * rji.z;
                        r3 = r2 * System.Math.Sqrt(r2);
                        geJob.leapfrogData.a[i] += geJob.bodies.mu[j] * rji / r3;
                    }
                }
                // add in any acceleration from Kepler & fixed bodies 
                for (int a = 0; a < geJob.lenMasslessBodies.Value; a++) {
                    i = geJob.masslessBodies[a];
                    // Kepler massive source
                    for (int b = 0; b < geJob.lenKeplerMassive.Value; b++) {
                        int j = geJob.keplerMassive[b];
                        rji = geJob.bodies.r[j] - geJob.bodies.r[i];
                        r2 = rji.x * rji.x + rji.y * rji.y + rji.z * rji.z;
                        r3 = r2 * System.Math.Sqrt(r2);
                        geJob.leapfrogData.a[i] += geJob.bodies.mu[j] * rji / r3;
                    }
                    // fixed bodies
                    for (int b = 0; b < geJob.lenFixedBodies.Value; b++) {
                        int j = geJob.fixedBodies[b];
                        rji = geJob.bodies.r[j] - geJob.bodies.r[i];
                        r2 = rji.x * rji.x + rji.y * rji.y + rji.z * rji.z;
                        r3 = r2 * System.Math.Sqrt(r2);
                        geJob.leapfrogData.a[i] += geJob.bodies.mu[j] * rji / r3;
                    }
                }

            } // if masslexx n > 0

            // External Acceleration
            ExternalAcceleration(t, dt, ref geJob, ref geJob.leapfrogData.a);

            // update velocity
            for (int a = 0; a < geJob.lenMassiveBodies.Value; a++) {
                i = geJob.massiveBodies[a];
                geJob.bodies.v[i] += geJob.leapfrogData.a[i] * 0.5 * dt;
            }
            for (int a = 0; a < geJob.lenMasslessBodies.Value; a++) {
                i = geJob.masslessBodies[a];
                geJob.bodies.v[i] += geJob.leapfrogData.a[i] * 0.5 * dt;
            }

            return geJob.parms[GECore.T_PARAM];
        }

        // could do seperate massive and massless index lists to better optimize...
        private enum EAFilter { MASSIVE, MASSLESS, NONE };
        private static void ExternalAcceleration(double t,
                                                double dt,
                                                ref GEPhysicsCore.GEPhysicsJob geJob,
                                                ref NativeArray<double3> a,
                                                EAFilter filter = EAFilter.NONE)
        {
            // External forces
            int i;
            double3 a_in;
            ExternalAccel.EAStateData extData = new ExternalAccel.EAStateData();
            //foreach (int k in geJob.extAccelIndices) {
            for (int k = 0; k < geJob.lenExtAccelIndices.Value; k++) {
                int kk = geJob.extAccelIndices[k];
                ExternalAccel.EADesc eaDesc = geJob.extADesc[kk]; // copy
                i = geJob.extADesc[kk].bodyId;
                if (filter != EAFilter.NONE) {
                    if (filter == EAFilter.MASSIVE && geJob.bodies.propType[i] != GEPhysicsCore.Propagator.GRAVITY)
                        continue;
                    if (filter == EAFilter.MASSLESS && geJob.bodies.propType[i] != GEPhysicsCore.Propagator.GRAVITY)
                        continue;
                }
                a_in = a[i];
                double3 a_out;
                int status;
                switch (geJob.extADesc[k].type) {
                    case ExternalAccel.ExtAccelType.SELF:
                        extData.Init(ref geJob.bodies, i);
                        status = ExternalAccel.ExtAccelFactory(ref eaDesc, ref extData, t, dt, a_in, geJob.extAData, out a_out, ref geJob);
                        // rocket staging currently only force interaction...so skip in ON_OTHER
                        if (status != 0) {
                            ReportStatus(status, ref geJob, i, t);
                        }
                        // UnityEngine.Debug.LogFormat("a_out ={0} v_f={1} a_grav={2} pitch={3}",
                        //    a_out, extData.v_from, a[i], geJob.extAData[0].y);
                        a[i] += a_out;
                        break;

                    case ExternalAccel.ExtAccelType.ON_OTHER:
                        // apply accel to every other massive and massless body that is not index i
                        for (int b = 0; b < geJob.lenMassiveBodies.Value; b++) {
                            int j = geJob.massiveBodies[b];
                            a_in = a[j];
                            if (j != i) {
                                extData.Init(ref geJob.bodies, i, j);
                                status = ExternalAccel.ExtAccelFactory(ref eaDesc, ref extData, t, dt, a_in, geJob.extAData, out a_out, ref geJob);
                                if (status != 0) {
                                    ReportStatus(status, ref geJob, i, t);
                                }
                                a[j] += a_out;
                            }
                        }
                        for (int b = 0; b < geJob.lenMasslessBodies.Value; b++) {
                            int j = geJob.masslessBodies[b];
                            a_in = a[j];
                            if (j != i) {
                                extData.Init(ref geJob.bodies, i, j);
                                status = ExternalAccel.ExtAccelFactory(ref eaDesc, ref extData, t, dt, a_in, geJob.extAData, out a_out, ref geJob);
                                if (status != 0) {
                                    ReportStatus(status, ref geJob, i, t);
                                }
                                a[j] += a_out;
                            }
                        }
                        break;

                    case ExternalAccel.ExtAccelType.SELF_INTEGRATED:
                        continue;

                    case ExternalAccel.ExtAccelType.INACTIVE:
                        continue;

                    default:
                        throw new System.NotImplementedException(
                            string.Format("Unknown External Acceleration type: {0}", geJob.extADesc[k].type));
                }
            }
        }

        private static void ReportStatus(int status, ref GEPhysicsCore.GEPhysicsJob geJob, int index, double t)
        {
            GEPhysicsCore.PhysEvent eaEvent = new GEPhysicsCore.PhysEvent {
                bodyId = index,
                statusCode = status,
                type = GEPhysicsCore.EventType.BOOSTER,
                r = geJob.bodies.r[index],
                t = t
            };
            geJob.gepcEvents.Add(eaEvent);
        }

        //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        // RK4
        //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        public struct RK4Data {
            public NativeArray<double3> r_k0;
            public NativeArray<double3> r_k1;
            public NativeArray<double3> r_k2;
            public NativeArray<double3> r_k3;
            public NativeArray<double3> r_tmp;

            public NativeArray<double3> v_k0;
            public NativeArray<double3> v_k1;
            public NativeArray<double3> v_k2;
            public NativeArray<double3> v_k3;
            public NativeArray<double3> v_tmp;

            public void Init(int size, Allocator allocator = Allocator.Persistent)
            {
                r_k0 = new NativeArray<double3>(size, allocator);
                r_k1 = new NativeArray<double3>(size, allocator);
                r_k2 = new NativeArray<double3>(size, allocator);
                r_k3 = new NativeArray<double3>(size, allocator);
                r_tmp = new NativeArray<double3>(size, allocator);

                v_k0 = new NativeArray<double3>(size, allocator);
                v_k1 = new NativeArray<double3>(size, allocator);
                v_k2 = new NativeArray<double3>(size, allocator);
                v_k3 = new NativeArray<double3>(size, allocator);
                v_tmp = new NativeArray<double3>(size, allocator);
            }

            public void InitFrom(RK4Data from)
            {
                int size = from.r_k0.Length;
                Init(size);
                r_k0.CopyFrom(from.r_k0);
                r_k1.CopyFrom(from.r_k1);
                r_k2.CopyFrom(from.r_k2);
                r_k2.CopyFrom(from.r_k3);
                r_tmp.CopyFrom(from.r_tmp);
                v_k0.CopyFrom(from.v_k0);
                v_k1.CopyFrom(from.v_k1);
                v_k2.CopyFrom(from.v_k2);
                v_k3.CopyFrom(from.v_k2);
                v_tmp.CopyFrom(from.v_tmp);
            }

            public void GrowBy(int growBy)
            {
                NativeArray<double3> oldr_k0 = r_k0;
                NativeArray<double3> oldr_k1 = r_k1;
                NativeArray<double3> oldr_k2 = r_k2;
                NativeArray<double3> oldr_k3 = r_k3;
                NativeArray<double3> oldr_tmp = r_tmp;

                NativeArray<double3> oldv_k0 = v_k0;
                NativeArray<double3> oldv_k1 = v_k1;
                NativeArray<double3> oldv_k2 = v_k2;
                NativeArray<double3> oldv_k3 = v_k3;
                NativeArray<double3> oldv_tmp = v_tmp;
                int n = oldr_k0.Length;
                NativeArray<double3>.Copy(oldr_k0, r_k0, n);
                NativeArray<double3>.Copy(oldr_k1, r_k1, n);
                NativeArray<double3>.Copy(oldr_k2, r_k2, n);
                NativeArray<double3>.Copy(oldr_k3, r_k3, n);
                NativeArray<double3>.Copy(oldr_tmp, r_tmp, n);

                NativeArray<double3>.Copy(oldv_k0, v_k0, n);
                NativeArray<double3>.Copy(oldv_k1, v_k1, n);
                NativeArray<double3>.Copy(oldv_k2, v_k2, n);
                NativeArray<double3>.Copy(oldv_k3, v_k3, n);
                NativeArray<double3>.Copy(oldv_tmp, v_tmp, n);

                int size = r_k0.Length + growBy;
                Allocator allocator = Allocator.Persistent;
                r_k0 = new NativeArray<double3>(size, allocator);
                r_k1 = new NativeArray<double3>(size, allocator);
                r_k2 = new NativeArray<double3>(size, allocator);
                r_k3 = new NativeArray<double3>(size, allocator);
                r_tmp = new NativeArray<double3>(size, allocator);

                v_k0 = new NativeArray<double3>(size, allocator);
                v_k1 = new NativeArray<double3>(size, allocator);
                v_k2 = new NativeArray<double3>(size, allocator);
                v_k3 = new NativeArray<double3>(size, allocator);
                v_tmp = new NativeArray<double3>(size, allocator);

                oldr_k0.Dispose();
                oldr_k1.Dispose();
                oldr_k2.Dispose();
                oldr_k3.Dispose();
                oldr_tmp.Dispose();

                oldv_k0.Dispose();
                oldv_k1.Dispose();
                oldv_k2.Dispose();
                oldv_k3.Dispose();
                oldv_tmp.Dispose();
            }

            public double3 CalcA(int i, double dt)
            {
                return dt / 6.0 * (v_k0[i] + 2.0 * v_k1[i] + 2.0 * v_k2[i] + v_k3[i]);
            }

            public void Dispose()
            {
                r_k0.Dispose();
                r_k1.Dispose();
                r_k2.Dispose();
                r_k3.Dispose();
                r_tmp.Dispose();

                v_k0.Dispose();
                v_k1.Dispose();
                v_k2.Dispose();
                v_k3.Dispose();
                v_tmp.Dispose();
            }
        }

        public static double RK4Step(ref GEPhysicsCore.GEPhysicsJob geJob)
        {
            double dt = geJob.parms[GECore.DT_PARAM];
            double t = geJob.parms[GECore.T_PARAM];
            geJob.parms[GECore.T_PARAM] += dt;
            double dtHalf = 0.5 * dt;
            int i;

            // Need to fill r_tmp with positions of Kepler massive bodies
            for (int b = 0; b < geJob.lenKeplerMassive.Value; b++) {
                int j = geJob.keplerMassive[b];
                geJob.rk4Data.r_tmp[j] = geJob.bodies.r[j];
            }
            // fill in with fixed bodies
            for (int b = 0; b < geJob.lenFixedBodies.Value; b++) {
                int j = geJob.fixedBodies[b];
                geJob.rk4Data.r_tmp[j] = geJob.bodies.r[j];
            }

            if (geJob.lenMassiveBodies.Value > 0) {
                // Massive
                EvaluateMassiveRK4(t, dt, ref geJob, ref geJob.bodies.r, ref geJob.bodies.v, ref geJob.rk4Data.r_k0, ref geJob.rk4Data.v_k0);
                for (int a = 0; a < geJob.lenMassiveBodies.Value; a++) {
                    i = geJob.massiveBodies[a];
                    geJob.rk4Data.r_tmp[i] = geJob.bodies.r[i] + dtHalf * geJob.rk4Data.r_k0[i];
                    geJob.rk4Data.v_tmp[i] = geJob.bodies.v[i] + dtHalf * geJob.rk4Data.v_k0[i];
                }
                EvaluateMassiveRK4(t + dtHalf, dt, ref geJob, ref geJob.rk4Data.r_tmp, ref geJob.rk4Data.v_tmp, ref geJob.rk4Data.r_k1, ref geJob.rk4Data.v_k1);
                for (int a = 0; a < geJob.lenMassiveBodies.Value; a++) {
                    i = geJob.massiveBodies[a];
                    geJob.rk4Data.r_tmp[i] = geJob.bodies.r[i] + dtHalf * geJob.rk4Data.r_k1[i];
                    geJob.rk4Data.v_tmp[i] = geJob.bodies.v[i] + dtHalf * geJob.rk4Data.v_k1[i];
                }
                EvaluateMassiveRK4(t + dtHalf, dt, ref geJob, ref geJob.rk4Data.r_tmp, ref geJob.rk4Data.v_tmp, ref geJob.rk4Data.r_k2, ref geJob.rk4Data.v_k2);
                for (int a = 0; a < geJob.lenMassiveBodies.Value; a++) {
                    i = geJob.massiveBodies[a];
                    geJob.rk4Data.r_tmp[i] = geJob.bodies.r[i] + dt * geJob.rk4Data.r_k2[i];
                    geJob.rk4Data.v_tmp[i] = geJob.bodies.v[i] + dt * geJob.rk4Data.v_k2[i];
                }
                EvaluateMassiveRK4(t + dt, dt, ref geJob, ref geJob.rk4Data.r_tmp, ref geJob.rk4Data.v_tmp, ref geJob.rk4Data.r_k3, ref geJob.rk4Data.v_k3);
                for (int a = 0; a < geJob.lenMassiveBodies.Value; a++) {
                    i = geJob.massiveBodies[a];
                    geJob.bodies.r[i] += dt / 6.0 * (geJob.rk4Data.r_k0[i] + 2.0 * geJob.rk4Data.r_k1[i] + 2.0 * geJob.rk4Data.r_k2[i] + geJob.rk4Data.r_k3[i]);
                    geJob.bodies.v[i] += dt / 6.0 * (geJob.rk4Data.v_k0[i] + 2.0 * geJob.rk4Data.v_k1[i] + 2.0 * geJob.rk4Data.v_k2[i] + geJob.rk4Data.v_k3[i]);
                }

            }

            // Massless
            if (geJob.lenMasslessBodies.Value > 0) {
                EvaluateMasslessRK4(t, dt, ref geJob, ref geJob.bodies.r, ref geJob.bodies.v, ref geJob.rk4Data.r_k0, ref geJob.rk4Data.v_k0);
                for (int a = 0; a < geJob.lenMasslessBodies.Value; a++) {
                    i = geJob.masslessBodies[a];
                    geJob.rk4Data.r_tmp[i] = geJob.bodies.r[i] + dtHalf * geJob.rk4Data.r_k0[i];
                    geJob.rk4Data.v_tmp[i] = geJob.bodies.v[i] + dtHalf * geJob.rk4Data.v_k0[i];
                }
                EvaluateMasslessRK4(t + dtHalf, dt, ref geJob, ref geJob.rk4Data.r_tmp, ref geJob.rk4Data.v_tmp, ref geJob.rk4Data.r_k1, ref geJob.rk4Data.v_k1);
                for (int a = 0; a < geJob.lenMasslessBodies.Value; a++) {
                    i = geJob.masslessBodies[a];
                    geJob.rk4Data.r_tmp[i] = geJob.bodies.r[i] + dtHalf * geJob.rk4Data.r_k1[i];
                    geJob.rk4Data.v_tmp[i] = geJob.bodies.v[i] + dtHalf * geJob.rk4Data.v_k1[i];
                }
                EvaluateMasslessRK4(t + dtHalf, dt, ref geJob, ref geJob.rk4Data.r_tmp, ref geJob.rk4Data.v_tmp, ref geJob.rk4Data.r_k2, ref geJob.rk4Data.v_k2);
                for (int a = 0; a < geJob.lenMasslessBodies.Value; a++) {
                    i = geJob.masslessBodies[a];
                    geJob.rk4Data.r_tmp[i] = geJob.bodies.r[i] + dt * geJob.rk4Data.r_k2[i];
                    geJob.rk4Data.v_tmp[i] = geJob.bodies.v[i] + dt * geJob.rk4Data.v_k2[i];
                }
                EvaluateMasslessRK4(t + dt, dt, ref geJob, ref geJob.rk4Data.r_tmp, ref geJob.rk4Data.v_tmp, ref geJob.rk4Data.r_k3, ref geJob.rk4Data.v_k3);
                for (int a = 0; a < geJob.lenMasslessBodies.Value; a++) {
                    i = geJob.masslessBodies[a];
                    geJob.bodies.r[i] += dt / 6.0 * (geJob.rk4Data.r_k0[i] + 2.0 * geJob.rk4Data.r_k1[i] + 2.0 * geJob.rk4Data.r_k2[i] + geJob.rk4Data.r_k3[i]);
                    geJob.bodies.v[i] += dt / 6.0 * (geJob.rk4Data.v_k0[i] + 2.0 * geJob.rk4Data.v_k1[i] + 2.0 * geJob.rk4Data.v_k2[i] + geJob.rk4Data.v_k3[i]);
                }
            }
            return geJob.parms[GECore.T_PARAM];
        }

        private static void EvaluateMassiveRK4(double t, double dt,
                                        ref GEPhysicsCore.GEPhysicsJob geJob,
                                        ref NativeArray<double3> r,
                                        ref NativeArray<double3> v,
                                        ref NativeArray<double3> dr_out,
                                        ref NativeArray<double3> dv_out)
        {
            double3 rji;
            double r2, r3;
            int n = geJob.lenMassiveBodies.Value;

            // a = 0 or init with eternal value
            for (int a = 0; a < n; a++) {
                int i = geJob.massiveBodies[a];
                dv_out[i] = 0.0;
            }
            for (int a = 0; a < n; a++) {
                int i = geJob.massiveBodies[a];
                dr_out[i] = v[i];
                for (int b = a + 1; b < n; b++) {
                    int j = geJob.massiveBodies[b];
                    // O(N^2) in here, unpack loops to optimize	
                    rji = r[j] - r[i];
                    r2 = rji.x * rji.x + rji.y * rji.y + rji.z * rji.z;
                    r3 = r2 * System.Math.Sqrt(r2);
                    dv_out[i] += geJob.bodies.mu[j] * rji / r3;
                    dv_out[j] -= geJob.bodies.mu[i] * rji / r3;
                }
            }
            // add in any acceleration from Kepler geJob.bodies
            for (int a = 0; a < geJob.lenMassiveBodies.Value; a++) {
                int i = geJob.massiveBodies[a];
                // Kepler massive sources
                for (int b = 0; b < geJob.lenKeplerMassive.Value; b++) {
                    int j = geJob.keplerMassive[b];
                    rji = r[j] - r[i];
                    r2 = rji.x * rji.x + rji.y * rji.y + rji.z * rji.z;
                    r3 = r2 * System.Math.Sqrt(r2);
                    dv_out[i] += geJob.bodies.mu[j] * rji / r3;
                }
                // fixed bodies
                for (int b = 0; b < geJob.lenFixedBodies.Value; b++) {
                    int j = geJob.fixedBodies[b];
                    rji = r[j] - r[i];
                    r2 = rji.x * rji.x + rji.y * rji.y + rji.z * rji.z;
                    r3 = r2 * System.Math.Sqrt(r2);
                    dv_out[i] += geJob.bodies.mu[j] * rji / r3;
                }
            }
            // For a rocket eqn will get called four time per dt, so want to decrement fuel based on that
            // (this is not exactly correct)
            ExternalAcceleration(t, 0.25 * dt, ref geJob, ref dv_out, EAFilter.MASSIVE);
        }

        private static void EvaluateMasslessRK4(double t, double dt,
                                        ref GEPhysicsCore.GEPhysicsJob geJob,
                                        ref NativeArray<double3> r,
                                        ref NativeArray<double3> v,
                                        ref NativeArray<double3> dr_out,
                                        ref NativeArray<double3> dv_out)
        {
            double3 rji;
            double r2, r3;
            // a = 0 or init with eternal value
            for (int a = 0; a < geJob.lenMasslessBodies.Value; a++) {
                int i = geJob.masslessBodies[a];
                dv_out[i] = 0.0;
            }

            for (int a = 0; a < geJob.lenMasslessBodies.Value; a++) {
                int i = geJob.masslessBodies[a];
                dr_out[i] = v[i];
                for (int b = 0; b < geJob.lenMassiveBodies.Value; b++) {
                    int j = geJob.massiveBodies[b];
                    rji = r[j] - r[i];
                    r2 = rji.x * rji.x + rji.y * rji.y + rji.z * rji.z;
                    r3 = r2 * System.Math.Sqrt(r2);
                    dv_out[i] += geJob.bodies.mu[j] * rji / r3;
                }
            }
            // add in any acceleration from Kepler geJob.bodies
            for (int a = 0; a < geJob.lenMasslessBodies.Value; a++) {
                int i = geJob.masslessBodies[a];
                // Kepler massive sources
                for (int b = 0; b < geJob.lenKeplerMassive.Value; b++) {
                    int j = geJob.keplerMassive[b];
                    rji = r[j] - r[i];
                    r2 = rji.x * rji.x + rji.y * rji.y + rji.z * rji.z;
                    r3 = r2 * System.Math.Sqrt(r2);
                    dv_out[i] += geJob.bodies.mu[j] * rji / r3;
                }
                // Fixed bodies
                for (int b = 0; b < geJob.lenFixedBodies.Value; b++) {
                    int j = geJob.fixedBodies[b];
                    rji = r[j] - r[i];
                    r2 = rji.x * rji.x + rji.y * rji.y + rji.z * rji.z;
                    r3 = r2 * System.Math.Sqrt(r2);
                    dv_out[i] += geJob.bodies.mu[j] * rji / r3;
                }
            }
            ExternalAcceleration(t, 0.25 * dt, ref geJob, ref dv_out, EAFilter.MASSLESS);
        }

    }

}
