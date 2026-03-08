using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;

namespace GravityEngine2 {
    public struct LambertPorkchop : IJob {

        // a bit much for a constructor so do struct init in the caller.
        public double3 r1;
        public double3 v1;
        public double3 r2;
        public double3 v2;

        public double mu;

        public double tXfer_min;
        public double tXfer_max;
        public double tDepart_start;
        public double tDepart_end;

        public int tDepart_steps;

        public double tArrive_start;
        public double tArrive_end;

        public int tArrive_steps;

        private double tDepart_step;
        private double tArrive_step;

        public double radius;

        public NativeArray<Lambert.LambertOutput> outputs;

        public void Init()
        {
            // determine how many transfer calculations will be needed
            tDepart_step = (tDepart_end - tDepart_start) / (tDepart_steps - 1);
            tArrive_step = (tArrive_end - tArrive_start) / (tArrive_steps - 1);
            double tDepart;
            double tArrive;
            double tXfer;
            int n = 0;
            for (int i = 0; i < tDepart_steps; i++) {
                tDepart = tDepart_start + i * tDepart_step;
                for (int j = 0; j < tArrive_steps; j++) {
                    tArrive = tArrive_start + j * tArrive_step;
                    tXfer = tArrive - tDepart;
                    if (tXfer >= tXfer_min && tXfer <= tXfer_max)
                        n++;
                }
            }
            outputs = new NativeArray<Lambert.LambertOutput>(n, Allocator.Persistent);
        }


        public void Execute()
        {
            KeplerPropagator.RVT rvt1 = new KeplerPropagator.RVT(r1, v1, 0.0, mu);
            KeplerPropagator.RVT rvt2 = new KeplerPropagator.RVT(r2, v2, 0.0, mu);

            double tDepart;
            double tArrive;
            double tXfer;
            int n = 0;
            for (int i = 0; i < tDepart_steps; i++) {
                tDepart = tDepart_start + i * tDepart_step;
                (int status, double3 r1x, double3 v1x) = KeplerPropagator.RVforTime(rvt1, tDepart);
                if (status != 0) {
                    Debug.LogErrorFormat("Error getting target state at time {0}", tDepart);
                    continue;
                }
                for (int j = 0; j < tArrive_steps; j++) {
                    tArrive = tArrive_start + j * tArrive_step;
                    tXfer = tArrive - tDepart;
                    if (tXfer >= tXfer_min && tXfer <= tXfer_max) {
                        // find target state at arrival time
                        (int status2, double3 r2x, double3 v2x) = KeplerPropagator.RVforTime(rvt2, tArrive);
                        if (status2 != 0) {
                            Debug.LogErrorFormat("Error getting target state at time {0}", tXfer);
                            continue;
                        }
                        Lambert.LambertOutput lamOutput = Lambert.TransferProgradeToPoint(mu, r1x, r2x, v1x, tXfer, radius);
                        // r1, r2 and v1 are set in the Transfer method
                        lamOutput.v2 = v2x;
                        lamOutput.t0 = tDepart;
                        if (lamOutput.status != Lambert.Status.OK) {
                            Debug.LogErrorFormat("Error computing Lambert transfer err={0}", lamOutput.status);
                            lamOutput.t = double.NaN;
                            continue;
                        }

                        outputs[n] = lamOutput;
                        n++;
                    }
                }
            }
        }

        public void Dispose()
        {
            outputs.Dispose();
        }
    }
}
