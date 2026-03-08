
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace GravityEngine2 {
    public struct LambertJob : IJob {
        private double mu;
        private double3 r1;
        private double3 r2;
        private double3 v1;
        private double3 v2;
        private double radius;

        public NativeArray<double> values;
        public NativeArray<Lambert.LambertOutput> outputs;

        public enum LambertType { TO_POINT_BYTIME, TO_POINT_BYANGLE, TO_TARGET_BYTIME };

        public LambertType xferType;

        public LambertJob(int n, double mu, double3 r1, double3 r2, double3 v1, double3 v2, double radius, LambertType type)
        {
            this.mu = mu;
            this.r1 = r1;
            this.r2 = r2;
            this.v1 = v1;
            this.v2 = v2;
            this.radius = radius;
            values = new NativeArray<double>(n, Allocator.Persistent);
            outputs = new NativeArray<Lambert.LambertOutput>(n, Allocator.Persistent);
            xferType = type;
        }

        public void Dispose()
        {
            values.Dispose();
            outputs.Dispose();
        }

        public void Execute()
        {
            // TODO: avoid creation of LambertOutput inside the Lambert code. 
            switch (xferType) {
                case LambertType.TO_POINT_BYTIME:
                    for (int i = 0; i < values.Length; i++) {
                        outputs[i] = Lambert.TransferProgradeToPoint(mu, r1, r2, v1, values[i], radius);
                    }
                    break;
                case LambertType.TO_POINT_BYANGLE:
                    for (int i = 0; i < values.Length; i++) {
                        outputs[i] = Lambert.TransferProgradeToPointWithFPA(mu, r1, r2, v1, values[i], radius);
                    }
                    break;
                case LambertType.TO_TARGET_BYTIME:
                    // Evolve the target to the position it will occupy at the time of arrival
                    bool prograde = true;
                    KeplerPropagator.RVT rvt2 = new KeplerPropagator.RVT(r2, v2, 0.0, mu);
                    for (int i = 0; i < values.Length; i++) {
                        (int s, double3 r2, double3 v2) = KeplerPropagator.RVforTime(rvt2, values[i]);
                        double angle = GravityMath.AngleRadiansPrograde(r1, v1, r2);
                        Lambert.MotionDirection motionDir = Lambert.MotionDirection.SHORT;
                        if (angle > math.PI) {
                            motionDir = Lambert.MotionDirection.LONG;
                        }
                        if (!prograde) {
                            motionDir = (motionDir == Lambert.MotionDirection.SHORT) ? Lambert.MotionDirection.LONG : Lambert.MotionDirection.SHORT;
                        }
                        var output = Lambert.Universal(mu, r1, r2, v1, motionDir, Lambert.EnergyType.HIGH, nrev: 0, values[i], 0.0, radius);
                        output.v2 = v2;
                        outputs[i] = output;
                    }
                    break;
                default:
                    throw new System.NotImplementedException();
            }
        }
    }
}
