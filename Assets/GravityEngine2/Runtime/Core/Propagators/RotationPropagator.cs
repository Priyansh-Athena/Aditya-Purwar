
using Unity.Mathematics;
using Unity.Collections;

namespace GravityEngine2 {
    public class RotationPropagator {
        private static double3 z_axis = new double3(0.0, 0.0, 1.0);

        /// <summary>
        /// Information about the rotation of a planet.
        /// May be in world or GE internal units depending on context.
        /// </summary>
        public struct PropInfo {

            public int centerId;
            public double3 axis;
            public double rate;
            public double phi0;

            public double theta0; // radians
            public double radius;   // radius of planet plus altitude

            public quaternionD axisRot;

            public PropInfo(int center_id,
                            double3 axis,
                            double rate,
                            double phi0Radians,
                            double latitudeDeg,
                            double longitudeDeg,
                            double radius)
            {
                centerId = center_id;
                this.axis = math.normalize(axis);
                this.rate = rate;
                phi0 = phi0Radians + math.radians(longitudeDeg);
                this.radius = radius;
                theta0 = math.radians(90.0 - latitudeDeg);
                axisRot = quaternionD.Normalize(quaternionD.FromToRotation(z_axis, axis));
            }
        }


        public static void EvolveAll(double t_to,
                                ref GEPhysicsCore.GEBodies bodies,
                                ref NativeArray<PropInfo> rotPropInfo,
                                ref NativeArray<int> indices,
                                int lenIndices)
        {
            for (int ii = 0; ii < lenIndices; ii++) {
                int i = indices[ii];
                if (bodies.self_integrating[i]) {
                    continue;
                }
                int pIndex = bodies.propIndex[i];
                (double3 r, double3 v) = Rotate(t_to, pIndex, ref rotPropInfo);
                bodies.r[i] = r;
                bodies.v[i] = v;
                bodies.r[i] += bodies.r[rotPropInfo[pIndex].centerId];
                bodies.v[i] += bodies.v[rotPropInfo[pIndex].centerId];
            }
        }

        public static void EvolveRelative(double t, int propId, ref NativeArray<PropInfo> rotPropInfo, ref GEBodyState state)
        {
            (double3 r, double3 v) = Rotate(t, propId, ref rotPropInfo);
            state.r = r;
            state.v = v;
        }

        private static (double3 r, double3 v) Rotate(double t, int propId, ref NativeArray<PropInfo> rotPropInfo)
        {
            double3 r2, v2;
            double phiRad = rotPropInfo[propId].phi0 + t * rotPropInfo[propId].rate;
            double radius = rotPropInfo[propId].radius;
            double thetaRad = rotPropInfo[propId].theta0;
            double3 r = new double3(radius * math.cos(phiRad) * math.sin(thetaRad),
                                radius * math.sin(phiRad) * math.sin(thetaRad),
                                radius * math.cos(thetaRad));
            double v_mag = radius * math.sin(thetaRad) * rotPropInfo[propId].rate;
            double3 v = math.normalize(math.cross(z_axis, r)) * v_mag;
            r2 = quaternionD.mul(rotPropInfo[propId].axisRot, r);
            v2 = quaternionD.mul(rotPropInfo[propId].axisRot, v);
            return (r2, v2);
        }
    }
}
