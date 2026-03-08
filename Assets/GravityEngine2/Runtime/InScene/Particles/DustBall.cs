using Unity.Mathematics;
using UnityEngine;
using Unity.Collections;

namespace GravityEngine2 {
    public class DustBall : MonoBehaviour, IGSParticlesInit {
        public GBUnits.Units units = GBUnits.Units.DL;

        [Header("Pos/Vel in World Units and RH Coords")]
        public Vector3 position;
        //! Velocity of each particle when it is initalized. 
        public Vector3 velocity;

        //! Radius of the ball of particles.
        public float radius = 1f;


        public void InitNewParticles(int numLastActive,
                                     int numActive,
                                     GBUnits.GEScaler geScaler,
                                     ref NativeArray<double3> r,
                                     ref NativeArray<double3> v)
        {
            float scale = 1.0f;
            double3 vel3 = GravityMath.Vector3ToDouble3(velocity);
            if (units != geScaler.WorldUnits()) {
                scale *= (float)GBUnits.DistanceConversion(units, geScaler.WorldUnits());
                vel3 *= (float)GBUnits.VelocityConversion(units, geScaler.WorldUnits());
            }
            scale *= (float)geScaler.ScaleLenWorldToGE(1.0);
            vel3 *= (float)geScaler.ScaleVelocityWorldToGE(1.0);
            Debug.LogFormat("Sclaing V: {0} then {1}",
                GBUnits.VelocityConversion(units, geScaler.WorldUnits()),
                geScaler.ScaleVelocityWorldToGE(1.0)
                );
            for (int i = numLastActive; i < numActive; i++) {
                Vector3 pos = (position + radius * UnityEngine.Random.insideUnitSphere) * scale;
                r[i] = new double3(pos.x, pos.y, pos.z);
                v[i] = vel3;
                if (i == 0)
                    Debug.LogFormat("p={0} v={1}", r[i], v[i]);
            }
        }

        void OnDrawGizmosSelected()
        {
            Gizmos.DrawSphere(transform.position, radius);
        }
    }
}
