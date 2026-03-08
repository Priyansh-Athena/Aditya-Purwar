using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using Random = UnityEngine.Random;

namespace GravityEngine2 {


    /// <summary>
    /// Dust ring.
    /// Create a ring of particles in orbit around an NBody mass. Allows full control over the orbital attributes
    /// of the ring particles. 
    ///
    /// Must be attached to a particle system with a GravityParticles component.
    /// 
    /// </summary>
    //[Obsolete("Please use DustOrbit instead")]
    public class DustRing : MonoBehaviour, IGSParticlesInit {
        public GBUnits.Units units = GBUnits.Units.DL;

        public BodyInitData bodyInitData = new BodyInitData();

        public GSDisplayBody centerDisplayBody;

        //
        // Create a ring of particles in orbit around a specific GameObject with an attached NBody script
        // Must be called once the position and velocity of the NBody has been initialized

        //! Width of particle ring as a percent of ring radius. 
        public float ringWidthPercent;

        private double mass;

        private Orbital.COE coe;
        private bool coeInited;

        // Use this for initialization
        void Start()
        {
            GSBody centerBody = centerDisplayBody.gsBody;
            mass = centerBody.mass; // Does this need to be scaled??
        }

        public void InitNewParticles(int numLastActive,
                                     int numActive,
                                     GBUnits.GEScaler geScaler,
                                     ref NativeArray<double3> r,
                                     ref NativeArray<double3> v)
        {
            if (units != geScaler.WorldUnits()) {
                coe.a *= (float)GBUnits.DistanceConversion(units, geScaler.WorldUnits());
                coe.p *= (float)GBUnits.VelocityConversion(units, geScaler.WorldUnits());
            }
            if (!coeInited) {
                coe = new Orbital.COE();
                bodyInitData.FillInCOE(coe);
                coe.a *= geScaler.ScaleLenWorldToGE(1.0);
                coe.p *= geScaler.ScaleVelocityWorldToGE(1.0);
                coe.mu = geScaler.ScaleMassWorldToGE(mass);
                coeInited = true;
            }
            for (int i = numLastActive; i < numActive; i++) {
                float E = Random.Range(0, 2 * Mathf.PI);
                double a = coe.a;
                // Use SetA to ensure p is also updated (that's what COE to RV will use!)
                coe.SetA(a + a * Random.Range(-ringWidthPercent / 100.0f, ringWidthPercent / 100.0f));
                // distribute at random E so get even spread around orbit
                (r[i], v[i]) = Orbital.COEWithPhasetoRVRelative(coe, Orbital.ConvertEtoTrueAnomoly(E, coe.e));
                coe.a = a;
            }
        }

    }

}
