using Unity.Collections;
using Unity.Mathematics;

namespace GravityEngine2 {
    /// <summary>
    /// Interface defining methods to be implemented to define particle positions and velocities for GravityParticles.
    /// 
    /// </summary>
    public interface IGSParticlesInit {

        /// <summary>
        /// Provide the initial positions and velocity for a range of particles. This method will be called
        /// as particles are created by the particle system.  The implementing class must fill in the r[] and
        /// v[] arrays for the range specified. These arrays are indexed by [particle_num, dimension] where
        /// dimension 0,1,2 correspond to x,y,z.
        ///
        /// See the DustBox script for a sample usage of this interface.
        /// </summary>
        /// <param name="fromParticle"></param>
        /// <param name="toParticle"></param>
        /// <param name="scaleWorldToGE"></param>
        /// <param name="r"></param>
        /// <param name="v"></param>
        void InitNewParticles(int fromParticle,
                              int toParticle,
                              GBUnits.GEScaler geScaler,
                              ref NativeArray<double3> r,
                              ref NativeArray<double3> v);
    }

}
