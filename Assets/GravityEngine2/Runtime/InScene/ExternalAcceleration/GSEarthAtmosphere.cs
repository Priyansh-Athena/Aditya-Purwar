using UnityEngine;
using Unity.Mathematics;

namespace GravityEngine2 {
    /// <summary>
    /// Computes the acceleration due to drag in the Earth's atmosphere for a given height and
    /// velocity. 
    /// 
    /// </summary>
    public class GSEarthAtmosphere : MonoBehaviour, GSExternalAcceleration {


        [SerializeField]
        [Tooltip("Height of the Earth's surface (km)")]
        public double heightSurfaceKm = 6371;

        // Cannot add { get; set; } to attribute with a tooltip - so public. (Ick)
        [SerializeField]
        [Tooltip("Inertial mass of the spaceship in kg")]
        public double inertialMassKg = 100;

        [SerializeField]
        [Tooltip("Cross sectional area in m^2")]
        public double crossSectionalArea;

        [SerializeField]
        [Tooltip("Drag Co-efficient. 2.0-2.1 for a sphere. 2.2 is a typical value")]
        public double coeefDrag = 2.1;

        private static double[] densityTablePer10km;

        /// <summary>
        /// Validation:
        /// - confirmed velocity in SI units matches expected orbital velocity of ISS
        /// - checked that SI acceleration = -g at terminal velocity (9.73 m/s^2 at 28 km)
        /// </summary>

        public int AddToGE(int id, GECore ge, GBUnits.Units units)
        {
            uint massLU = 0;
            double3[] data = EarthAtmosphere.Alloc(heightSurfaceKm, coeefDrag, crossSectionalArea, inertialMassKg, massLU);
            return ge.ExternalAccelerationAdd(id,
                                        ExternalAccel.ExtAccelType.SELF,
                                        ExternalAccel.AccelType.EARTH_ATMOSPHERE,
                                        data);
        }

    }
}
