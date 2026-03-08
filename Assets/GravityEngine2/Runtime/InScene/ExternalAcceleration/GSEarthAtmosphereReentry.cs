using UnityEngine;
using Unity.Mathematics;

namespace GravityEngine2
{
    /// <summary>
    /// Unity component to add EarthAtmosphereReentry external acceleration to a GECore body.
    /// 
    /// This component demonstrates how to integrate EarthAtmosphereReentry into the Unity scene system.
    /// It follows the same pattern as GSEarthAtmosphere but uses the reentry-specific physics model.
    /// 
    /// The reentry model includes:
    /// - Atmospheric drag based on exponential density model
    /// - Self-integrated evolution (handles its own position/velocity updates)
    /// - Configurable spacecraft parameters (mass, drag coefficient, cross-sectional area)
    /// - Time step control for accurate reentry simulation
    /// </summary>
    public class GSEarthAtmosphereReentry : MonoBehaviour, GSExternalAcceleration
    {

        [Header("Earth Parameters")]
        [SerializeField]
        [Tooltip("Height of the Earth's surface (km)")]
        public double heightSurfaceKm = 6371;

        [Header("Spacecraft Parameters")]
        [SerializeField]
        [Tooltip("Inertial mass of the spacecraft in kg")]
        public double inertialMassKg = 1000;

        [SerializeField]
        [Tooltip("Cross sectional area in m^2")]
        public double crossSectionalArea = 10.0;

        [SerializeField]
        [Tooltip("Drag coefficient. 2.0-2.1 for a sphere. 2.2 is a typical value for spacecraft")]
        public double coeffDrag = 2.2;

        [Header("Simulation Parameters")]
        [SerializeField]
        [Tooltip("Time step for reentry simulation in seconds. Smaller values provide more accuracy but slower performance")]
        public double timeStepSec = 0.1;

        [SerializeField]
        [Tooltip("Earth's gravitational parameter in SI units (m^3/s^2)")]
        public double earthMuSI = 3.986e14;

        /// <summary>
        /// Add this EarthAtmosphereReentry external acceleration to a GECore body.
        /// 
        /// This method creates the necessary data structure and adds it to the GECore
        /// as a self-integrated external acceleration.
        /// </summary>
        /// <param name="id">The GECore body ID to add the acceleration to</param>
        /// <param name="ge">The GECore instance</param>
        /// <param name="units">The units system being used</param>
        /// <returns>The external acceleration ID, or -1 if failed</returns>
        public int AddToGE(int id, GECore ge, GBUnits.Units units)
        {
            // Create the data structure for EarthAtmosphereReentry
            double3[] data = EarthAtmosphereReentry.Alloc(
                heightSurfaceKm,
                coeffDrag,
                crossSectionalArea,
                inertialMassKg,
                earthMuSI,
                timeStepSec
            );

            // Add the external acceleration to GECore
            // Note: In a real implementation, you would use:
            // ExternalAccel.AccelType.EARTH_ATMOSPHERE_REENTRY
            // For now, we use CUSTOM since the type isn't yet integrated
            return ge.ExternalAccelerationAdd(
                id,
                ExternalAccel.ExtAccelType.SELF_INTEGRATED, // Self-integrated since it handles its own evolution
                ExternalAccel.AccelType.CUSTOM, // Use CUSTOM for now since EARTH_ATMOSPHERE_REENTRY not yet added
                data
            );
        }

        /// <summary>
        /// Validate the component parameters to ensure they are within reasonable ranges.
        /// </summary>
        private void OnValidate()
        {
            // Validate Earth surface height
            if (heightSurfaceKm <= 0)
            {
                Debug.LogWarning("Earth surface height must be positive");
                heightSurfaceKm = 6371;
            }

            // Validate spacecraft mass
            if (inertialMassKg <= 0)
            {
                Debug.LogWarning("Spacecraft mass must be positive");
                inertialMassKg = 1000;
            }

            // Validate cross-sectional area
            if (crossSectionalArea <= 0)
            {
                Debug.LogWarning("Cross-sectional area must be positive");
                crossSectionalArea = 10.0;
            }

            // Validate drag coefficient
            if (coeffDrag <= 0 || coeffDrag > 5)
            {
                Debug.LogWarning("Drag coefficient should be between 0 and 5");
                coeffDrag = Mathf.Clamp((float)coeffDrag, 0.1f, 5.0f);
            }

            // Validate time step
            if (timeStepSec <= 0 || timeStepSec > 10)
            {
                Debug.LogWarning("Time step should be between 0 and 10 seconds");
                timeStepSec = Mathf.Clamp((float)timeStepSec, 0.01f, 10.0f);
            }

            // Validate Earth gravitational parameter
            if (earthMuSI <= 0)
            {
                Debug.LogWarning("Earth gravitational parameter must be positive");
                earthMuSI = 3.986e14;
            }
        }

        /// <summary>
        /// Get a summary of the current configuration for debugging purposes.
        /// </summary>
        /// <returns>A string describing the current configuration</returns>
        public string GetConfigurationSummary()
        {
            return $"EarthAtmosphereReentry Configuration:\n" +
                   $"  Surface Height: {heightSurfaceKm:F1} km\n" +
                   $"  Spacecraft Mass: {inertialMassKg:F0} kg\n" +
                   $"  Cross-sectional Area: {crossSectionalArea:F1} m²\n" +
                   $"  Drag Coefficient: {coeffDrag:F2}\n" +
                   $"  Time Step: {timeStepSec:F3} s\n" +
                   $"  Earth μ: {earthMuSI:E2} m³/s²";
        }

        /// <summary>
        /// Log the current configuration to the console for debugging.
        /// </summary>
        [ContextMenu("Log Configuration")]
        public void LogConfiguration()
        {
            Debug.Log(GetConfigurationSummary());
        }
    }
}
