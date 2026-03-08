using Unity.Mathematics;
using UnityEngine;

namespace GravityEngine2 {
    /// <summary>
    /// Set the illumination direction for a directional light for the Earth based on evolving time
    /// in world seconds.
    /// </summary>


    public class EarthSunlight : MonoBehaviour {

        public GSBody earth;
        public Light directionalLight;
        public GSController gsController;



        // Start is called before the first frame update
        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {
            double jd = gsController.JDTime();
            (double3 rSun, double ra, double decl) = SolarSystemTools.Sun(jd);
        }
    }
}
