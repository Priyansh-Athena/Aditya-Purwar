using UnityEngine;
using Unity.Mathematics;

namespace GravityEngine2 {
    /// <summary>
    /// Display the specified point on the Orbit referenced by GSDisplayOrbit by moving the transform 
    /// of this object to the correct location. 
    /// 
    /// This is done in the context of a specific GravitySceneDisplay.
    /// 
    /// </summary>
    public class GSDisplayOrbitPoint : GSDisplayObject {
        public GSDisplayOrbit displayOrbit;

        public Orbital.OrbitPoint orbitPoint = Orbital.OrbitPoint.PERIAPSIS;

        public double trueAnomDeg;

        private int center_id;


        private void DisplayPoint(GECore ge, GSDisplay.MapToSceneFn mapToScene, double t, bool alwaysUpdate = false, bool maintainCoRo = false)
        {
            Orbital.COE coe = displayOrbit.LastCOE();
            // simple mapping to orbit point
            Orbital.OrbitPoint point = orbitPoint;
            (double3 r, double3 v) = Orbital.RVForOrbitPoint(coe, point, deg: trueAnomDeg);
            Vector3 r_vec = GravityMath.Double3ToVector3(r);
            if (gsd.xzOrbitPlane)
                GravityMath.Vector3ExchangeYZ(ref r_vec);
            // center may not be at zero
            GEBodyState centerState = new GEBodyState();
            ge.StateById(center_id, ref centerState, maintainCoRo: maintainCoRo);
            Vector3 centerPos = GravityMath.Double3ToVector3(centerState.r);
            transform.position = mapToScene(GravityMath.Vector3ToDouble3(r_vec + centerPos));
        }


        public override void AddToSceneDisplay(GSDisplay gsd)
        {
            int body_id = -1;

            if (displayOrbit.bodyToDisplay != null) {
                body_id = displayOrbit.bodyToDisplay.gsBody.Id();
                if (!displayOrbit.bodyToDisplay.gsBody.gameObject.activeInHierarchy || displayOrbit.bodyToDisplay.gsBody.Id() < 0) {
                    Debug.LogErrorFormat("{0} not active or id {1} is invalid", displayOrbit.bodyToDisplay.gsBody.gameObject.name, displayOrbit.bodyToDisplay.gsBody.Id());
                    return;
                }
            }

            center_id = displayOrbit.centerDisplayBody.gsBody.Id();
            this.gsd = gsd;
            displayId = gsd.RegisterDisplayObject(this, body_id,
                                            transform: null,
                                            displayInScene: DisplayPoint);
        }
    }
}
