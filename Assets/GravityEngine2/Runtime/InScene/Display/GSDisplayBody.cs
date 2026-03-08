using UnityEngine;
using Unity.Mathematics;

namespace GravityEngine2 {
    /// <summary>
    /// Class to display the location of a GSBody in a scene. 
    /// 
    /// See https://nbodyphysics.com/ge2/html/SceneComponents.html#gsdisplaybody
    /// </summary>
    public class GSDisplayBody : GSDisplayObject {

        public GSBody gsBody;

        // trajectory
        public bool showTrajectory;
        public LineRenderer trajLine;

        // display object
        public GameObject displayGO;

        public enum DisplayGOMode { ROTATION, VEL_ALIGN, PLANET_SURFACE, NONE };

        public DisplayGOMode displayGOMode;

        public bool doRotation;
        public float longitudeOffset;
        // odd ball special case. For now make it a toggle here. Icky.
        public bool computeEarthOffset;
        public bool scaleUsingBodyRadius = true;

        private Vector3 rotationAxisDisplay;
        private double omega;   // rad/sec
        private double phi0;    // initial rotation at t=0 in radians

        public GravityMath.AxisV3 axisV3;

        //! Align displayGO axis with the current velocity and using the specified yaw
        private Vector3 alignAxis = GravityMath.Axis(GravityMath.AxisV3.Y_AXIS);


        public float alignYawDeg;

        private Quaternion goInitialRotation;

        // Start is called before the first frame update
        void Awake()
        {
            GSBody attachedBody = GetComponent<GSBody>();
            if (attachedBody != null)
                gsBody = attachedBody;
            alignAxis = GravityMath.Axis(axisV3);
        }

        override
        public void DisplayEnabledSet(bool value)
        {
            base.DisplayEnabledSet(value);
            if (displayGO != null)
                displayGO.SetActive(value);
            if (trajLine != null)
                trajLine.enabled = value;
        }

        /// <summary>
        /// Each GravitySceneDisplay object will ask scene objects to add themselves so that each display object
        /// can specify the type of scene update (and associated info) it requires.
        /// </summary>
        /// <param name="gsd"></param>
        override
        public void AddToSceneDisplay(GSDisplay gsd)
        {
            if (!gsBody.gameObject.activeInHierarchy || gsBody.Id() < 0) {
                Debug.LogErrorFormat("GSDisplayBody {0} not active or id {1} is invalid", gsBody.gameObject.name, gsBody.Id());
                return;
            }
            this.gsd = gsd;
            GSDisplay.DisplayInScene displayFn = null;
            if (displayGO != null) {
                goInitialRotation = displayGO.transform.rotation;
                if (displayGOMode == DisplayGOMode.ROTATION && doRotation) {
                    omega = gsBody.rotationRate;
                    if (!gsd.xzOrbitPlane)
                        omega = -omega;
                    rotationAxisDisplay = gsBody.rotationAxis;
                    if (gsd.xzOrbitPlane)
                        GravityMath.Vector3ExchangeYZ(ref rotationAxisDisplay);
                    if (computeEarthOffset) {
                        // get JD and find location of lon=0 at this time
                        double jd = gsd.gsController.JDTime();
                        phi0 = (longitudeOffset - Orbital.GreenwichSiderealTime(jd)) % (2.0 * math.PI);
                    } else {
                        phi0 = longitudeOffset % (2.0 * math.PI);
                    }
                    displayFn = DisplayRotation;
                } else if (displayGOMode == DisplayGOMode.VEL_ALIGN) {
                    displayFn = AlignWithVelocity;
                } else if (displayGOMode == DisplayGOMode.PLANET_SURFACE) {
                    displayFn = AlignForSurface;
                }
                ScaleDisplayGO();
            }


            // display controller can just update our transform directly
            displayId = gsd.RegisterDisplayObject(this, gsBody.Id(),
                                                    transform: transform,
                                                    displayInScene: displayFn,
                                                    trajectory: showTrajectory,
                                                    trajLine: trajLine);

        }

        public void ScaleDisplayGO()
        {
            if (displayGOMode == DisplayGOMode.ROTATION && scaleUsingBodyRadius && displayGO != null) {
                if (gsBody.radius > 0) {
                    // assume body is a sphere. Scale provides the diameter
                    double unitConversion = GBUnits.DistanceConversion(gsBody.bodyInitData.units, gsd.gsController.defaultUnits);
                    float scale = (float)(2.0 * gsBody.radius * unitConversion * gsd.scale);
                    displayGO.transform.localScale = new Vector3(scale, scale, scale);
                }
            }
        }

        /// <summary>
		/// Rotate the model based on the rotation rate and axis from GSBody.
		///
		/// To allow for time jumps, determine the rotation alignment based on the
		/// current world time.
		///
		/// Note the rotation axis is specified in world space, so if there is an
		/// XZ orbit plane, need to adjust acccordingly (this is done when registering)
		/// </summary>
		/// <param name="mapToScene"></param>
		/// <param name="timeWorld"></param>
        private void DisplayRotation(GECore ge, GSDisplay.MapToSceneFn mapToScene, double timeWorld, bool alwaysUpdate = false, bool maintainCoRo = false)
        {
            if (displayEnabled) {
                double phi = (phi0 - timeWorld * omega) % (2.0 * math.PI);
                float angle = (float)(phi * GravityMath.RAD2DEG);
                displayGO.transform.rotation = goInitialRotation *
                    Quaternion.AngleAxis(angle, rotationAxisDisplay);
            }
        }

        private Vector3 velocity = Vector3.zero;
        GEBodyState bodyState = new GEBodyState();

        private void AlignWithVelocity(GECore ge, GSDisplay.MapToSceneFn mapToScene, double timeWorld, bool alwaysUpdate = false, bool maintainCoRo = false)
        {
            // debug hack until editor working
            if (displayEnabled) {
                // body might be FIXED (sitting on launch pad if LATLONG_POS). Do not align
                if (ge.PropagatorTypeById(gsBody.Id()) == GEPhysicsCore.Propagator.FIXED)
                    return;

                if (ge.StateById(gsBody.Id(), ref bodyState)) {
                    GravityMath.Double3IntoVector3(bodyState.v, ref velocity);
                    if (gsd.xzOrbitPlane)
                        GravityMath.Vector3ExchangeYZ(ref velocity);
                    displayGO.transform.rotation = Quaternion.AngleAxis(alignYawDeg, alignAxis) *
                        Quaternion.FromToRotation(alignAxis, velocity) * goInitialRotation;
                }
            }
        }

        private void AlignForSurface(GECore ge, GSDisplay.MapToSceneFn mapToScene, double timeWorld, bool alwaysUpdate = false, bool maintainCoRo = false)
        {
            if (displayEnabled) {
                if (ge.StateById(gsBody.Id(), ref bodyState)) {
                    Vector3 r_unit = GravityMath.Double3ToVector3(bodyState.r).normalized;
                    if (gsd.xzOrbitPlane)
                        GravityMath.Vector3ExchangeYZ(ref r_unit);
                    // now orient toward velocity
                    Vector3 v_unit = GravityMath.Double3ToVector3(bodyState.v).normalized;
                    if (gsd.xzOrbitPlane)
                        GravityMath.Vector3ExchangeYZ(ref v_unit);
                    Quaternion q2 = new Quaternion();
                    q2.SetLookRotation(r_unit, v_unit);
                    displayGO.transform.rotation = q2 * goInitialRotation;
                }
            }
        }


#if UNITY_EDITOR
        /// <summary>
        /// Stateless version used for Editor Gizmo stuff
        /// </summary>
        /// <returns></returns>
        public GSBody EditorGetGSBody()
        {
            GSBody gsb = gsBody;
            if (gsb == null)
                gsb = GetComponent<GSBody>();
            return gsb;
        }
#endif
    }
}
