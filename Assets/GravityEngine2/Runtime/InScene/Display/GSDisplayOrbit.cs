using UnityEngine;
using Unity.Mathematics;

namespace GravityEngine2 {

    /// <summary>
    /// GSDisplayOrbit:
    /// This component has several usages:
    /// 1) Display an orbit for an existing (evolving) orbit in the scene
    /// - assumes the body is attached to component that also has GSDisplayBody for the object for which an orbit will
    ///   be drawn
    /// 
    /// 2) Display an orbit for the (r, v) state specified via the inspector or (more typically) via a script.
    /// - this option is assumed if the displayBody is null
    /// - this option moves the transform of this object to the r location in the scene
    ///
    /// In both cases a reference to a center object is required. The orbit rendered is Keplerian i.e. it is assumed
    /// the center body is a gravitational point source and no external bodies are affecting the path of the orbit
    /// via their gravity. 
    /// 
    /// Commonly a LineRenderer is also attached to the same component. If detected, the GSDisplayOrbit will configure
    /// the points in the line renderer to correspond to the points in the orbit. 
    /// 
    /// External scripts can get a copy of the orbit points via GetOrbitPoints() if some other rendering approach is
    /// preferred. 
    /// 
    /// The editor script for this component also draws an orbit gizmo in the editor when not playing.
    /// 
    /// </summary>
    public class GSDisplayOrbit : GSDisplayObject {

        // when non-null, this is the body that will be tracked
        public GSDisplayBody bodyToDisplay;

        // show orbit with respect to this center body
        public GSDisplayBody centerDisplayBody;

        // In cases where want to display a reference orbit without a specific body being in that orbit
        // allow user to provide the details of the orbit in any input format they wish.
        public BodyInitData bodyInitData = new BodyInitData();

        private int body_id;
        private int center_id = -1;
        public int numPoints = 250;

        public LineRenderer lineR;

        // r and v in GE units (BID has them in world space in units specified by BID)
        private double3 r, v;
        private double mu;

        //! Cache the COE so inspector can display for debug
        private Orbital.COE coe_last;

        private Vector3 centerPosLast;

        private double3 r_last;

        private Vector3[] points;

        // Start is called before the first frame update
        void Awake()
        {
            if (lineR == null)
                lineR = GetComponent<LineRenderer>();
            if (lineR != null)
                lineR.useWorldSpace = true;
            points = new Vector3[numPoints];
            // editor script may not have set this up. Does mean if we want an empty orbit
            // cannot have a GSBody attached
            if (bodyToDisplay == null) {
                bodyToDisplay = GetComponent<GSDisplayBody>();
                // make an attempt to get center display body if it is null
                if (centerDisplayBody == null && transform.parent != null) {
                    centerDisplayBody = transform.parent.GetComponent<GSDisplayBody>();
                }
            }
            centerPosLast = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
        }

        override
        public void DisplayEnabledSet(bool value)
        {
            base.DisplayEnabledSet(value);
            if (lineR != null)
                lineR.enabled = value;
        }


        public override void AddToSceneDisplay(GSDisplay gsd)
        {
            this.gsd = gsd;
            center_id = centerDisplayBody.gsBody.Id();
            if (bodyToDisplay != null) {
                if (!bodyToDisplay.gsBody.gameObject.activeInHierarchy || bodyToDisplay.gsBody.Id() < 0) {
                    Debug.LogErrorFormat("{0} not active or id {1} is invalid", bodyToDisplay.gsBody.gameObject.name, bodyToDisplay.gsBody.Id());
                    lineR.enabled = false;
                    return;
                }
            }

            SetCOELast();
            this.gsd = gsd;

            if (bodyToDisplay == null) {
                // use Update generic for case when not tracking a body
                displayId = gsd.RegisterDisplayObject(this,
                            -1,
                            transform: null,
                            displayInScene: DisplayOrbit);
            } else {
                body_id = bodyToDisplay.gsBody.Id();
                displayId = gsd.RegisterDisplayObject(this,
                    body_id,
                    transform: null,
                    displayInScene: DisplayOrbit);
            }
        }

        private void SetCOELast()
        {
            coe_last = new Orbital.COE();
            mu = gsd.gsController.GECore().MuWorld(center_id);

            GBUnits.Units sceneUnits = gsd.gsController.defaultUnits;
            if (centerDisplayBody == null) {
                Debug.LogError("centerDisplayBody is not set for " + gameObject.name);
                return;
            }

            if (bodyToDisplay == null) {
                // initial data has been given. This is in units specified in the BID, need to scale to GE
                if (bodyInitData.initData == BodyInitData.InitDataType.RV_RELATIVE) {
                    if (bodyInitData.units != sceneUnits) {
                        r *= GBUnits.DistanceConversion(bodyInitData.units, sceneUnits);
                        v *= GBUnits.DistanceConversion(bodyInitData.units, sceneUnits);
                    }
                    coe_last = Orbital.RVtoCOE(r, v, mu);
                } else {
                    bodyInitData.FillInCOE(coe_last);
                    if (bodyInitData.units != sceneUnits) {
                        coe_last.p *= GBUnits.DistanceConversion(bodyInitData.units, sceneUnits);
                        coe_last.a *= GBUnits.DistanceConversion(bodyInitData.units, sceneUnits);
                    }
                }
            } else {
                body_id = bodyToDisplay.gsBody.Id();
                coe_last = gsd.gsController.GECore().COE(body_id, center_id);
            }
            coe_last.mu = mu;
        }


        /// <summary>
        /// Set the relative R, V of the orbit display in GE units.
        /// 
        /// If there is a display body to track then that will be used to determine the orbit.
        /// The display body should be set to null before this is used. 
        /// </summary>
        /// <param name="r"></param>
        /// <param name="v"></param>
        /// <param name="updateCOE">If true, will update the COE to match the new r, v</param>
        public void RVRelativeSet(double3 r, double3 v, bool updateCOE = true)
        {
            if (bodyToDisplay != null) {
                Debug.LogError("GSDisplayOrbit.RVRelativeSet() called with a body display set. Set to null before calling.");
                return;
            }
            this.r = r;
            this.v = v;
            if (updateCOE)
                SetCOELast();
#if UNITY_EDITOR
            // update BID so we can see r, v in inspector
            bodyInitData.r = r;
            bodyInitData.v = v;
            bodyInitData.initData = BodyInitData.InitDataType.RV_RELATIVE;
#endif
        }

        /// <summary>
        /// Callback from GSDisplay to allow this code to determine the current 
        /// orbit elements and create a series of points for a line renderer to 
        /// display the orbit. 
        /// 
        /// This can be expensive and the use of the display object "framesBetweenUpdates"
        /// can be used to reduce how often this routine is called. By default GSDisplay will
        /// call it on every Update() cycle.
        /// 
        /// If maintainCoRo is true, then we must compute the orbit in the inertial frame (this
        /// is how COE is defined) and then transform to the CoRo frame.
        /// </summary>
        /// <param name="ge"></param>
        /// <param name="mapToScene"></param>
        /// <param name="t"></param>
        /// <param name="alwaysUpdate"></param>
        /// <param name="maintainCoRo"></param>
        public void DisplayOrbit(GECore ge, GSDisplay.MapToSceneFn mapToScene, double t, bool alwaysUpdate = false, bool maintainCoRo = false)
        {
            if (displayEnabled) {
                Orbital.COE coe;
                Vector3 centerPos = GravityMath.Double3ToVector3(ge.RWorldById(center_id, maintainCoRo: maintainCoRo));

                double3 rBody;
                if (bodyToDisplay != null) {
                    coe = ge.COE(body_id, center_id);
                    GEBodyState state = new GEBodyState();
                    ge.StateByIdRelative(body_id, center_id, ref state);
                    rBody = state.r;
                    if (!maintainCoRo && !alwaysUpdate && Orbital.COEEqual(coe, coe_last, includePhase: false) &&
                        Vector3.Distance(centerPos, centerPosLast) < 1E-2 &&
                        math.length(rBody - r_last) < 1E-2) {
                        // line renderer has the current values, no need to update
                        return;
                    }
                    coe_last = coe;
                } else {
                    coe = coe_last; // use COE from BodyInitData (not tracking body)
                    rBody = bodyInitData.r;
                }
                centerPosLast = centerPos;
                r_last = rBody;
                if (double.IsNaN(coe.e)) {
                    // radial infall, just do current pos and center
                    for (int i = 0; i < points.Length; i++) {
                        points[i] = new Vector3(0, 0, 0);
                    }
                    points[points.Length - 1] = GravityMath.Double3ToVector3(rBody);
                } else {
                    OrbitDisplayCommon.OrbitPositions(coe, ref points);
                }
                // if CoRo, need to rotate the world COE into the CoRo frame
                if (maintainCoRo)
                    CR3BP.InertialToRotatingVec3(ref points, ge.TimeGE());
                for (int i = 0; i < points.Length; i++) {
                    points[i] = mapToScene(GravityMath.Vector3ToDouble3(points[i] + centerPos));
                }
                // if ellipse, then wrap the renderer
                if (coe.e < 1.0)
                    points[points.Length - 1] = points[0];
                if (lineR != null) {
                    lineR.positionCount = points.Length;
                    lineR.SetPositions(points);
                }
            } else {
                if (lineR != null && lineR.positionCount > 0)
                    lineR.positionCount = 0;
            }
        }

        /// <summary>
        /// Return the points of the orbit in display space coordinates. 
        /// </summary>
        /// <returns></returns>
        public Vector3[] OrbitPoints()
        {
            return points;
        }

        public Orbital.COE LastCOE()
        {
            if (center_id < 0) {
                return null;
            }
            if (coe_last == null)
                SetCOELast();
            return coe_last;
        }


#if UNITY_EDITOR
        private Orbital.COE gizmoCOE;

        public Vector3[] GizmoOrbitPositions(ref Vector3[] editorPoints)
        {

            if (centerDisplayBody == null) {
                Debug.LogWarning("Could not find display body for " + gameObject.name);
                return null;
            }
            if (bodyToDisplay != null) {
                if (bodyToDisplay.gsBody != null) {
                    (bool haveCoe, Orbital.COE coe) = bodyToDisplay.gsBody.EditorCOE();
                    if (haveCoe)
                        OrbitDisplayCommon.OrbitPositions(coe, ref editorPoints);
                }
            } else {
                if (gizmoCOE == null)
                    gizmoCOE = new Orbital.COE();
                bodyInitData.FillInCOE(gizmoCOE);
                OrbitDisplayCommon.OrbitPositions(gizmoCOE, ref editorPoints);
            }
            return null;
        }

        public GSBody EditorGetCenterGSBody()
        {
            return centerDisplayBody.EditorGetGSBody();
        }
#endif

    }
}
