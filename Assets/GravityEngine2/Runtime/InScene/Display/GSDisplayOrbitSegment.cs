using UnityEngine;
using Unity.Mathematics;

namespace GravityEngine2 {

    /// <summary>
    /// Displays a segment of an orbit around a central body. 
    ///
	/// The to and from locations can be specified as a true anomaly or by referencing an orbit point type
	/// Orbital.OrbitPoint (peri, apo, nodes). Note that not all orbit points are defined for all orbit
	/// types (e.g. a circle has no apopapsis).
	/// 
    /// </summary>
    public class GSDisplayOrbitSegment : GSDisplayObject {

        // show orbit with respect to this center body
        public GSDisplayBody centerDisplayBody;

        // when non null, this is the body that will be used to determine the orbit (but not the to or from point)
        public GSDisplayBody bodyDisplay;

        // In cases where want to display a reference orbit without a specific body being in that orbit
        // allow user to provide the details of the orbit in any input format they wish.
        public BodyInitData bodyInitData = new BodyInitData();

        // This enum extends Orbital>OrbitPoint to add the body as a choice. The Body entry MUST
        // be last to maintain integer value alignment. 
        public enum OrbitSegmentPoint { APOAPSIS, PERIAPSIS, ASC_NODE, DESC_NODE, TRUEANOM_DEG, BODY, RVEC_WORLD };

        // from can be from a specific body or from a designated point on an orbit
        public OrbitSegmentPoint fromPoint = OrbitSegmentPoint.TRUEANOM_DEG;
        // Units only needed if doing to/from a relataive world vector
        public GBUnits.Units units;
        public double fromTrueAnomDeg = 0;
        public double3 fromRvec;
        public OrbitSegmentPoint toPoint = OrbitSegmentPoint.TRUEANOM_DEG;
        public double toTrueAnomDeg = 0;
        public double3 toRvec;

        private int body_id;
        private int center_id;

        private double fromTA, toTA;
        // radial relative position of body in world units
        private double3 bodyR;

        public int numPoints = 200;

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
            fromTA = math.radians(fromTrueAnomDeg);
            toTA = math.radians(toTrueAnomDeg);

            points = new Vector3[numPoints];
        }

        override
        public void DisplayEnabledSet(bool value)
        {
            displayEnabled = value;
            if (lineR != null)
                lineR.enabled = value;
        }

        public override void AddToSceneDisplay(GSDisplay gsd)
        {
            if (bodyDisplay != null) {
                if (!bodyDisplay.gsBody.gameObject.activeInHierarchy || bodyDisplay.gsBody.Id() < 0) {
                    Debug.LogErrorFormat("{0} not active or id {1} is invalid", bodyDisplay.gsBody.gameObject.name, bodyDisplay.gsBody.Id());
                    lineR.enabled = false;
                    return;
                }
            }
            this.gsd = gsd;
            coe_last = new Orbital.COE();
            GBUnits.Units sceneUnits = gsd.gsController.defaultUnits;
            center_id = centerDisplayBody.gsBody.Id();
            mu = gsd.gsController.GECore().MuWorld(center_id);
            coe_last.mu = mu;


            if (bodyDisplay == null) {
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
                // use Update generic for case when not tracking a body
                displayId = gsd.RegisterDisplayObject(this,
                            -1,
                            transform: null,
                            displayInScene: DisplaySegment);
            } else {
                body_id = bodyDisplay.gsBody.Id();
                this.gsd = gsd;
                displayId = gsd.RegisterDisplayObject(this,
                    body_id,
                    transform: null,
                    displayInScene: DisplaySegment);
            }
        }

        /// <summary>
        /// Set the relative R, V of the orbit display in GE units. 
        /// 
        /// If there is a display body to track then that will be used to determine the orbit.
        /// The display body should be set to null before this is used. 
        /// </summary>
        /// <param name="r"></param>
        /// <param name="v"></param>
        /// <param name="mu"></param>
        public void RVRelativeSet(double3 r, double3 v, bool updateCOE = true)
        {
            if (bodyDisplay != null) {
                Debug.LogError("GSDisplayOrbitSegment.RVRelativeSet() called with a body display set. Set to null before calling.");
                return;
            }
            this.r = r;
            this.v = v;
            if (updateCOE)
                coe_last = Orbital.RVtoCOE(r, v, mu);
#if UNITY_EDITOR
            // update BID so we can see r, v in inspector
            bodyInitData.r = r;
            bodyInitData.v = v;
            bodyInitData.initData = BodyInitData.InitDataType.RV_RELATIVE;
#endif
        }


        public void FromRVecSet(double3 r)
        {
            fromPoint = OrbitSegmentPoint.RVEC_WORLD;
            fromRvec = r;
        }

        public void ToRVecSet(double3 r)
        {
            toPoint = OrbitSegmentPoint.RVEC_WORLD;
            toRvec = r;
        }

        private double TAforPoint(OrbitSegmentPoint point, Orbital.COE coe, double taDeg, double3 r)
        {
            double ta = 0;
            switch (point) {
                case OrbitSegmentPoint.APOAPSIS:
                case OrbitSegmentPoint.PERIAPSIS:
                case OrbitSegmentPoint.ASC_NODE:
                case OrbitSegmentPoint.DESC_NODE:
                    Orbital.OrbitPoint fromOP = (Orbital.OrbitPoint)point;
                    ta = Orbital.PhaseForOrbitPoint(coe, fromOP, fromTrueAnomDeg);
                    break;
                case OrbitSegmentPoint.TRUEANOM_DEG:
                    ta = math.radians(taDeg);
                    break;
                case OrbitSegmentPoint.BODY:
                    ta = coe.nu;
                    break;
                case OrbitSegmentPoint.RVEC_WORLD:
                    ta = Orbital.PhaseAngleRadiansForDirection(r, coe);
                    break;
            }
            return ta;
        }

        /// <summary>
        /// Update orbit display points in the line render based on the provided COE. Typically used by
        /// GravitySceneController to show an orbit for an existing body.
        /// 
        /// </summary>
        /// <param name="gsd"></param>
        /// <param name="coe">COE in world units</param>
        /// <param name="geToSceneScale"></param>
        public void DisplaySegment(GECore ge, GSDisplay.MapToSceneFn mapToScene, double t, bool alwaysUpdate, bool maintainCoRo = false)
        {
            if (displayEnabled) {
                Orbital.COE coe;
                Vector3 centerPos = GravityMath.Double3ToVector3(ge.RWorldById(center_id, maintainCoRo: maintainCoRo));
                double3 rBody = new double3(0, 0, 0);

                if (bodyDisplay != null) {
                    coe = ge.COE(body_id, center_id);
                    GEBodyState state = new GEBodyState();
                    ge.StateByIdRelative(body_id, center_id, ref state);
                    rBody = state.r;
                    if (!alwaysUpdate && Orbital.COEEqual(coe, coe_last, includePhase: false) &&
                        Vector3.Distance(centerPos, centerPosLast) < 1E-2 &&
                        math.length(rBody - r_last) < 1E-2) {
                        // line renderer has the current values, no need to update
                        return;
                    }
                    coe_last = coe;
                } else {
                    coe = coe_last; // use COE from BodyInitData (not tracking body)
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
                    fromTA = TAforPoint(fromPoint, coe, fromTrueAnomDeg, fromRvec);
                    toTA = TAforPoint(toPoint, coe, toTrueAnomDeg, toRvec);
                    points = OrbitDisplayCommon.OrbitPositionsToFromTA(coe, numPoints, fromTA, toTA);
                }

                for (int i = 0; i < points.Length; i++) {
                    points[i] = mapToScene(GravityMath.Vector3ToDouble3(points[i] + centerPos));
                }

                if (lineR != null) {
                    lineR.positionCount = points.Length;
                    lineR.SetPositions(points);
                }

            } else {
                if (lineR != null && lineR.positionCount > 0)
                    lineR.positionCount = 0;
            }
        }

        public Orbital.COE LastCOE()
        {
            return coe_last;
        }

        /// <summary>
        /// Return the points of the orbit in display space coordinates. 
        /// </summary>
        /// <returns></returns>
        public Vector3[] OrbitPoints()
        {
            return points;
        }


#if UNITY_EDITOR
        private Orbital.COE gizmoCOE;

        // TODO: Show just the segment
        public void GizmoOrbitPositions(ref Vector3[] points, bool xzOrbit)
        {

            if (centerDisplayBody == null) {
                Debug.LogWarning("Could not find display body for " + gameObject.name);
                return;
            }
            if (bodyDisplay != null) {
                if (bodyDisplay.gsBody != null) {
                    (bool haveCoe, Orbital.COE coe) = bodyDisplay.gsBody.EditorCOE();
                    if (haveCoe)
                        OrbitDisplayCommon.OrbitPositions(coe, ref points);
                }
            } else {
                if (gizmoCOE == null)
                    gizmoCOE = new Orbital.COE();
                bodyInitData.FillInCOE(gizmoCOE);
                OrbitDisplayCommon.OrbitPositions(gizmoCOE, ref points);
            }
        }

        public GSBody EditorGetCenterGSBody()
        {
            return centerDisplayBody.EditorGetGSBody();
        }
#endif

    }
}
