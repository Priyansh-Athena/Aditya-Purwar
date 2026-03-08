using Unity.Mathematics;
using UnityEngine;

namespace GravityEngine2 {

    /// <summary>
    /// Display the path of a body on a rails. The time to display can be from the
    /// current time or a specific time. 
    /// 
    /// If the current time is earlier than the start time then nothing will be displayed.
    /// </summary>
    public class GSDisplayOnRailsPath : GSDisplayObject {

        public GSDisplayBody bodyToDisplay;

        [Header("Start time. If < 0 will start at current time.")]
        public double startWorldTime = -1;

        public double durationWorldTime = 100;

        public enum Mode { RELATIVE, ABSOLUTE };

        public Mode mode = Mode.RELATIVE;

        private double dT = 0.0;

        public LineRenderer lineR;
        public int numPoints = 250;

        private Vector3[] points;

        private Vector3 centerPosLast;

        private int body_id;

        private GECore ge;

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

            }
            centerPosLast = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
            dT = durationWorldTime / numPoints;
        }


        public override void AddToSceneDisplay(GSDisplay gsd)
        {
            this.gsd = gsd;
            body_id = bodyToDisplay.gsBody.Id();

            if (bodyToDisplay != null) {
                if (!bodyToDisplay.gsBody.gameObject.activeInHierarchy || bodyToDisplay.gsBody.Id() < 0) {
                    Debug.LogErrorFormat("{0} not active or id {1} is invalid", bodyToDisplay.gsBody.gameObject.name, bodyToDisplay.gsBody.Id());
                    lineR.enabled = false;
                    return;
                }
                // check the body is on rails
                ge = gsd.gsController.GECore();
                if (!ge.BodyOnRails(body_id)) {
                    Debug.LogErrorFormat("{0} is not on rails", bodyToDisplay.gsBody.gameObject.name);
                    lineR.enabled = false;
                    return;
                }
            }

            this.gsd = gsd;

            if (bodyToDisplay == null) {
                Debug.LogError("GSDisplayOnRailsPath: bodyToDisplay is null");
            } else {
                displayId = gsd.RegisterDisplayObject(this,
                    body_id,
                    transform: null,
                    displayInScene: DisplayOrbit);
            }
        }


        public void DisplayOrbit(GECore ge, GSDisplay.MapToSceneFn mapToScene, double t_now, bool alwaysUpdate = false, bool maintainCoRo = false)
        {
            if (displayEnabled) {
                if (bodyToDisplay == null) {
                    Debug.LogError("GSDisplayOnRailsPath: bodyToDisplay is null");
                } else {
                    // greedy for now. Eventually do a circular buffer of points and update only as needed
                    // (but watch out for when maneuvers have changed the patches or the state)
                    GEBodyState state = new GEBodyState();
                    double tp = startWorldTime;
                    if (mode == Mode.RELATIVE) {
                        tp += t_now;
                    }
                    if (tp < 0.0) {
                        tp = 0.0;
                    }
                    for (int i = 0; i < numPoints; i++) {
                        ge.StateByIdAtTime(body_id, tp, ref state);
                        tp += dT;
                        if (state.HasNaN()) {
                            Debug.LogErrorFormat("State has NaN at time {0} skipping {1}", tp, state.LogString());
                            continue;
                        }
                        points[i] = mapToScene(state.r);
                    }
                    lineR.positionCount = numPoints;
                    lineR.SetPositions(points);
                }
            }
        }
    }
}
