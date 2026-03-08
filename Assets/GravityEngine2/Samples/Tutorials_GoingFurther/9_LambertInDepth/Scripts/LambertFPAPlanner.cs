using System.Collections.Generic;
using TMPro;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.UI;

namespace GravityEngine2 {
    public class LambertFPAPlanner : MonoBehaviour {

        [Header("F/T to select graph. X to transfer")]
        public GSBody body1;
        public GSBody body2;
        public GSBody center;

        public GSDisplay gsDisplay;

        private GSController gsController;

        public GameObject orbitPrefab;

        public TextMeshProUGUI orbitText;

        public TextMeshProUGUI sliderText;

        public Slider fpaSlider;

        public Slider timeSlider;

        public float radius = 0.0f;

        public enum DVMode {
            DEPART,
            TOTAL
        }

        // TO_POINT can only evaluate depart velocity (since target point velocity is not known)
        private DVMode dvMode = DVMode.DEPART;

        public Plot2D plot2D;

        public GameObject plotMarker;

        private float activeWidth = 0.2f;

        private static int NUM_SCENARIOS = 3; // 2 MinTimes + variable from slider

        private static int SLIDER_INDEX = 2; // slider controlled transfer

        private float FPA_RANGE_DEG = 45.0f;

        private struct OrbitStep {
            public Lambert.LambertOutput lamOutput;
            public Renderer[] renders;

            public LineRenderer[] lineRenderers;
            public GameObject gameObject;

            public string name;
        }

        private OrbitStep[] orbitSteps;

        // Start is called before the first frame update
        void Start()
        {
            gsController = gsDisplay.gsController;
            gsController.ControllerStartedCallbackAdd(TransferShipOrbits);
            fpaSlider.minValue = -FPA_RANGE_DEG;
            fpaSlider.maxValue = FPA_RANGE_DEG;
            fpaSlider.onValueChanged.AddListener(OnFPASliderValueChanged);
            timeSlider.onValueChanged.AddListener(OnTimeSliderValueChanged);
        }



        private Color[] colors = { Color.red, Color.green, Color.blue, Color.yellow };

        private void ActivateOrbit(int index)
        {
            if (orbitSteps[index].gameObject != null) {
                foreach (LineRenderer lr in orbitSteps[index].lineRenderers) {
                    lr.startWidth = activeWidth;
                    lr.endWidth = activeWidth;
                    lr.material.color = colors[index];
                    if (lr.GetComponent<GSDisplayOrbitSegment>() != null) {
                        lr.startWidth = 4 * activeWidth;
                        lr.endWidth = 4 * activeWidth;
                    }
                }
            }
        }

        private void OnFPASliderValueChanged(float value)
        {
            int i = SLIDER_INDEX;
            double angle = math.radians(value);
            orbitSteps[i].lamOutput = Lambert.TransferProgradeToPointWithFPA(mu, body1State.r, body2State.r, body1State.v, angle, radius: radius);
            double tof = Lambert.ComputeTransferTime(mu, body1State.r, body2State.r, ref orbitSteps[i].lamOutput);
            timeSlider.SetValueWithoutNotify((float)tof);
            //Debug.Log($"tof={tof} status={orbitSteps[i].lamOutput.status} v1t={orbitSteps[i].lamOutput.v1t}");
            if (orbitSteps[i].lamOutput.status == Lambert.Status.OK || orbitSteps[i].lamOutput.status == Lambert.Status.HIT_PLANET) {
                // set the display orbit to show this transfer
                GSDisplayOrbit gsdo = orbitSteps[i].gameObject.GetComponent<GSDisplayOrbit>();
                gsdo.RVRelativeSet(body1State.r, orbitSteps[i].lamOutput.v1t, updateCOE: true);

                GSDisplayOrbitSegment gsdos = orbitSteps[i].gameObject.GetComponentInChildren<GSDisplayOrbitSegment>();
                gsdos.RVRelativeSet(body1State.r, orbitSteps[i].lamOutput.v1t, updateCOE: true);
                // Log the xfer orbit
                Orbital.COE xferOrbit = Orbital.RVtoCOE(body1State.r, orbitSteps[i].lamOutput.v1t, mu);
                sliderText.text = string.Format("{0} t={1:##.###} |dv|={2:##.###}", orbitSteps[i].name, orbitSteps[i].lamOutput.t,
                                        DV(orbitSteps[i].lamOutput));
                if (orbitSteps[i].lamOutput.status == Lambert.Status.HIT_PLANET) {
                    sliderText.text += "\n(hit planet)";
                }

                if (lamJob.xferType == LambertJob.LambertType.TO_POINT_BYANGLE) {
                    plot2D.MarkerUpdate(new Vector2(math.radians(value), (float)DV(orbitSteps[i].lamOutput)), plotMarker);
                } else {
                    plot2D.MarkerUpdate(new Vector2((float)tof, (float)DV(orbitSteps[i].lamOutput)), plotMarker);
                }
            } else {
                Debug.LogErrorFormat("LambertVallado.LambertUniv failed error={0}", orbitSteps[i].lamOutput.status);
            }
        }

        private void OnTimeSliderValueChanged(float value)
        {
            int i = SLIDER_INDEX;
            orbitSteps[i].lamOutput = Lambert.TransferProgradeToPoint(mu, body1State.r, body2State.r, body1State.v,
                            value, radius: radius, prograde: true);
            if (orbitSteps[i].lamOutput.status == Lambert.Status.OK || orbitSteps[i].lamOutput.status == Lambert.Status.HIT_PLANET) {
                // set the display orbit to show this transfer
                GSDisplayOrbit gsdo = orbitSteps[i].gameObject.GetComponent<GSDisplayOrbit>();
                gsdo.RVRelativeSet(body1State.r, orbitSteps[i].lamOutput.v1t, updateCOE: true);

                GSDisplayOrbitSegment gsdos = orbitSteps[i].gameObject.GetComponentInChildren<GSDisplayOrbitSegment>();
                gsdos.RVRelativeSet(body1State.r, orbitSteps[i].lamOutput.v1t, updateCOE: true);
                // Log the xfer orbit
                Orbital.COE xferOrbit = Orbital.RVtoCOE(body1State.r, orbitSteps[i].lamOutput.v1t, mu);
                // Debug.LogFormat("Xfer orbit: {0}", xferOrbit.LogStringDegrees());
                sliderText.text = string.Format("{0} t={1:##.###} |dv|={2:##.###}", orbitSteps[SLIDER_INDEX].name, orbitSteps[SLIDER_INDEX].lamOutput.t,
                                        DV(orbitSteps[SLIDER_INDEX].lamOutput));
                if (orbitSteps[i].lamOutput.status == Lambert.Status.HIT_PLANET) {
                    sliderText.text += "\n(hit planet)";
                }

                double fpaRad = Orbital.FlightPathAngle(body1State.r, orbitSteps[i].lamOutput.v1t);
                double fpaAngle = math.degrees(fpaRad);

                fpaSlider.SetValueWithoutNotify((float)fpaAngle);

                if (lamJob.xferType == LambertJob.LambertType.TO_POINT_BYANGLE) {
                    plot2D.MarkerUpdate(new Vector2((float)fpaRad, (float)DV(orbitSteps[i].lamOutput)), plotMarker);
                } else {
                    plot2D.MarkerUpdate(new Vector2((float)orbitSteps[i].lamOutput.t, (float)DV(orbitSteps[i].lamOutput)), plotMarker);
                }

            } else {
                Debug.LogErrorFormat("LambertVallado.LambertUniv failed error={0}", orbitSteps[i].lamOutput.status);
            }
        }

        private double DV(Lambert.LambertOutput lamOutput)
        {
            if (dvMode == DVMode.DEPART) {
                return math.length(lamOutput.v1t - body1State.v);
            } else {
                return math.length(lamOutput.v1t - body2State.v) + math.length(lamOutput.v2t - body2State.v);
            }
        }


        private GEBodyState body1State;
        private GEBodyState body2State;
        private double mu;

        double tmin, tminp, tminenergy;

        private void TransferShipOrbits(GSController controller)
        {
            Debug.Log("LambertOrbits - Pausing GSController");
            gsController.PausedSet(true);
            // create a TransferShip object
            GECore ge = controller.GECore();
            body1State = new GEBodyState();
            bool stateOk = ge.StateById(body1.Id(), ref body1State);
            body2State = new GEBodyState();
            stateOk = ge.StateById(body2.Id(), ref body2State);
            mu = ge.MuWorld(center.Id());

            orbitSteps = new OrbitStep[NUM_SCENARIOS];
            (tmin, tminp, tminenergy) = Lambert.MinTimesPrograde(mu, body1State.r, body2State.r, body1State.v);
            Debug.LogFormat("tmin={0} tminp={1} tminenergy={2}", tmin, tminp, tminenergy);

            string info = "";

            for (int i = 0; i < NUM_SCENARIOS; i++) {

                orbitSteps[i] = new OrbitStep();
                GameObject go = Instantiate(orbitPrefab);
                orbitSteps[i].gameObject = go;

                orbitSteps[i].renders = go.GetComponentsInChildren<Renderer>();
                orbitSteps[i].lineRenderers = go.GetComponentsInChildren<LineRenderer>();
                // get the min transfer time
                // double t_min = transferShip.GetMinTime();
                // Debug.LogFormat("Min transfer time: {0} days", t_min / 86400.0);
                double xferTime = 0.0;
                switch (i) {
                    case 0:
                        xferTime = tminp;
                        orbitSteps[i].name = "Parabola (red)";
                        break;
                    case 1:
                        xferTime = tminenergy;
                        orbitSteps[i].name = "Min Energy (green)";
                        break;
                    case 2: // slider controllerd. Start with 2* Parabola
                        xferTime = 2.0 * tminp;
                        orbitSteps[i].name = "Slider (blue)";
                        break;
                }
                orbitSteps[i].lamOutput = Lambert.TransferProgradeToPoint(mu, body1State.r, body2State.r, body1State.v,
                                xferTime, radius: 0.0, prograde: true);
                if (orbitSteps[i].lamOutput.status == Lambert.Status.OK) {
                    if (i != SLIDER_INDEX) {
                        info += string.Format("{0} t={1:##.###} depart |dv|={2:##.###}\n", orbitSteps[i].name, xferTime,
                                        DV(orbitSteps[i].lamOutput));
                    }
                    // set the display orbit to show this transfer
                    GSDisplayOrbit gsdo = go.GetComponent<GSDisplayOrbit>();
                    gsdo.centerDisplayBody = center.GetComponent<GSDisplayBody>();
                    gsDisplay.DisplayObjectAdd(gsdo);
                    gsdo.RVRelativeSet(body1State.r, orbitSteps[i].lamOutput.v1t, updateCOE: true);
                    gsdo.DisplayEnabledSet(true);

                    GSDisplayOrbitSegment gsdos = go.GetComponentInChildren<GSDisplayOrbitSegment>();
                    gsdos.centerDisplayBody = center.GetComponent<GSDisplayBody>();
                    gsDisplay.DisplayObjectAdd(gsdos);
                    gsdos.RVRelativeSet(body1State.r, orbitSteps[i].lamOutput.v1t, updateCOE: true);
                    gsdos.FromRVecSet(body1State.r);
                    gsdos.ToRVecSet(body2State.r);
                    gsdos.DisplayEnabledSet(true);
                    ActivateOrbit(i);
                    // Log the xfer orbit
                    Orbital.COE xferOrbit = Orbital.RVtoCOE(body1State.r, orbitSteps[i].lamOutput.v1t, mu);
                    Debug.LogFormat("Xfer orbit: {0}", xferOrbit.LogStringDegrees());
                } else {
                    Debug.LogErrorFormat("LambertVallado.LambertUniv failed error={0}", orbitSteps[i].lamOutput.status);
                }

            }
            orbitText.text = info;

            // set the slider scale from the parabola to the orbital period of body one
            Orbital.COE coe = Orbital.RVtoCOE(body1State.r, body1State.v, mu);
            double period = coe.GetPeriod();
            fpaSlider.maxValue = 80.0f;
            fpaSlider.minValue = -80.0f;
            fpaSlider.value = 0.0f;

            timeSlider.minValue = (float)(0.5 * tminp);
            timeSlider.maxValue = (float)(3.0 * tminenergy);
            FPAJob();
        }

        private bool jobRunning = false;
        private JobHandle jobHandle;

        private LambertJob lamJob;
        private int n = 100;

        private int counter = 0;

        private void FPAJob()
        {
            jobRunning = true;
            lamJob = new LambertJob(n, mu, body1State.r, body2State.r, body1State.v, body2State.v, 0.0, LambertJob.LambertType.TO_POINT_BYANGLE);
            // take angles from -45 to 45 degrees
            double dTheta = math.radians((fpaSlider.maxValue - fpaSlider.minValue) / (n - 1));
            double theta = math.radians(fpaSlider.minValue);
            for (int i = 0; i < n; i++) {
                lamJob.values[i] = theta;
                theta += dTheta;
            }
            jobHandle = lamJob.Schedule();
            counter = 0;
        }

        public void Update()
        {
            // F to plot FPA vs DV
            if (Input.GetKeyDown(KeyCode.F) && !jobRunning && plot2D != null) {
                FPAJob();
            }
            // T to plot TOF vs DV
            if (Input.GetKeyDown(KeyCode.T) && !jobRunning && plot2D != null) {
                jobRunning = true;
                lamJob = new LambertJob(n, mu, body1State.r, body2State.r, body1State.v, body2State.v, 0.0, LambertJob.LambertType.TO_POINT_BYTIME);
                // take times from tminp to 3*tminenergy
                double dTime = (timeSlider.maxValue - timeSlider.minValue) / (n - 1);
                double time = timeSlider.minValue;
                for (int i = 0; i < n; i++) {
                    lamJob.values[i] = time;
                    time += dTime;
                }
                jobHandle = lamJob.Schedule();
                counter = 0;
            }
            if (Input.GetKeyDown(KeyCode.X)) {
                // xfer to point (so do not have an arrival v2 target)
                List<GEManeuver> maneuvers = orbitSteps[SLIDER_INDEX].lamOutput.Maneuvers(center.Id(), body1.propagator, intercept: true);
                foreach (GEManeuver m in maneuvers) {
                    gsController.GECore().ManeuverAdd(body1.Id(), m);
                }
                gsController.PausedSet(false);
                // stop displaying the orbits, graph, sliders etc.
                for (int i = 0; i < NUM_SCENARIOS; i++) {
                    orbitSteps[i].gameObject.SetActive(false);
                }
                plot2D.gameObject.SetActive(false);
                fpaSlider.gameObject.SetActive(false);
                timeSlider.gameObject.SetActive(false);
                sliderText.gameObject.SetActive(false);
                orbitText.gameObject.SetActive(false);
                plotMarker.SetActive(false);
            }
            if (jobRunning) {
                double minDV = double.MaxValue;
                int minIndex = 0;
                if (jobHandle.IsCompleted) {
                    jobHandle.Complete();
                    Vector2[] points = new Vector2[n];
                    for (int i = 0; i < n; i++) {
                        double dv = DV(lamJob.outputs[i]);
                        if (dv < minDV) {
                            minDV = dv;
                            minIndex = i;
                        }
                        points[i] = new Vector2((float)lamJob.values[i], (float)dv);
                    }
                    if (lamJob.xferType == LambertJob.LambertType.TO_POINT_BYANGLE) {
                        plot2D.xLabelText = "Flight Path Angle (rad.)";
                    } else {
                        plot2D.xLabelText = "Time of Flight (world units)";
                    }
                    plot2D.PlotData(points);
                    // force a slider update to refesh the value on the curve
                    OnFPASliderValueChanged(fpaSlider.value);
                    jobRunning = false;
                    lamJob.Dispose();
                    Debug.Log($"Job completed counter={counter}");
                    Debug.LogFormat("Min DV={0} at index={1}", minDV, minIndex);
                } else {
                    counter++;
                }

            }
        }
    }
}
