using System.Collections.Generic;
using TMPro;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.UI;

namespace GravityEngine2 {
    public class LambertToTargetPlanner : MonoBehaviour {


        [Header("Press X to initiate transfer")]
        public GSBody ship;
        public GSBody target;
        public GSBody center;

        public GSDisplay gsDisplay;

        private GSController gsController;

        public GameObject orbitPrefab;

        [Header("Transfer Time As Factor of Target Orbit Period")]
        public double xferTimeFactor = 0.4;

        public TextMeshProUGUI sliderText;

        public Slider slider;

        public enum DVMode {
            DEPART,
            TOTAL
        }
        public DVMode dvMode = DVMode.TOTAL;

        public Plot2D plot2D;

        public GameObject markerObject;

        [Header("Planet Radius (if non-zero will check for collision)")]
        public double radius = 0.0;

        private struct OrbitDisplay {
            public GSDisplayOrbit orbit;
            public GSDisplayOrbitSegment segment;
        }

        private OrbitDisplay shipOrbit;
        private OrbitDisplay targetOrbit;

        private GEBodyState body1State;
        private GEBodyState body2State;
        private double mu;

        private KeplerPropagator.RVT targetProp;

        private float targetPeriod;

        // Start is called before the first frame update
        void Start()
        {
            gsController = gsDisplay.gsController;
            gsController.ControllerStartedCallbackAdd(SetupOrbits);
            slider.onValueChanged.AddListener(OnSliderValueChanged);
        }


        private Color[] colors = { Color.red, Color.green, Color.blue, Color.yellow };



        private void OnSliderValueChanged(float value)
        {
            ComputeTransfer(value * targetPeriod);
        }

        private double DV(Lambert.LambertOutput lamOutput)
        {
            if (dvMode == DVMode.DEPART) {
                return math.length(lamOutput.v1t - lamOutput.v1);
            } else {
                return math.length(lamOutput.v1t - lamOutput.v1) + math.length(lamOutput.v2t - lamOutput.v2);
            }
        }

        private Lambert.LambertOutput lamOutput;
        private void ComputeTransfer(double time)
        {

            lamOutput = Lambert.TransferProgradeToTarget(mu, body1State.r, body2State.r, body1State.v, body2State.v, time, radius: radius, prograde: true);
            bool showOrbit = lamOutput.status == Lambert.Status.OK || lamOutput.status == Lambert.Status.HIT_PLANET;
            if (!showOrbit) {
                Debug.LogErrorFormat("Error computing Lambert transfer err={0}", lamOutput.status);
                shipOrbit.segment.DisplayEnabledSet(false);
                targetOrbit.segment.DisplayEnabledSet(false);
                return;
            }
            // Debug.LogFormat("Transfer time={0} (days={1})", time, time / 86400.0);
            // display the propagation of the target using the orbit segment
            targetOrbit.segment.ToRVecSet(lamOutput.r2);
            targetOrbit.segment.DisplayEnabledSet(true);

            // set the ship orbit to the transfer point
            shipOrbit.orbit.RVRelativeSet(lamOutput.r1, lamOutput.v1t, updateCOE: true);
            shipOrbit.segment.RVRelativeSet(lamOutput.r1, lamOutput.v1t, updateCOE: true);
            shipOrbit.segment.FromRVecSet(lamOutput.r1);
            shipOrbit.segment.ToRVecSet(lamOutput.r2);
            shipOrbit.segment.DisplayEnabledSet(true);

            sliderText.text = string.Format("TOF={0:F3} DV={1:F3}", time, DV(lamOutput));
            if (lamOutput.status == Lambert.Status.HIT_PLANET) {
                sliderText.text += "\nHIT PLANET";
            }

            plot2D.MarkerUpdate(new Vector2((float)time, (float)DV(lamOutput)), markerObject);
        }


        private void SetupOrbits(GSController controller)
        {
            Debug.Log("LambertOrbits - Pausing GSController");
            gsController.PausedSet(true);
            // create a TransferShip object
            GECore ge = controller.GECore();
            body1State = new GEBodyState();
            bool stateOk = ge.StateById(ship.Id(), ref body1State);
            body2State = new GEBodyState();
            stateOk = ge.StateById(target.Id(), ref body2State);
            mu = ge.MuWorld(center.Id());

            shipOrbit = OrbitDisplayCreate(body1State.r, body1State.v);
            shipOrbit.orbit.gameObject.name = "Ship Orbit";
            targetOrbit = OrbitDisplayCreate(body2State.r, body2State.v);
            targetOrbit.orbit.gameObject.name = "Target Orbit";

            targetProp = new KeplerPropagator.RVT(body2State.r, body2State.v, t0: 0.0, mu);

            // setup time slider. Assume it will be from 0.1 to 1.0 of the period of the target orbit
            targetPeriod = (float)ge.COE(target.Id(), center.Id(), false).GetPeriod();
            Debug.LogFormat("Target period={0} coe={1}", targetPeriod, ge.COE(target.Id(), center.Id(), false).LogStringDegrees());
            slider.minValue = 0.1F;
            slider.maxValue = 1.0f;
            slider.value = (float)xferTimeFactor;

            LambertJobStart();
        }

        private OrbitDisplay OrbitDisplayCreate(double3 r, double3 v)
        {
            OrbitDisplay od = new OrbitDisplay();
            GameObject go = Instantiate(orbitPrefab);
            od.orbit = go.GetComponent<GSDisplayOrbit>();
            od.segment = go.GetComponentInChildren<GSDisplayOrbitSegment>();
            od.orbit.centerDisplayBody = center.GetComponent<GSDisplayBody>();
            od.orbit.bodyInitData.units = gsController.defaultUnits;
            od.segment.centerDisplayBody = center.GetComponent<GSDisplayBody>();
            od.segment.bodyInitData.units = gsController.defaultUnits;
            od.segment.units = gsController.defaultUnits;
            gsDisplay.DisplayObjectAdd(od.orbit);
            gsDisplay.DisplayObjectAdd(od.segment);

            // set initial state
            od.orbit.RVRelativeSet(r, v, updateCOE: true);
            od.orbit.DisplayEnabledSet(true);
            od.segment.RVRelativeSet(r, v, updateCOE: true);
            od.segment.FromRVecSet(r);
            od.segment.DisplayEnabledSet(false);
            return od;
        }


        private bool jobRunning = false;
        private JobHandle jobHandle;

        private LambertJob lamJob;
        private int n = 100;

        private int counter = 0;

        private void LambertJobStart()
        {
            if (!jobRunning && plot2D != null) {
                jobRunning = true;
                lamJob = new LambertJob(n, mu, body1State.r, body2State.r, body1State.v, body2State.v, radius: 0.0, LambertJob.LambertType.TO_TARGET_BYTIME);
                // take times from slider to show the transfer time vs DV
                double dTime = targetPeriod * (slider.maxValue - slider.minValue) / (n - 1);
                double time = slider.minValue * targetPeriod;
                for (int i = 0; i < n; i++) {
                    lamJob.values[i] = time;
                    time += dTime;
                }
                jobHandle = lamJob.Schedule();
                counter = 0;
            }
        }
        public void Update()
        {
            // T to plot TOF vs DV
            if (Input.GetKeyDown(KeyCode.T)) {
                LambertJobStart();
            }
            if (Input.GetKeyDown(KeyCode.X)) {
                // xfer to point (so do not have an arrival v2 target)
                List<GEManeuver> maneuvers = lamOutput.Maneuvers(center.Id(), ship.propagator, intercept: false);
                gsController.GECore().ManeuverListAdd(maneuvers, ship.Id(), 0.0);
                gsController.PausedSet(false);
                // stop displaying the orbits, graph, sliders etc.
                shipOrbit.orbit.gameObject.SetActive(false);
                targetOrbit.orbit.gameObject.SetActive(false);
                plot2D.gameObject.SetActive(false);
                slider.gameObject.SetActive(false);
                sliderText?.gameObject.SetActive(false);
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
                    plot2D.PlotData(points);
                    jobRunning = false;
                    lamJob.Dispose();
                    Debug.Log($"Job completed counter={counter}");
                    Debug.LogFormat("Min DV={0} at index={1}", minDV, minIndex);

                    // force marker update
                    OnSliderValueChanged(slider.value);
                } else {
                    counter++;
                }

            }
        }

    }
}
