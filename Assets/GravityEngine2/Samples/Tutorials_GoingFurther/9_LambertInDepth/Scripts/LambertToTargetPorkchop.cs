using System.Collections.Generic;
using TMPro;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace GravityEngine2 {
    public class LambertToTargetPorkchop : MonoBehaviour {

        public GSBody ship;
        public GSBody target;
        public GSBody center;

        public GSDisplay gsDisplay;

        private GSController gsController;

        public GameObject orbitPrefab;

        [Header("Minimum Transfer Duration As Factor of Ship Orbit Period")]
        public double minXferDurationFactor = 0.25;

        [Header("Maximum Transfer Duration As Factor of Ship Orbit Period")]
        public double maxXferDurationFactor = 0.75;

        [Header("Depart Time as fraction of Ship Orbit Period")]
        public double tDepart_startFactor = 0.0;
        public double tDepart_endFactor = 1.0;
        public int tDepart_steps = 100;

        [Header("Arrival Time as fraction of Ship Orbit Period")]
        public double tArrive_startFactor = 0.3;
        public double tArrive_endFactor = 1.0;
        public int tArrive_steps = 100;

        [Header("Transfer Details")]
        public TextMeshProUGUI orbitDetailsText;

        public enum PlotMode {
            DEPART_VEL,
            DV_DEPART,
            C3_DEPART,
            DV_TOTAL
        }
        public PlotMode dvMode = PlotMode.DV_DEPART;

        public DisplayFunctionAsMesh displayMesh;

        [Header("Max Plot (as multiple of min value)")]
        public float maxPlotFactor = 4.0f;

        [Header("X to init clicked xfer")]
        public bool useKeys = true;

        public bool porkchopAtStart = true;

        private class OrbitDisplay {
            public GSDisplayOrbit orbit;
            public GSDisplayOrbitSegment segment;
        }

        private OrbitDisplay shipOrbit;
        private OrbitDisplay targetOrbit;

        private GEBodyState body1State;
        private GEBodyState body2State;
        private double mu;

        private KeplerPropagator.RVT targetProp;
        private KeplerPropagator.RVT shipProp;

        private LambertPorkchop lp;

        private double shipPeriod;

        // Start is called before the first frame update
        void Start()
        {
            gsController = gsDisplay.gsController;
            if (porkchopAtStart) {
                gsController.ControllerStartedCallbackAdd(SetupOrbits);
            }
        }


        private double DV(Lambert.LambertOutput lamOutput)
        {
            switch (dvMode) {
                case PlotMode.DV_DEPART:
                    return math.length(lamOutput.v1t - lamOutput.v1);
                case PlotMode.DEPART_VEL:
                    return math.length(lamOutput.v1t);
                case PlotMode.C3_DEPART:
                    double dv = math.length(lamOutput.v1t - lamOutput.v1);
                    return dv * dv;
                case PlotMode.DV_TOTAL:
                    return math.length(lamOutput.v1t - lamOutput.v1) + math.length(lamOutput.v2t - lamOutput.v2);
            }
            return 0.0;
        }

        private Lambert.LambertOutput lastLamOutput;

        private void ComputeTransfer(double tDepart, double tArrive)
        {
            if (shipOrbit == null) {
                shipOrbit = OrbitDisplayCreate(body1State.r, body1State.v);
                shipOrbit.orbit.gameObject.name = "Ship Orbit";
            }
            if (targetOrbit == null) {
                targetOrbit = OrbitDisplayCreate(body2State.r, body2State.v);
                targetOrbit.orbit.gameObject.name = "Target Orbit";
            }
            // determine depart position and velocity
            (int status, double3 r1, double3 v1) = KeplerPropagator.RVforTime(shipProp, tDepart);
            if (status != 0) {
                Debug.LogErrorFormat("Error getting ship state at time {0}", tDepart);
                return;
            }
            // determine where the target will be at the time of arrival and 
            // then determine the transfer to get to that point in the given time. 
            (int status2, double3 r2, double3 v2) = KeplerPropagator.RVforTime(targetProp, tArrive);
            if (status2 != 0) {
                Debug.LogErrorFormat("Error getting target state at time {0}", tArrive);
                return;
            }
            double tXfer = tArrive - tDepart;
            lastLamOutput = Lambert.TransferProgradeToPoint(mu, r1, r2, v1, tXfer, radius: 0.0, prograde: true);
            if (lastLamOutput.status != Lambert.Status.OK) {
                Debug.LogErrorFormat("Error computing Lambert transfer err={0}", lastLamOutput.status);
                shipOrbit.segment.DisplayEnabledSet(false);
                targetOrbit.segment.DisplayEnabledSet(false);
                return;
            }
            lastLamOutput.v2 = v2;
            lastLamOutput.t0 = tDepart;
            Debug.LogFormat("Transfer time={0} days={1} dV={2} tDepart={3} tArrive={4}", tXfer, tXfer / 86400.0, DV(lastLamOutput), tDepart, tArrive);
            // display the propagation of the target using the orbit segment
            targetOrbit.segment.ToRVecSet(r2);
            targetOrbit.segment.DisplayEnabledSet(true);

            // set the ship orbit to the transfer point
            shipOrbit.orbit.RVRelativeSet(r1, lastLamOutput.v1t, updateCOE: true);
            shipOrbit.segment.RVRelativeSet(r1, lastLamOutput.v1t, updateCOE: true);
            shipOrbit.segment.FromRVecSet(r1);
            shipOrbit.segment.ToRVecSet(r2);
            shipOrbit.segment.DisplayEnabledSet(true);

            if (orbitDetailsText != null) {
                float scale = displayMesh.displayTimeScale == DisplayFunctionAsMesh.DisplayTimeScale.SECONDS ? 1.0f : (float)GBUnits.SECS_PER_SOLAR_DAY;
                string units = displayMesh.displayTimeScale == DisplayFunctionAsMesh.DisplayTimeScale.SECONDS ? "s" : "days";
                orbitDetailsText.text = $" t_depart={tDepart / scale:F3} {units}\n t_arrive={tArrive / scale:F3} {units}\n t_xfer={tXfer / scale:F3} {units}\n DV={DV(lastLamOutput):F3}";
                Debug.Log(orbitDetailsText.text);
            }

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

            shipPeriod = ge.COE(ship.Id(), center.Id(), false).GetPeriod();
            shipProp = new KeplerPropagator.RVT(body1State.r, body1State.v, t0: 0.0, mu);

            targetProp = new KeplerPropagator.RVT(body2State.r, body2State.v, t0: 0.0, mu);

            // compute the porkchop plot
            lp = new LambertPorkchop() {
                r1 = body1State.r,
                v1 = body1State.v,
                r2 = body2State.r,
                v2 = body2State.v,
                mu = mu,
                tXfer_min = minXferDurationFactor * shipPeriod,
                tXfer_max = maxXferDurationFactor * shipPeriod,
                tDepart_start = tDepart_startFactor * shipPeriod,
                tDepart_end = tDepart_endFactor * shipPeriod,
                tArrive_start = tArrive_startFactor * shipPeriod,
                tArrive_end = tArrive_endFactor * shipPeriod,
                tDepart_steps = tDepart_steps,
                tArrive_steps = tArrive_steps,
                radius = 0.0
            };
            lp.Init();
            double jd = gsController.JDTime();
            Debug.Log($"Ship period = {shipPeriod / GBUnits.SECS_PER_SOLAR_DAY} days {shipPeriod} seconds");
            Debug.Log($"Depart time={TimeUtils.JDTimeFormattedYMDhms(jd + lp.tDepart_start / GBUnits.SECS_PER_SOLAR_DAY)}...{TimeUtils.JDTimeFormattedYMDhms(jd + lp.tDepart_end / GBUnits.SECS_PER_SOLAR_DAY)}");
            Debug.Log($"Arrival time={TimeUtils.JDTimeFormattedYMDhms(jd + lp.tArrive_start / GBUnits.SECS_PER_SOLAR_DAY)}...{TimeUtils.JDTimeFormattedYMDhms(jd + lp.tArrive_end / GBUnits.SECS_PER_SOLAR_DAY)}");
            jobHandle = lp.Schedule();
            jobRunning = true;
            Debug.LogFormat("Porkchop plot started");
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
        private int counter = 0;

        private void GridClicked(float x, float y)
        {
            Debug.LogFormat("GridClicked x={0} y={1}", x, y);
            // x is departure time and y is the arrival time
            ComputeTransfer(x, y);
        }

        public void Update()
        {

            if (useKeys && !porkchopAtStart && Input.GetKeyDown(KeyCode.P)) {
                SetupOrbits(gsController);
            }
            if (useKeys && Input.GetKeyDown(KeyCode.X) && !jobRunning) {
                List<GEManeuver> maneuvers = lastLamOutput.Maneuvers(center.Id(), ship.propagator, intercept: false);
                gsController.GECore().ManeuverListAdd(maneuvers, ship.Id(), lastLamOutput.t0 + gsController.GECore().TimeWorld());
                gsController.PausedSet(false);
                shipOrbit.orbit.gameObject.SetActive(false);
                shipOrbit.orbit.DisplayEnabledSet(false);
                shipOrbit.segment.DisplayEnabledSet(false);
                targetOrbit.orbit.gameObject.SetActive(false);
                targetOrbit.orbit.DisplayEnabledSet(false);
                targetOrbit.segment.DisplayEnabledSet(false);
                displayMesh.gameObject.SetActive(false);
                Debug.Log("Added maneuvers");
            }

            if (jobRunning) {
                double minDV = double.MaxValue;
                double dV;
                int minIndex = 0;
                if (jobHandle.IsCompleted) {
                    jobHandle.Complete();
                    Debug.Log("Completed job");
                    // Fill in a data array and call GenerateFromSparseData
                    // x = departure time, y = arrival time (departure time + transfer time)
                    float3[] data = new float3[lp.outputs.Length];
                    for (int i = 0; i < lp.outputs.Length; i++) {
                        dV = DV(lp.outputs[i]);
                        data[i] = new float3((float)lp.outputs[i].t0,
                                             (float)(lp.outputs[i].t0 + lp.outputs[i].t),
                                             (float)dV);
                        // Debug.LogFormat("i={0} x={1} y={2} z={3}", i, data[i].x, data[i].y, data[i].z);
                        if (dV < minDV) {
                            minDV = dV;
                            minIndex = i;
                        }
                    }
                    displayMesh.GenerateFromSparseData(data, lp.tDepart_steps, lp.tArrive_steps, GridClicked, zLimit: (float)(minDV * maxPlotFactor));
                    ComputeTransfer(lp.outputs[minIndex].t0, lp.outputs[minIndex].t0 + lp.outputs[minIndex].t);
                    Debug.LogFormat("***Porkchop DONE Min DV={0} at index={1} frames={2}", minDV, minIndex, counter);
                    jobRunning = false;
                    lp.Dispose();

                } else {
                    counter++;
                    Debug.LogFormat("***Porkchop NOT DONE counter={0}", counter);
                }
            }
        }

    }
}
