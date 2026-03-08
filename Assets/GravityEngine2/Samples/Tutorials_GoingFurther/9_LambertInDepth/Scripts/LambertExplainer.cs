using System.Collections;
using System.Collections.Generic;
using TMPro;
using Unity.Mathematics;
using UnityEngine;

namespace GravityEngine2 {
    public class LambertExplainer : MonoBehaviour {

        [Header("Uses TransferShip")]
        public GSBody body1;
        public GSBody body2;
        public GSBody center;

        public GSDisplay gsDisplay;

        public GSController gsController;

        public GSDisplayOrbit gsDisplayOrbit;

        public GSDisplayOrbitSegment gsdSegment;

        public float fromTime = 0.9f;

        public float toTime = 1.1f;

        public int numSteps = 10;

        public bool retro;

        public TextMeshProUGUI text;

        private Color activeColor = Color.red;

        private Color inactiveColor = Color.black;
        private float inactiveWidth = 0.1f;
        private float activeWidth = 0.2f;

        private List<GEManeuver>[] maneuverLists;
        private struct OrbitStep {
            public List<GEManeuver> maneuvers;
            public Renderer[] renders;

            public LineRenderer[] lineRenderers;
            public GameObject gameObject;
        }

        private OrbitStep[] orbitSteps;

        // Start is called before the first frame update
        void Start()
        {
            gsController.ControllerStartedCallbackAdd(TransferShipOrbits);
        }


        private int index = 0;
        // Update is called once per frame

        private void InactivateOrbit(int index)
        {
            if (orbitSteps[index].gameObject != null) {
                foreach (Renderer r in orbitSteps[index].renders) {
                    r.material.color = inactiveColor;
                }
                foreach (LineRenderer lr in orbitSteps[index].lineRenderers) {
                    lr.startWidth = inactiveWidth;
                    lr.endWidth = inactiveWidth;
                }
            }
        }
        void Update()
        {
            if (Input.GetKeyDown(KeyCode.N)) {
                InactivateOrbit(index);
                index++;
                if (index >= orbitSteps.Length) {
                    index = 0;
                }
                if (orbitSteps[index].gameObject != null) {
                    foreach (Renderer r in orbitSteps[index].renders) {
                        r.material.color = activeColor;
                    }
                    foreach (LineRenderer lr in orbitSteps[index].lineRenderers) {
                        lr.startWidth = activeWidth;
                        lr.endWidth = activeWidth;
                        if (lr.GetComponent<GSDisplayOrbitSegment>() != null) {
                            lr.startWidth = 4 * activeWidth;
                            lr.endWidth = 4 * activeWidth;
                        }
                        double dV = math.length(maneuverLists[index][0].dV);
                        if (index != 0) {
                            text.text = string.Format("t={0:##.###} dV={1:##.###}", maneuverLists[index][1].t_relative, dV);
                        } else {
                            text.text = string.Format("Min Energy Orbit\n t={0:##.###} dV={1:##.###}", maneuverLists[index][1].t_relative, dV);
                        }
                    }
                }
            }
        }

        private void TransferShipOrbits(GSController controller)
        {
            Debug.Log("LambertOrbits - Pausing GSController");
            gsController.PausedSet(true);
            // create a TransferShip object
            GECore ge = controller.GECore();
            GEBodyState body1State = new GEBodyState();
            bool stateOk = ge.StateById(body1.Id(), ref body1State);
            GEBodyState body2State = new GEBodyState();
            stateOk = ge.StateById(body2.Id(), ref body2State);
            double mu = ge.MuWorld(center.Id());
            Orbital.COE toOrbit = Orbital.RVtoCOE(body1State.r, body1State.v, mu);
            TransferShip transferShip = new TransferShip(body1State, toOrbit, mu, retrograde: retro);
            transferShip.SetTargetPoint(body2State.r);
            transferShip.lambertAlways = true;
            transferShip.SetTimeFactor(1.0);
            maneuverLists = new List<GEManeuver>[numSteps + 2];
            orbitSteps = new OrbitStep[numSteps + 2];
            orbitSteps[0] = new OrbitStep();
            orbitSteps[0].gameObject = gsDisplayOrbit.gameObject;
            orbitSteps[0].renders = gsDisplayOrbit.GetComponentsInChildren<Renderer>();
            orbitSteps[0].lineRenderers = gsDisplayOrbit.GetComponentsInChildren<LineRenderer>();
            // get the min transfer time
            // double t_min = transferShip.GetMinTime();
            // Debug.LogFormat("Min transfer time: {0} days", t_min / 86400.0);
            TransferShip.Status status = transferShip.Compute(TransferShip.TargetMode.SHIP_TO_POINT);
            if (status == TransferShip.Status.OK) {
                // the minimum energy transfer time is the last maneuver time
                maneuverLists[0] = transferShip.Maneuvers();
                double t_min = maneuverLists[0][maneuverLists[0].Count - 1].t_relative;
                text.text = string.Format("Min Energy Orbit\n t={0} dV={1:##.###}", t_min, math.length(maneuverLists[0][0].dV));

                Debug.LogFormat("Min transfer time: {0} r={1} v={2}", t_min, body1State.r, maneuverLists[0][0].velocityParam);
                // set the display orbit to show this transfer
                if (gsDisplayOrbit != null) {
                    gsDisplayOrbit.RVRelativeSet(body1State.r, maneuverLists[0][0].velocityParam, updateCOE: true);
                    gsDisplayOrbit.DisplayEnabledSet(true);
                }
                if (gsdSegment != null) {
                    gsdSegment.RVRelativeSet(body1State.r, maneuverLists[0][0].velocityParam, updateCOE: true);
                    gsdSegment.FromRVecSet(body1State.r);
                    gsdSegment.ToRVecSet(body2State.r);
                    gsdSegment.DisplayEnabledSet(true);
                }
                // Log the xfer orbit
                Orbital.COE xferOrbit = Orbital.RVtoCOE(body1State.r, maneuverLists[0][0].velocityParam, mu);
                Debug.LogFormat("Xfer orbit: {0}", xferOrbit.LogStringDegrees());

                // Now determine the transfer time for a series of steps
                double dt = (toTime - fromTime) / numSteps;
                for (int i = 0; i <= numSteps; i++) {
                    double t = fromTime + i * dt;
                    transferShip.SetTimeFactor(t);
                    status = transferShip.Compute(TransferShip.TargetMode.SHIP_TO_POINT);
                    if (status == TransferShip.Status.OK) {
                        maneuverLists[i + 1] = transferShip.Maneuvers();
                        // Debug.LogFormat("t={0} v={1}", t, maneuverLists[i + 1][0].velocityParam);
                        GameObject orbitStep = Instantiate(gsDisplayOrbit.gameObject);
                        orbitSteps[i + 1] = new OrbitStep();
                        orbitSteps[i + 1].gameObject = orbitStep;
                        orbitSteps[i + 1].renders = orbitStep.GetComponentsInChildren<Renderer>();
                        orbitSteps[i + 1].lineRenderers = orbitStep.GetComponentsInChildren<LineRenderer>();
                        foreach (Renderer r in orbitSteps[i + 1].renders) {
                            r.material.color = inactiveColor;
                        }
                        orbitStep.transform.parent = gsDisplay.transform;

                        GSDisplayOrbit gsdo = orbitStep.GetComponent<GSDisplayOrbit>();
                        gsDisplay.DisplayObjectAdd(gsdo);
                        gsdo.RVRelativeSet(body1State.r, maneuverLists[i + 1][0].velocityParam, updateCOE: true);
                        gsdo.DisplayEnabledSet(true);

                        GSDisplayOrbitSegment gsdos = orbitStep.GetComponentInChildren<GSDisplayOrbitSegment>();
                        gsDisplay.DisplayObjectAdd(gsdos);
                        gsdos.RVRelativeSet(body1State.r, maneuverLists[i + 1][0].velocityParam, updateCOE: true);
                        gsdos.FromRVecSet(body1State.r);
                        gsdos.ToRVecSet(body2State.r);
                        gsdos.DisplayEnabledSet(true);
                        InactivateOrbit(i + 1);
                    }
                }

            } else {
                Debug.LogErrorFormat("TransferShip.Compute failed error={0}", status);
            }
        }
    }


}
