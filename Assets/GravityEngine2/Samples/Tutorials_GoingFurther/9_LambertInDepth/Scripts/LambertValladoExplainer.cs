using TMPro;
using UnityEngine;

namespace GravityEngine2 {
    public class LambertValladoExplainer : MonoBehaviour {

        public GSBody body1;
        public GSBody body2;
        public GSBody center;

        public GSDisplay gsDisplay;

        public GSController gsController;

        public GameObject orbitPrefab;

        public double xferTime = 20.0;

        public TextMeshProUGUI text;

        private Color activeColor = Color.red;

        private Color inactiveColor = Color.gray;

        private float inactiveWidth = 0.1f;
        private float activeWidth = 0.2f;

        private struct OrbitStep {
            public Lambert.LambertOutput lamOutput;
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
                index = (index + 1) % orbitSteps.Length;
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
                        if (text != null) {
                            text.text = string.Format("{0} {1} nrev={2}", scenarios[index].dm, scenarios[index].de, scenarios[index].nrev);
                        }
                    }
                }
            }
        }

        private struct ScenarioInfo {
            public Lambert.MotionDirection dm;
            public Lambert.EnergyType de;
            public int nrev;

            public ScenarioInfo(Lambert.MotionDirection dm, Lambert.EnergyType de, int nrev)
            {
                this.dm = dm;
                this.de = de;
                this.nrev = nrev;
            }
        }

        ScenarioInfo[] scenarios = new ScenarioInfo[] {
                new ScenarioInfo(Lambert.MotionDirection.SHORT, Lambert.EnergyType.LOW, 0),
                new ScenarioInfo(Lambert.MotionDirection.LONG, Lambert.EnergyType.LOW, 0),
            };

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

            orbitSteps = new OrbitStep[scenarios.Length];

            for (int i = 0; i < scenarios.Length; i++) {
                double tmin, tminp, tminenergy;
                (tmin, tminp, tminenergy) = Lambert.MinTimes(mu, body1State.r, body2State.r, scenarios[i].dm, scenarios[i].nrev);
                Debug.LogFormat("dm={0} nrev={1} tmin={2} tminp={3} tminenergy={4}", scenarios[i].dm, scenarios[i].nrev, tmin, tminp, tminenergy);

                orbitSteps[i] = new OrbitStep();
                GameObject go = Instantiate(orbitPrefab);
                orbitSteps[i].gameObject = go;

                orbitSteps[i].renders = go.GetComponentsInChildren<Renderer>();
                orbitSteps[i].lineRenderers = go.GetComponentsInChildren<LineRenderer>();
                // get the min transfer time
                // double t_min = transferShip.GetMinTime();
                // Debug.LogFormat("Min transfer time: {0} days", t_min / 86400.0);
                orbitSteps[i].lamOutput = Lambert.Universal(mu, body1State.r, body2State.r, body1State.v,
                                scenarios[i].dm, scenarios[i].de, scenarios[i].nrev,
                                xferTime, 0.0, 0.0);
                if (orbitSteps[i].lamOutput.status == Lambert.Status.OK) {

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
                    InactivateOrbit(i);
                    // Log the xfer orbit
                    Orbital.COE xferOrbit = Orbital.RVtoCOE(body1State.r, orbitSteps[i].lamOutput.v1t, mu);
                    Debug.LogFormat("Xfer orbit: {0}", xferOrbit.LogStringDegrees());


                } else {
                    Debug.LogErrorFormat("LambertVallado.LambertUniv failed error={0}", orbitSteps[i].lamOutput.status);
                }

            }
        }


    }
}
