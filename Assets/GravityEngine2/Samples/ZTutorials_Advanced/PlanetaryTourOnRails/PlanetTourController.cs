using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;


namespace GravityEngine2 {
    /// <summary>
    /// Do a preview plot of the path of a ship that exits planetA's SOI and (may) intersect other planets SOIs.
    ///
    /// The ship trajectory is kept simple - just extend the velocity at the time P is pressed by some percentage and
    /// maintain the current velocity direction. This can be iterated by increasing/decreasing the percentage by A/D keys.
    ///
    /// The idea is to make use of OrbitPropagators for the ship and planets and evolve them forward in lockstep and then
    /// determine when SOI boundaries are crossed. When this happens the ship propagator is switched to the new primary
    /// gravitational source. This is a simpler implementation that e.g. cloning the GravityState and evolving because
    /// the ship would need a series of KeplerSequence segments and there would be a lot of "moving parts".
    ///
    /// A line renderer is used to show the potential path. It hold the absolute positions in the solar system space.
    ///
    /// The plot routine also draws an SOI circle at each enter and exit point (since as the ship moves the planets are
    /// moving).
    /// 
    /// </summary>
    public class PlanetTourController : MonoBehaviour {

        public GSDisplay gsDisplay;

        [Header("A/S to alter burn.")]
        public GSBody ship;

        [Header("Star")]
        public GSBody star;

        [Header("Planets")]
        public GSBody[] planets;

        [Header("Duration of path plot (world time)")]
        public double tFinal = 10.0;

        [Header("Escape burn as a percent of orbital velocity")]
        public double escapeBurnDv = 1.5;
        public double dVStepPercent = 0.05;

        [Header("Line for planet tour")]
        public LineRenderer lineR;
        public double lineDt = 0.1;

        private bool plotAtStart = true;

        public GameObject circlePrefab;

        private GSController gsController;

        /// <summary>
        /// What sphere of influence is the ship currently in?
        /// 0..N-1 Planet 1..N
        /// -1 = star (i.e. in between planet SOIs)
        /// </summary>
        private int activeSphere = 0;   // start in orbit around planet A
        private const int SUN_SOI = -1;
        private double[] soiRadius;

        private GECore ge;

        // struct to store orbit segments of preview path so they can be added to Kseq on execute
        private struct OrbitSegment {
            public double3 r;
            public double3 v;
            public double t;
            public double mu;
            public GSBody center;

            public OrbitSegment(double3 r, double3 v, double t, double mu, GSBody center)
            {
                this.r = r;
                this.v = v;
                this.t = t;
                this.mu = mu;
                this.center = center;
            }
        }
        private List<OrbitSegment> orbitSegments;
        private List<GameObject> circles;

        // Start is called before the first frame update
        void Start()
        {
            gsController = gsDisplay.gsController;
            gsController.ControllerStartedCallbackAdd(GEStarted);
            circles = new List<GameObject>();
        }

        private void GEStarted(GSController gsc)
        {
            ge = gsController.GECore();
            soiRadius = new double[planets.Length];
            double starMass = ge.MassWorld(star.Id());
            for (int i = 0; i < planets.Length; i++) {
                double moonMass = ge.MassWorld(planets[i].Id());
                Orbital.COE coe = ge.COE(planets[i].Id(), star.Id());
                soiRadius[i] = Orbital.SoiRadius(starMass, moonMass, coe.a);
                Debug.LogFormat("SOI radius for {0} is {1}", planets[i].gameObject.name, soiRadius[i]);
            }
        }

        // Update is called once per frame
        void Update()
        {
            if (plotAtStart) {
                gsController.PausedSet(true);
                PlotPath();
                plotAtStart = false;
            }

            if (Input.GetKeyDown(KeyCode.A)) {
                escapeBurnDv += dVStepPercent;
                PlotPath();
            }
            if (Input.GetKeyDown(KeyCode.S)) {
                escapeBurnDv -= dVStepPercent;
                PlotPath();
            }
            if (Input.GetKeyDown(KeyCode.X)) {
                ExecutePath();
                gsController.PausedSet(false);
            }
        }


        private void PlotPath()
        {
            activeSphere = 0;
            // clear any prevous SOIs
            foreach (GameObject go in circles) {
                Destroy(go);
            }
            circles.Clear();
            // create an Orbit Propator to exit PlanetA. Need relative R, V
            GEBodyState shipState = new GEBodyState();
            ge.StateById(ship.Id(), ref shipState);
            GEBodyState planetState = new GEBodyState();
            ge.StateById(planets[activeSphere].Id(), ref planetState);
            double3 v0 = shipState.v - planetState.v;
            v0 *= escapeBurnDv;
            double3 r0 = shipState.r - planetState.r;
            EvolveToTime(r0, v0);
        }

        private void ExecutePath()
        {
            GEBodyState bodyState = new GEBodyState();
            foreach (OrbitSegment segment in orbitSegments) {
                bodyState.r = segment.r;
                bodyState.v = segment.v;
                ge.PatchCreateAndAdd(ship.Id(), segment.center.Id(), bodyState, GEPhysicsCore.Propagator.KEPLER, segment.t);
            }
            lineR.positionCount = 0;
            lineR.enabled = false;
            foreach (GameObject go in circles) {
                Destroy(go);
            }
            circles.Clear();
        }


        private const int SOI_DEBOUNCE = 5; // min number of time steps a ship must stay in an SOI

        private double MuBody(int activeId)
        {
            if (activeId == SUN_SOI)
                return ge.MassWorld(star.Id());
            else
                return ge.MassWorld(planets[activeId].Id());
        }

        /// <summary>
        /// Propagate the ship and planets forward in time and at each timeStep check to see if an SOI transition has happened
        /// (modulo some debounce after an SOI change).
        ///
        /// If an SOI change has occured, update the propagator for the new regime.
        ///
        /// At the same time, store up orbital segments so they can be added to the ship Kseq if the user decides to
        /// execute. 
        /// 
        /// </summary>
        /// <param name="r0"></param>
        /// <param name="v0"></param>
        private void EvolveToTime(double3 r0, double3 v0)
        {
            int n = (int)(tFinal / lineDt) + 1;
            double tEvolved = 0.0;
            Vector3[] points = new Vector3[n];
            int pIndex = 0;
            int soiDebounce = SOI_DEBOUNCE;
            orbitSegments = new List<OrbitSegment>();
            int nPlanets = planets.Length;

            GEBodyState[] planetState = new GEBodyState[nPlanets];

            KeplerPropagator.RVT shipProp = new KeplerPropagator.RVT(r0, v0, 0.0, MuBody(activeSphere));
            orbitSegments.Add(new OrbitSegment(r0, v0, ge.TimeWorld(), MuBody(activeSphere), planets[activeSphere]));
            while (tEvolved < tFinal) {
                // get the planet states
                for (int i = 0; i < nPlanets; i++) {
                    ge.StateByIdAtTime(planets[i].Id(), tEvolved, ref planetState[i]);
                }
                tEvolved += lineDt;
                (int status, double3 r, double3 v) = KeplerPropagator.RVforTime(shipProp, tEvolved);
                points[pIndex] = gsDisplay.MapToScene(r);

                if (activeSphere != SUN_SOI)
                    points[pIndex] += gsDisplay.MapToScene(planetState[activeSphere].r);
                pIndex++;
                // check to see if we cross the SOI
                if (soiDebounce++ > SOI_DEBOUNCE) {
                    if (activeSphere == SUN_SOI) {
                        // need to check if we enter ANY of the planet SOIs
                        for (int i = 0; i < planets.Length; i++) {
                            if (math.length(r - planetState[i].r) < soiRadius[i]) {
                                // ENTERED planet SOI
                                DrawCircle(GravityMath.Double3ToVector3(planetState[i].r), (float)soiRadius[i]);
                                // entered planet SOI, move to relative r, v
                                r = r - planetState[i].r;
                                v = v - planetState[i].v;
                                double mu_seg = ge.MuWorld(planets[i].Id());
                                shipProp = new KeplerPropagator.RVT(r, v, tEvolved, mu_seg);
                                orbitSegments.Add(new OrbitSegment(r, v, tEvolved, mu_seg, planets[i]));
                                activeSphere = i;
                                Debug.LogFormat("Enter SOI {0} at {1} ", planets[i].gameObject.name, tEvolved);
                                soiDebounce = 0;

                                break;
                            }
                        }
                    } else {
                        // r is relative to planet
                        if (math.length(r) > soiRadius[activeSphere]) {
                            // EXITED planet SOI
                            DrawCircle(GravityMath.Double3ToVector3(planetState[activeSphere].r), (float)soiRadius[activeSphere]);
                            v = v + planetState[activeSphere].v;
                            r = r + planetState[activeSphere].r;
                            activeSphere = SUN_SOI;
                            shipProp = new KeplerPropagator.RVT(r, v, tEvolved, MuBody(activeSphere));
                            orbitSegments.Add(new OrbitSegment(r, v, tEvolved, MuBody(activeSphere), star));
                            Debug.LogFormat("Enter Sun SOI at {0}", tEvolved);
                            soiDebounce = 0;
                        }
                    }
                }
            }
            lineR.positionCount = pIndex;
            lineR.SetPositions(points);
        }

        /// <summary>
        /// Very simple & dumb code to draw a circle
        /// </summary>
        /// <param name="center"></param>
        /// <param name="radius"></param>
        private void DrawCircle(Vector3 center, float radius)
        {
            Debug.LogFormat("DrawCircle {0} {1}", center, radius);
            Vector3[] points = new Vector3[360];
            GameObject go = Instantiate(circlePrefab);
            LineRenderer line = go.GetComponent<LineRenderer>();
            float theta;
            for (int i = 0; i < 360; i++) {
                theta = (float)(i) * Mathf.Deg2Rad;
                double3 p = new double3(center.x + radius * Mathf.Cos(theta), center.y + radius * Mathf.Sin(theta), 0);
                points[i] = gsDisplay.MapToScene(p);
            }
            line.positionCount = 360;
            line.useWorldSpace = true;
            line.loop = true;
            line.SetPositions(points);
            circles.Add(go);
        }
    }

}


