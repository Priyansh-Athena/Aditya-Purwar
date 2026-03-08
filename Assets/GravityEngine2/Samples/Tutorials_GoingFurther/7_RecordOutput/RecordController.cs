using Unity.Jobs;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;

namespace GravityEngine2 {
    /// <summary>
    /// Simple test to evolve using record output at regular intervals and then 
    /// use a series of line renderers to draw the paths. 
    /// </summary>
    public class RecordController : MonoBehaviour {
        public double t_end = 50.0;
        public int numPoints = 100;

        public bool asJob = false;

        [Header("Need three bodies")]
        public LineRenderer[] lines;

        private GECore ge;
        private double[] timePoints;
        private int[] bodies;
        private JobHandle jobHandle;

        private bool done;

        // Start is called before the first frame update
        void Start()
        {
            // use default dt and scale
            double mu = 1000.0;
            ge = new GECore(integrator: Integrators.Type.RK4,
                                                  defaultUnits: GBUnits.Units.DL,
                                                  lengthScale: 100.0,
                                                  massScale: mu,
                                                  stepsPerOrbit: 1000.0);
            int centerId = ge.BodyAdd(rWorld: double3.zero, vWorld: double3.zero, massWorld: mu, isFixed: true);
            // now add three orbiting objects
            int[] id = new int[3];
            Orbital.COE coe = new Orbital.COE();
            coe.EllipseInitDegrees(mu, a: 100, e: 0.4, i: 0, omegaL: 0, omegaU: 0, nu: 0);
            GEPhysicsCore.Propagator propGravity = GEPhysicsCore.Propagator.GRAVITY;
            id[0] = ge.BodyAddInOrbitWithCOE(coe, centerId, propGravity);
            coe.EllipseInitDegrees(mu, a: 100, e: 0.4, i: 0, omegaL: 90, omegaU: 0, nu: 0);
            id[1] = ge.BodyAddInOrbitWithCOE(coe, centerId, propGravity);
            coe.EllipseInitDegrees(mu, a: 100, e: 0.4, i: 0, omegaL: 180, omegaU: 0, nu: 0);
            id[2] = ge.BodyAddInOrbitWithCOE(coe, centerId, propGravity);

            // setup the time points (in world time) that we wish to record state for
            // (here we do equally spaced points)
            timePoints = new double[numPoints];
            double t = 0.0;
            double dt = t_end / (double)numPoints;
            for (int i = 0; i < numPoints; i++) {
                timePoints[i] = t;
                t += dt;
            }
            // create a list of the body Ids of the bodies we want to record state info for
            bodies = new int[3];
            for (int i = 0; i < 3; i++)
                bodies[i] = id[i];
            // Now evolve these bodies, either in job mode or immediatly
            if (asJob) {
                ge.ScheduleRecordOutput(t_end, timePoints, bodies);
            } else {
                ge.EvolveNowRecordOutput(t_end, timePoints, bodies);
                CheckOutput();
            }

        }

        private void CheckOutput()
        {
            // Now get the result and transfer to line renderers
            int line = 0;
            foreach (int bodyId in bodies) {
                GEBodyState[] recordedOutput = ge.RecordedOutputForBody(bodyId, worldUnits: true);
                Debug.LogFormat("Check output {0} ", recordedOutput.Length);

                Vector3[] points = new Vector3[numPoints];

                lines[line].positionCount = numPoints;
                for (int j = 0; j < numPoints; j++) {
                    points[j] = new Vector3((float)recordedOutput[j].r.x, (float)recordedOutput[j].r.y, (float)recordedOutput[j].r.z);
                }
                lines[line].SetPositions(points);
                line++;
            }
            ge.Dispose();
        }

        // Update is called once per frame
        void Update()
        {
            if (asJob && !done) {
                if (ge.IsCompleted()) {
                    ge.Complete();
                    CheckOutput();
                    done = true;
                } else {
                    Debug.Log("Waiting for job " + Time.time);
                }
            }
        }
    }
}
