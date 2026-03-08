using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;
using Unity.Mathematics;

namespace GravityEngine2 {
    /// <summary>
	/// Display objects and trajectories on XY plot of ascent/launch path.
	///
	/// This display is focused on the ascent of a single GSBody with respect to a planet in a given
	/// orbital plane. 
	///
	/// NOTE: to have the usual Cartesian X, Y plane need to have the camera located at -Z looking
	/// toward the origin, since Unity has a LH coordinate system.
	///
	/// </summary>
    public class GSLaunchDisplay : GSDisplay {

        // note that default plane is 10 x 10 units
        [Header("Downrange Display (Unity units)")]
        public float displayWidth = 100.0f;

        [Header("Height (Unity units)")]
        public float displayHeight = 50.0f;

        [Header("Downrange (World Units)")]
        public float worldWidth;

        [Header("Height (World Units)")]
        public float worldHeight;

        [Header("(optional) Axis Display Line Renderer")]
        public LineRenderer lineR;

        public LineRenderer previewLine;

        public GSBoosterMultiStage booster;
        public GSBody centerBody;

        public Vector3 orbitNormal;

        private Vector3 origin = Vector3.zero;

        private double3 r_center;
        private double3 r_init;
        private double r_mag;

        Vector3 x_axis = Vector3.right;
        Vector3 y_axis = Vector3.up;

        override public List<int> Init()
        {
            // Init is called after GSController has added all bodies
            GECore ge = gsController.GECore();
            GEBodyState centerState = new GEBodyState();
            ge.StateById(centerBody.Id(), ref centerState);
            GEBodyState shipState = new GEBodyState();
            ge.StateById(booster.stageGsBody[booster.numStages - 1].Id(), ref shipState);
            r_center = centerState.r;
            r_init = shipState.r - r_center;
            r_mag = math.length(r_init);
            if (lineR != null) {
                DrawAxes();
            }
            booster.RegisterLaunchPreviewCallback(PreviewSet);

            // Initialize worldWidth and worldHeight to 100 if they are zero
            if (worldWidth == 0) worldWidth = 100f;
            if (worldHeight == 0) worldHeight = 100f;

            return base.Init();
        }

        private void DrawAxes()
        {
            Vector3[] points = new Vector3[] { new Vector3(0, displayHeight, 0),
                                               new Vector3(0, 0, 0),
                                               new Vector3(displayWidth, 0, 0)};
            // fun fact: must set length first
            lineR.positionCount = points.Length;
            lineR.SetPositions(points);
        }

        override
        protected void DisplayUpdateInit(GECore ge, double elapsedWorldTime, double timeOvershoot)
        {
            // need a reference point for the center body world position
            GEBodyState centerState = new GEBodyState();
            ge.StateById(centerBody.Id(), ref centerState);
        }

        private void PreviewSet(GEBodyState[] worldStates, double3 orbitNormal)
        {
            // The preview is in world coordinates, so we need to convert it to the display coordinates
            if (worldStates.Length == 0) {
                previewLine.positionCount = 0;
                return;
            }
            r_init = worldStates[0].r - r_center;
            r_mag = math.length(r_init);
            Vector3[] points = new Vector3[worldStates.Length];
            for (int i = 0; i < worldStates.Length; i++) {
                points[i] = MapToScene(worldStates[i].r, worldStates[i].t);
            }
            previewLine.positionCount = points.Length;
            previewLine.SetPositions(points);
        }

        private (float x, float y) MapToXY(Vector3 rWorldAbs, double time)
        {
            return (0f, 0f);
        }

        /// <summary>
		/// Intended to only be called for the body being launched!
		/// </summary>
		/// <param name="rWorldAbs"></param>
		/// <param name="time"></param>
		/// <returns></returns>
        override
        public Vector3 MapToScene(double3 rWorldAbs, double time)
        {
            Vector3 r_rel = GravityMath.Double3ToVector3(rWorldAbs - r_center);
            Vector3 r_init_vec = GravityMath.Double3ToVector3(r_init).normalized;
            float x = (float)r_mag * Mathf.Deg2Rad * Vector3.Angle(r_rel.normalized, r_init_vec);
            float y = (float)(r_rel.magnitude - r_mag);
            // apply world scaling
            x *= displayWidth / worldWidth;
            y *= displayHeight / worldHeight;
            return new Vector3(x, y, 0) + transform.position;
        }

        /// <summary>
		/// In a map display need to thread the trajectory under the map when we wrap off the
		/// edge (either horizontally or vertically). In this case we need to add more than one
		/// point for a given traj. point, hence the need to override the entire traj. update.
		///
		/// Since this is the only use case, C&P the common code for now.
		/// </summary>
		/// <param name="geTrajectory"></param>
        override
        protected void TrajectoryUpdate(GECore geTrajectory)
        {
            int size = (int)geTrajectory.GetParm(GECore.TRAJ_NUMSTEPS_PARM);
            int start = ((int)geTrajectory.GetParm(GECore.TRAJ_INDEX_PARAM) + 1) % size;
            // TRAJ_INDEX_PARAM points to last entry written to
            NativeArray<GEBodyState> recordedOutput = geTrajectory.RecordedOutputGEUnits();
            NativeArray<int> recordedBodies = geTrajectory.RecordedBodies();
            int stride = recordedBodies.Length;
            // for now fill the entire LR array each time. Would have to do a shuffle anyway...
            int maxLoops = 5; // max number of threads under map
            Vector3[] points = new Vector3[size + maxLoops * 3];
            Vector3 r_vec;
            float rScaleToWorld = (float)geTrajectory.GEScaler().ScaleLenGEToWorld(1.0);
            float tScaleToWorld = (float)geTrajectory.GEScaler().ScaleTimeGEToWorld(1.0);
            // TODO: Greedy. Better to keep a list of those objects that want trajectories
            for (int ii = 0; ii < numDO; ii++) {
                int i = displayIndices[ii];
                if (displayObjects[i].trajectory) {
                    int rbIndex = recordedBodies.IndexOf(displayObjects[i].bodyId);
                    if (rbIndex < 0) {
                        // not sure why this happens when adding each subsequent traj. 
                        // Debug.LogWarning("No index for body " + displayObjects[i].bodyId);
                        continue;
                    }
                    // p is wrap-around index to read from circular buffer
                    int p = start;
                    int pCount = 0;
                    for (int j = 0; j < size - 1; j++) {
                        r_vec = rScaleToWorld * recordedOutput[rbIndex + p * stride].RasVector3();
                        double t = tScaleToWorld * recordedOutput[rbIndex + p * stride].t;
                        (float x, float y) = MapToXY(r_vec, t);

                        points[pCount] = x * x_axis + y * y_axis + origin;
                        pCount++;
                        p = (p + 1) % size;
                    }
                    // fill in the line renderer
                    displayObjects[i].trajLine.positionCount = pCount - 1;
                    displayObjects[i].trajLine.SetPositions(points);
                }
            }
        }

    }
}
