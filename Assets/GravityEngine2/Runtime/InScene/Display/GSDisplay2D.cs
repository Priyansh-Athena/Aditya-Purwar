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
    public class GSDisplay2D : GSDisplay {

        // note that default plane is 10 x 10 units
        [Header("Downrange (default units)")]
        public float width = 100.0f;

        [Header("Height (default units)")]
        public float height = 50.0f;

        [Header("(optional) Axis Display Line Renderer")]
        public LineRenderer lineR;

        public enum Plane { XY, XZ, YZ };

        public Plane plane = Plane.XY;
        private Vector3 origin = Vector3.zero;

        private Vector3 r_center;
        private Vector3 r_init;
        private float r_mag;

        public LineRenderer axisRenderer;

        Vector3 x_axis = Vector3.right;
        Vector3 y_axis = Vector3.up;

        override public List<int> Init()
        {
            // Init is caled after GSController has added all bodies
            if (lineR != null) {
                DrawAxes();
            }
            return base.Init();
        }

        private void DrawAxes()
        {
            Vector3[] points = new Vector3[] { new Vector3(0, height, 0),
                                               new Vector3(0, 0, 0),
                                               new Vector3(width, 0, 0)};
            // fun fact: must set length first
            lineR.positionCount = points.Length;
            lineR.SetPositions(points);
        }


        private (float x, float y) MapToPlane(Vector3 rWorldAbs, double time)
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
        public Vector3 MapToScene(double3 rWorld_d3, double time)
        {
            Vector3 rWorld = GravityMath.Double3ToVector3(rWorld_d3);
            Vector3 rPlane = Vector3.zero;
            switch (plane) {
                case Plane.XY:
                    rPlane = new Vector3(rWorld.x, rWorld.y, 0);
                    break;
                case Plane.XZ:
                    rPlane = new Vector3(rWorld.x, 0, rWorld.z);
                    break;
                case Plane.YZ:
                    rPlane = new Vector3(0, rWorld.y, rWorld.z);
                    break;
            }
            return displayRotation * (scale * rPlane) + displayPosition;
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
            for (int i = 0; i < displayObjects.Length; i++) {
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
                        (float x, float y) = MapToPlane(r_vec, t);

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
