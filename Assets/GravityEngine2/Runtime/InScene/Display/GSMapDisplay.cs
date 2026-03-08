using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;
using Unity.Mathematics;
namespace GravityEngine2 {
    /// <summary>
	/// Display objects and trajectories on a map projection of a planet.
	///
	/// NOTE: to have the usual Cartesian X, Y plane need to have the camera located at -Z looking
	/// toward the origin, since Unity has a LH coordinate system.
	///
	/// </summary>
    public class GSMapDisplay : GSDisplay {

        // note that default plane is 10 x 10 units
        [Header("Map Width (Unit units)")]
        public float width = 100.0f;

        [Header("Width Axis")]
        public Vector3 widthAxis = new Vector3(1, 0, 0);

        [Header("Map Height (Unit units)")]
        public float height = 50.0f;

        [Header("Height Axis")]
        public Vector3 heightAxis = new Vector3(0, 0, 1);   // assume XZ mode is more common

        // when a line/body is positioned over a map need to put it slightly above the map (and
        // will step some traj points below, so the right/left edge wrap-around is put below the
        // map.
        public Vector3 aboveMapOffset;

        // Vector in direction of x-axis for spherical decomp. Needs to be ortho to rotation axis.
        public Vector3 longitude0Axis = Vector3.right;

        // offset for lon=0 on the Map. For Greenwich in middle this is 180.0
        public float longitude0Deg = 0;
        private float long0OffsetRad;

        // If Earth, need to align initial Earth rotation with JD
        public bool earthOffset;
        private float phi0StartOffset;

        public GSBody centerBody;

        public LineRenderer debug3060;

        private Vector3 rCenter;

        private Vector3 origin = Vector3.zero;
        private Vector3 relativeYaxis;
        private Vector3 rotationAxis;
        // rotation rate (CCW, rad/sec)
        private float omega;

        private float TWO_PI = 2.0f * Mathf.PI;

        override public List<int> Init()
        {
            rotationAxis = centerBody.rotationAxis.normalized;
            if (rotationAxis.magnitude == 0) {
                Debug.LogError("Setup Issue. Rotation is a zero vector");
            } else if (longitude0Axis.magnitude == 0) {
                Debug.LogError("Setup Issue. Lon0 is a zero vector");
            } else if (widthAxis.magnitude == 0) {
                Debug.LogError("Setup Issue. Width axis is zero");
            } else if (heightAxis.magnitude == 0) {
                Debug.LogError("Setup Issue. Height axis is zero");
            }
            omega = (float)centerBody.rotationRate;
            relativeYaxis = Vector3.Cross(rotationAxis, longitude0Axis).normalized;
            Debug.Log("rY=" + relativeYaxis);
            // assume map is centered on transform position
            origin = -0.5f * width * widthAxis + transform.position;
            long0OffsetRad = longitude0Deg * Mathf.Deg2Rad;
            phi0StartOffset = 0;
            if (earthOffset) {
                double jd = gsController.JDTime();
                phi0StartOffset = (float)Orbital.GreenwichSiderealTime(jd);
                Debug.LogFormat("phi offset = {0} at jd={1}", phi0StartOffset, jd);
            }
            if (debug3060 != null)
                Draw3060Lines();
            return base.Init();
        }

        override
        protected void DisplayUpdateInit(GECore ge, double elapsedWorldTime, double timeOvershoot)
        {
            // need a reference point for the center body world position
            GEBodyState centerState = new GEBodyState();
            ge.StateById(centerBody.Id(), ref centerState);
            if (displayMode == DisplayMode.INTERPOLATE) {
                centerState.r -= centerState.v * timeOvershoot;
            }
            rCenter = GravityMath.Double3ToVector3(centerState.r);
        }

        private (float x, float y) MapToXY(Vector3 rWorldAbs, double time)
        {
            Vector3 rWorldNorm = (rWorldAbs - rCenter).normalized;
            // Determine spherical polar co-ordinates of orbit position
            // 1) Determine (x, y, z) with respect to the rotation axis
            float z_component = Vector3.Dot(rWorldNorm, rotationAxis);
            Vector3 xy_component = rWorldNorm - z_component * rotationAxis;
            // define x axis as aligned with longitude
            float x_component = Vector3.Dot(xy_component, longitude0Axis);
            float y_component = Vector3.Dot(xy_component, relativeYaxis);
            // 2) Get theta (from z) and phi (from longitudeRef)
            float theta = Mathf.Acos(z_component); // denom orbitPosition is normalized
            float phi = Mathf.Atan2(y_component, x_component);
            // NASA map is just a pure 1:1 angle map. Not Mercator
            phi -= (float)(time * omega);
            phi += long0OffsetRad;
            phi -= phi0StartOffset;
            return MapProject(theta, phi);
        }

        override
        public Vector3 MapToScene(double3 rWorldAbs, double time)
        {
            (float x, float y) = MapToXY(GravityMath.Double3ToVector3(rWorldAbs), time);
            return x * widthAxis + y * heightAxis + origin + aboveMapOffset;
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
            float x_last = 0, y_last = 0;
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
                        if (j == 0) {
                            x_last = x;
                            y_last = y;
                        }
                        // Note: duplicate line from GSDMapToScene, since we need to watch for x, y jumps
                        // LR wrap jumps by map width, UD jumps by about half map width
                        if (Mathf.Abs(x - x_last) > 0.4f * width) {
                            // Wrapping: repeat previous point at -mapOffset, then go below map to new edge, then up
                            points[pCount] = x_last * widthAxis + y_last * heightAxis + origin - aboveMapOffset;
                            points[pCount + 1] = x * widthAxis + y * heightAxis + origin - aboveMapOffset;
                            points[pCount + 2] = x * widthAxis + y * heightAxis + origin + aboveMapOffset;
                            pCount += 3;
                        } else {
                            points[pCount] = x * widthAxis + y * heightAxis + origin + aboveMapOffset;
                            pCount++;
                        }
                        p = (p + 1) % size;
                        x_last = x;
                        y_last = y;
                    }
                    // fill in the line renderer
                    displayObjects[i].trajLine.positionCount = pCount - 1;
                    displayObjects[i].trajLine.SetPositions(points);
                }
            }
        }

        public (float x, float y) MapProject(double theta, double phiIn)
        {
            double phi = phiIn; // + longitudeOffsetRadians;
            phi = phi % TWO_PI;
            if (phi < 0) {
                phi += TWO_PI;
            }
            float x = (float)(width * phi / TWO_PI);
            float latitude = (float)(0.5f * Mathf.PI - theta);
            float y = latitude * height / Mathf.PI;
            return (x, y);
        }

        /// <summary>
		/// For texture calibration it is useful to draw lines at latitudes -60, -30, 30, 60 and see if the image
		/// is actually a 1:1 projection or has a different vertical scaling.
		///
		/// For Earth the southern tip of Africa is about 33 S degrees.
		/// Southern tip of Greenland in 60 N.
		/// 
		/// </summary>
        private void Draw3060Lines()
        {
            Vector3[] points = new Vector3[3 * 5];
            // load with phi/theta and then map to x, y
            float RHS = TWO_PI * 0.99f;
            float thirty = Mathf.PI / 6.0f;
            float theta = thirty;
            float x, y;
            for (int i = 0; i < 5; i++) {
                (x, y) = MapProject(theta, 0);
                points[3 * i] = x * widthAxis + y * heightAxis + origin;
                (x, y) = MapProject(theta, RHS);
                points[3 * i + 1] = x * widthAxis + y * heightAxis + origin;
                points[3 * i + 2] = points[3 * i]; // back to start
                theta += thirty;
            }
            debug3060.positionCount = points.Length;
            debug3060.SetPositions(points);
        }
    }
}
