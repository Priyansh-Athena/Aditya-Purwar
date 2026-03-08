using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
namespace GravityEngine2 {
    /// <summary>
	/// Project the physical orbit onto the XZ view plane. 
    /// 
    /// ALL the heavy lifting is done by GSDisplay.
	///
	/// </summary>
    public class GSDisplay2DCoro : GSDisplay {

        // note that default plane is 10 x 10 units
        [Header("Downrange (default units)")]
        public float width = 100.0f;

        [Header("Height (default units)")]
        public float height = 50.0f;

        [Header("(optional) Axis Display Line Renderer")]
        public LineRenderer lineR;

        public enum Plane { XY, XZ, YZ };

        public Plane plane = Plane.XY;
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


        /// <summary>
		/// Intended to only be called for the body being launched!
		/// </summary>
		/// <param name="rWorldAbs"></param>
		/// <param name="time"></param>
		/// <returns></returns>
        override
        public Vector3 MapToScene(double3 rWorld_d3, double time)
        {
            rWorld_d3 = rWorld_d3 - wrtWorldPosition;
            Vector3 rWorld = GravityMath.Double3ToVector3(rWorld_d3);

            if (xzOrbitPlane)
                GravityMath.Vector3ExchangeYZ(ref rWorld);
            switch (plane) {
                case Plane.XY:
                    rWorld = new Vector3(rWorld.x, 0, rWorld.y);
                    break;
                case Plane.XZ:
                    rWorld = new Vector3(rWorld.x, 0, rWorld.z);
                    break;
                case Plane.YZ:
                    rWorld = new Vector3(rWorld.y, 0, rWorld.z);
                    break;
            }
            return displayRotation * (scale * rWorld) + displayPosition;
        }

    }
}
