using System;
using UnityEngine;

namespace GravityEngine2 {

    public class OrbitDisplayCommon : MonoBehaviour {
        const float HYPERBOLA_EPSILON = 0.001f;

        /// <summary>
        /// Determine the orbit positions assuming center at (0,0,0). The scale is set by coe.p
        /// </summary>
        /// <param name="coe"></param>
        /// <param name="numPoints"></param>
        /// <param name="xzOrbit"></param>
        /// <returns></returns>
        public static void OrbitPositions(Orbital.COE coe, ref Vector3[] points, bool snapToShip = true)
        {
            // Set the orientation of the orbit
            Quaternion rot = Quaternion.AngleAxis((float)(GravityMath.RAD2DEG * coe.omegaU), GravityMath.zAxis)
                        * Quaternion.AngleAxis((float)(GravityMath.RAD2DEG * coe.i), GravityMath.xAxis)
                        * Quaternion.AngleAxis((float)(GravityMath.RAD2DEG * coe.omegaL), GravityMath.zAxis);
            float dtheta = 2.0f * Mathf.PI / points.Length;
            float theta = 0;
            if (double.IsNaN(coe.e)) {
                // radial infall, no way to know where the ship is since COE will be Nans
                for (int i = 0; i < points.Length; i++) {
                    points[i] = new Vector3(0, 0, 0);
                }
                return;
            } else if (coe.e >= 1.0) {
                // limits for hyperbola Vallado 2.29
                float acose = Mathf.Acos(1.0f / (float)coe.e);
                float to = Mathf.PI - acose - HYPERBOLA_EPSILON;
                float from = -Mathf.PI + acose + HYPERBOLA_EPSILON;
                theta = from;
                dtheta = (2.0f * to) / points.Length;
            }
            float r;
            bool snapped = !snapToShip;
            for (int i = 0; i < points.Length; i++) {
                r = (float)(coe.p / (1 + coe.e * Mathf.Cos(theta)));
                points[i] = new Vector3(r * Mathf.Cos(theta), r * Mathf.Sin(theta), 0);
                points[i] = rot * points[i];
                theta += dtheta;
                if (!snapped && theta > coe.nu) {
                    // snap to ship position
                    theta = (float)coe.nu;
                    snapped = true;
                }
            }
        }

        /// <summary>
        /// Determine the orbit positions assuming center at (0,0,0). The scale is set by coe.p
        /// 
        /// If the orbit is an ellipse then the points can wrap from the to to the from TA.
        /// 
        /// If the orbit is a hyperbola then a cutoff is applied to the TA range to prevent wrapping.
        /// </summary>
        /// <param name="coe"></param>
        /// <param name="numPoints"></param>
        /// <param name="xzOrbit"></param>
        /// <returns></returns>
        public static Vector3[] OrbitPositionsToFromTA(Orbital.COE coe,
                                                       int numPoints,
                                                       double fromTARad,
                                                       double toTARad)
        {
            Vector3[] points = new Vector3[numPoints];
            // Set the orientation of the orbit
            Quaternion rot = Quaternion.AngleAxis((float)(GravityMath.RAD2DEG * coe.omegaU), GravityMath.zAxis)
                        * Quaternion.AngleAxis((float)(GravityMath.RAD2DEG * coe.i), GravityMath.xAxis)
                        * Quaternion.AngleAxis((float)(GravityMath.RAD2DEG * coe.omegaL), GravityMath.zAxis);
            // need net angular distance clockwise from -> to
            double toRad = toTARad;
            double fromRad = fromTARad;

            if (coe.e >= 1.0) {
                // limits for hyperbola Vallado 2.29
                // float is good enough for display points
                float acose = Mathf.Acos(1.0f / (float)coe.e);
                float max_to = Mathf.PI - acose - HYPERBOLA_EPSILON;
                float min_from = -Mathf.PI + acose + HYPERBOLA_EPSILON;
                toRad = GravityMath.AngleNegPiToPi(toRad);
                fromRad = GravityMath.AngleNegPiToPi(fromRad);
                // cannot wrap hyperbola, so flip
                if (toRad < fromRad) {
                    (fromRad, toRad) = (toRad, fromRad);
                }
                // change to angles in range -pi to pi
                toRad = Math.Min(max_to, toRad);
                fromRad = Math.Max(min_from, fromRad);

            } else {
                if (toRad < fromRad) {
                    toRad += 2.0f * Mathf.PI;
                }
            }
            float dtheta = (float)(toRad - fromRad) / (points.Length - 1);
            float theta = (float)fromRad;
            float r;
            for (int i = 0; i < points.Length; i++) {
                r = (float)(coe.p / (1 + coe.e * Mathf.Cos(theta)));
                points[i] = new Vector3(r * Mathf.Cos(theta), r * Mathf.Sin(theta), 0);
                points[i] = rot * points[i];
                theta += dtheta;
            }
            return points;
        }

        /// <summary>
        /// Provide equally spaced point around the ellipse by iterating over the eccentric anamoly. 
        /// 
        /// REF: Idea from DiPrinzo "Methods of Orbital Maneuvering" AIAA Press.
        /// </summary>
        /// <param name="coe"></param>
        /// <param name="numPoints"></param>
        /// <param name="xzOrbit"></param>
        /// <returns></returns>
        public static Vector3[] OrbitPositionsEllipseByE(Orbital.COE coe, int numPoints)
        {
            Vector3[] points = new Vector3[numPoints];
            // Set the orientation of the line renderer
            Quaternion rot = Quaternion.AngleAxis((float)(GravityMath.RAD2DEG * coe.omegaU), GravityMath.zAxis)
                        * Quaternion.AngleAxis((float)(GravityMath.RAD2DEG * coe.i), GravityMath.xAxis)
                        * Quaternion.AngleAxis((float)(GravityMath.RAD2DEG * coe.omegaL), GravityMath.zAxis);
            float dE = 2.0f * Mathf.PI / points.Length;
            float E = 0;
            float r;
            for (int i = 0; i < points.Length; i++) {
                r = (float)(coe.a * (1 - coe.e * Mathf.Cos(E)));
                float theta = (float)Orbital.ConvertEtoTrueAnomoly(E, coe.e);
                points[i] = new Vector3(r * Mathf.Cos(theta), r * Mathf.Sin(theta), 0);
                points[i] = rot * points[i];
                E += dE;
            }

            return points;
        }
    }
}
