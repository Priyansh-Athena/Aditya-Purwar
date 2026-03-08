using Unity.Mathematics;
using UnityEngine;


namespace GravityEngine2 {
    public class EarthLaunchSite {

        public enum Site { KENNEDY, WALLOPS, VANDENBERG, KOUROU, BAIKONAOUR, CHINA };

        public Site site;

        public static double[] latForSite = { 28.0, 37.8, 34.4, 5.2, 45.6, 28.25 };
        public static double[] longForSite = { -80.0, -75.5, -120.35, -52.8, 63.4, 102.0 };


        public bool rotateToStartTime = true;

        private float omegaDeg;
        // relative angle of 0 longitude wrt Vernal equinox (x-axis) at GE start time
        private float initPhi;

        private double latitude;
        private double longitude;
        private double radius;

        public EarthLaunchSite(Site site, double startJD, double radius, double latitude = 0.0, double longitude = 0.0)
        {
            this.radius = radius;

            this.latitude = latitude;
            this.longitude = longitude;

            omegaDeg = 360.0f / (float)GBUnits.SECS_PER_SIDEREAL_DAY;
            // determine initial rotation of Earth based on Julian day
            double theta_GMST = Orbital.GreenwichSiderealTime(startJD);
            initPhi = (float)theta_GMST * Mathf.Rad2Deg;

        }

        private (double, double, double) GetXZY(double timeSec, bool rotate)
        {
            double theta = (90.0 - latitude) * Mathf.Deg2Rad;

            double phi = (longitude > 0) ? longitude : 360.0f + longitude;
            if (rotate)
                phi += initPhi + omegaDeg * timeSec;
            phi *= Mathf.Deg2Rad;
            // XZ mode
            double x = radius * math.cos(phi) * math.sin(theta);
            double z = radius * math.sin(phi) * math.sin(theta);
            double y = radius * math.cos(theta);
            return (x, z, y);
        }

        public static (double latitude, double longitude) SiteLatLon(Site site)
        {
            return (latForSite[(int)site], longForSite[(int)site]);
        }

        public Vector3 ComputePosition(double timeSec)
        {
            // get position wrt to model radius. Do not do auto rotate
            (double x, double z, double y) = GetXZY(timeSec, rotate: false);
            Vector3 pos = new Vector3((float)x, (float)y, (float)z);
            return pos;
        }

        /// <summary>
        /// Compute the velocity due to the rotation of the earth. By default it reports this for the
        /// Earth radius, but during ascent it can be asked for the coro velocity for atmospheric drag.
        /// </summary>
        /// <returns>Launch Velocity in km/sec</returns>
        public double3 LaunchVelocityKmSec()
        {
            (double x, double z, double y) = GetXZY(radius, rotateToStartTime);
            double axialR = math.sqrt(x * x + z * z);
            double v_mag = omegaDeg * Mathf.Deg2Rad * axialR; // axial velocity in km/sec
            double3 v = math.normalize(math.cross(new double3(x, z, 0), new double3(0, 1, 0)));
            return v_mag * v;
        }

        /// <summary>
        /// Determine the launch frame given target inclination, lat, lon. 
        /// x is prograde direction of lauch
        /// y is local vertical
        /// (x, y) are in the target plane
        /// n is the normal to x, y
        /// </summary>
        /// <param name="targetInclinatonDeg"></param>
        /// <returns>(x, y, n)</returns>
        public Vector3 TargetFrame(double timeSec, double targetInclinatonDeg)
        {
            float latAbs = (float)math.abs(latitude);
            if (targetInclinatonDeg < latAbs) {
                Debug.LogWarning("Cannot achieve desired inclination from launch point");
                return Vector3.zero;
            } else {
                Vector3 r = ComputePosition(timeSec);
                return TargetFrame(targetInclinatonDeg, r, latitude);
            }
        }


        public static Vector3 TargetFrame(double targetInclDeg, Vector3 r, double latitude)
        {
            Vector3 earthAxis = new Vector3(0, 1, 0); // XZ mode
            r = r.normalized;

            float incl = (float)targetInclDeg * Mathf.Deg2Rad;
            // spherical trig. Vallado p338 and Appendix C
            float latAbs = (float)math.abs(latitude);
            float sinBeta = Mathf.Cos(incl) / Mathf.Cos(latAbs * Mathf.Deg2Rad);
            sinBeta = Mathf.Clamp(sinBeta, -1.0f, 1.0f);
            float betaDeg = Mathf.Asin(sinBeta) * Mathf.Rad2Deg;
            // get local x, n on surface of earth (y is same as r, local vertical)
            Vector3 r_y = Vector3.Project(r, earthAxis);
            Vector3 n;
            if (r_y.magnitude < Orbital.SMALL_E) {
                n = earthAxis;
            } else {
                n = r_y - Vector3.Dot(r_y, r) * r;
            }
            n = n.normalized;
            // find plane of launch. Rotate n frame by betaCompl
            float betaComlp = 90.0f - betaDeg;
            n = Quaternion.AngleAxis(-betaComlp, r) * n;

            //Debug.LogFormat("y={0} n={1} beta={2} (deg) |n|={3} angle={4}",
            //    r, n.normalized, betaDeg, n.magnitude, Vector3.Angle(n,r));

            return n.normalized;
        }


    }
}
