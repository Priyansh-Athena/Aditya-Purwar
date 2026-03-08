using System; // Math
using Unity.Mathematics;

namespace GravityEngine2 {
    /// <summary>
    /// Orbital utilities relating to classical orbital elements (COE) and converting between (r,v) <=> COE. 
    /// 
    /// A number of these algorithms originate with Vallado's excellent "Fundamentals of Astrodynamics and Applications"
    /// (Microcosm Press) and their adaptations in the original Gravity Engine asset.
    /// 
    /// There are also some other useful algorithms. 
    /// 
    /// 
    /// </summary>
    public class Orbital {
        // small angle in radians
        public const double SMALL_ANGLE = 1E-3;

        // limit for small eccentricity (i.e. when an orbit is a circle)
        public const double SMALL_E = 1E-3;

        public const double SMALL_P = 1E-6;

        public const double SMALL_SME = 1E-6; // energy in OrbitalUniversal

        public enum OrbitType { UNKNOWN, CIRCULAR_EQUATORIAL, CIRCULAR_INCLINED, ELLIPSE_EQUATORIAL, ELLIPSE_INCLINED, HYPERBOLA, FREEFALL };

        /// <summary>
        /// Classical orbit elements.
        /// - choose p, but provide helper functions to init from semi-major a
        ///
        /// All angles are in radians.
        ///
        /// This first five COEs determine the shape an orientation of the orbit.
        /// nu determines the objects poistion in the orbit so it applies at a specific time.
        ///
        /// There is some degeneracy in the COEs in several special cases.
        ///
        /// 1) Circular orbit
        /// - omegaU and omegaL are just an arbitrary change in the nu=0 point of the orbit
        ///   they should be avoided and set to zero in this case
        /// - nu is defined with respect to the positive X axis TODO: Confirm
        ///   
        /// 2) Equatorial (i=0) elliptical orbit
        /// - with no inclination the argument of perigee should be contained in omegaL and
        ///   omegaU should be zero
        /// 
        /// </summary>
        public class COE {
            public double p = 1;        // semi-latus rectum NOT periapsis
            public double a = 1;        // semi-major axis (redundant with p)
            public double e = 0;        // eccentricity
            public double i = 0;        // inclination
            public double omegaL = 0;   // lower case omega, aka argument of perigee
            public double omegaU = 0;   // upper case omega, also known as right ascension of ascending node
            public double nu = 0;       // phase in the orbit from the periapsis
            public double3x3 rotation; // computed from omegaU, omegaL and i
            public bool defined = false;
            public double mu;       // G*M for the body this orbit is around. In some cases (display only) may be NaN.
            public double startEpochJD; // (optional) start epoch Julian date. Typically used when OE from JPL. 0 when unused.

            // optional
            public double m;    // mean anomoly
            public double eccanom;

            public OrbitType orbitType;

            public COE()
            {

            }

            // standard init
            public COE(double mu, double p, double e, double i = 0.0, double omegaL = 0.0, double omegaU = 0.0, double nu = 0.0)
            {
                this.p = p;
                a = p / (1 - e * e); // can be negative for hyperbola
                this.e = e;
                this.i = i;
                this.omegaL = omegaL;
                this.omegaU = omegaU;
                this.nu = nu;
                orbitType = OrbitType.UNKNOWN;
                rotation = new double3x3();     // must assign all fields before can call a function
                defined = true;
                this.mu = mu;
                // dummy values
                m = 0;
                eccanom = 0;
                // setup rotation
                rotation = RotationMatrix();
                ComputeType();
            }

            public COE(COE fromOE)
            {
                p = fromOE.p;
                a = fromOE.a;
                e = fromOE.e;
                i = fromOE.i;
                omegaL = fromOE.omegaL;
                omegaU = fromOE.omegaU;
                nu = fromOE.nu;
                orbitType = fromOE.orbitType;
                rotation = fromOE.rotation;
                defined = true;
                mu = fromOE.mu;
                // dummy values
                m = 0;
                eccanom = 0;
            }

            public bool IsDefined()
            {
                return defined;
            }

            public bool HasNan()
            {
                return double.IsNaN(p) || double.IsNaN(e) || double.IsNaN(i) || double.IsNaN(omegaL) || double.IsNaN(omegaU)
                    || double.IsNaN(nu);
            }

            /// <summary>
            /// Init Ellipse orbital parameters. 
            /// 
            /// All angles (including inclination!) are in Radians.
            /// </summary>
            /// <param name="a"></param>
            /// <param name="e"></param>
            /// <param name="i"></param>
            /// <param name="omegaL"></param>
            /// <param name="omegaU"></param>
            /// <param name="nu"></param>
            public void EllipseInit(double mu, double a, double e, double i, double omegaL, double omegaU, double nu)
            {
                this.a = a;
                p = a * (1 - e * e);
                this.e = e;
                this.i = i;
                this.omegaL = omegaL;
                this.omegaU = omegaU;
                this.nu = nu;
                this.mu = mu;
                orbitType = OrbitType.UNKNOWN;
                rotation = new double3x3();     // must assign all fields before can call a function
                rotation = RotationMatrix();
                ComputeType();
            }
            /// <summary>
            /// Init classical orbital elements using angles in degress. 
            /// </summary>
            /// <param name="mu"></param>
            /// <param name="a"></param>
            /// <param name="e"></param>
            /// <param name="i"></param>
            /// <param name="omegaL"></param>
            /// <param name="omegaU"></param>
            /// <param name="nu"></param>
            public void EllipseInitDegrees(double mu, double a, double e, double i, double omegaL, double omegaU, double nu)
            {
                double D2R = GravityMath.DEG2RAD;
                this.a = a;
                p = a * (1 - e * e);
                if (e >= 1.0) {
                    p *= -1.0;
                }
                this.e = e;
                this.i = i * D2R;
                this.omegaL = omegaL * D2R;
                this.omegaU = omegaU * D2R;
                this.nu = nu * D2R;
                this.mu = mu;
                orbitType = OrbitType.UNKNOWN;
                rotation = new double3x3();     // must assign all fields before can call a function
                rotation = RotationMatrix();
                ComputeType();
            }

            public void CopyFrom(COE fromCoe)
            {
                a = fromCoe.a;
                p = fromCoe.p;
                e = fromCoe.e;
                i = fromCoe.i;
                omegaL = fromCoe.omegaL;
                omegaU = fromCoe.omegaU;
                nu = fromCoe.nu;
                orbitType = fromCoe.orbitType;
                rotation = fromCoe.rotation;
                mu = fromCoe.mu;
            }

            public void InitFromCOEStruct(COEStruct coeS)
            {
                a = coeS.a;
                p = coeS.p;
                e = coeS.e;
                i = coeS.i;
                omegaL = coeS.omegaL;
                omegaU = coeS.omegaU;
                nu = coeS.nu;
                rotation = coeS.rotation;
                mu = coeS.mu;
            }

            /// <summary>
            /// Scale the length parameters of the COE by the factor scale. 
            /// </summary>
            /// <param name="scale"></param>
            public void ScaleLength(double scale)
            {
                a *= scale;
                p *= scale;
            }

            /// <summary>
			/// Set length scale (semi-major axis). Use this and NOT a direct set of coe.a to ensure both
			/// a and p are scaled properly. Many COEtoRV utilities use the p value instead!
			/// </summary>
			/// <param name="a"></param>
            public void SetA(double a)
            {
                this.a = a;
                p = a * (1 - e * e);
            }

            /// <summary>
            /// Report if orbit is circular. This assumes that ComputeType() has been called. The provided
            /// constructors all compute the type on init.
            /// 
            /// An orbit is considered circular if the eccentricity is less than "smallE" (1E-3 default)
            /// </summary>
            /// <returns>true if circular</returns>
            public bool IsCircular()
            {
                return (orbitType == OrbitType.CIRCULAR_INCLINED) || (orbitType == OrbitType.CIRCULAR_EQUATORIAL);
            }

            /// <summary>
            /// Report if orbit is inclined. Assume ComputeType() called (standard constructors will do this)
            /// 
            /// The i=0 threshold is smallAngle (1E-3 radians)
            /// </summary>
            /// <returns></returns>
            public bool IsInclined()
            {
                return (orbitType == OrbitType.CIRCULAR_INCLINED) || (orbitType == OrbitType.ELLIPSE_INCLINED);
            }

            /// <summary>
            /// Determine the phase for a circular or near circular orbit. Ideally a circular, flat orbit (e = 0) will
            /// not have a value for omegaU or omegaL, but in the numerical world things are seldom that tidy. A
            /// phase for a circular orbit wrt to the x-axis is determined here. 
            /// </summary>
            /// <returns></returns>
            public double GetCircPhase()
            {
                return (omegaU + omegaL + nu) % 360.0;
            }

            /// <summary>
            /// Get angular velocity in radian per time unit
            /// </summary>
            /// <param name="mu"></param>
            /// <returns></returns>
            public double GetAngularVelocity()
            {
                return Math.Sqrt(mu / (a * a * a));
            }

            /// <summary>
            /// Get the period of the orbit. The time units are those corresponding to the
            /// length and mass units used to initialize the COE. 
            /// </summary>
            /// <returns></returns>
            public double GetPeriod()
            {
                return 2.0 * Math.PI * Math.Sqrt(a * a * a / mu);
            }

            /// <summary>
            /// Compute the mean and eccentric anomolies and fill them in in the COE struct.
            ///
            /// WARNING: Internal field values do not persist when called in PKEPLERProp. WHY?!
            /// Some kind of copy is going on, but I'm missing it.
            /// </summary>
            /// <param name="small"></param>
            public (double M, double E) ComputeMandE(double small = 1E-3)
            {
                ComputeType();
                // ------------ find mean anomaly for all orbits -----------
                if (IsCircular()) {
                    m = nu;
                    eccanom = nu;
                } else {
                    (m, eccanom) = NewtonNu(e, nu, small);
                }
                return (m, eccanom);
            }

            public void ComputeType(double smallE = SMALL_E, double smallAngle = SMALL_ANGLE)
            {
                if (e >= 1.0)
                    orbitType = OrbitType.HYPERBOLA;
                else if ((e <= smallE) && (i <= smallAngle))
                    orbitType = OrbitType.CIRCULAR_EQUATORIAL;
                else if ((e < smallE) && (i > smallAngle))
                    orbitType = OrbitType.CIRCULAR_INCLINED;
                else if ((e > smallE) && (i < smallAngle))
                    orbitType = OrbitType.ELLIPSE_EQUATORIAL;
                else if ((e > smallE) && (i > smallAngle))
                    orbitType = OrbitType.ELLIPSE_INCLINED;
            }



            /// <summary>
            /// COE has sigularities when e=0 and/or i=0. Make some pragmatic adjustments
            /// </summary>
            public void NonSingularAdjust()
            {
                if (orbitType == OrbitType.UNKNOWN)
                    ComputeType();
                if (IsInclined()) {
                    if (IsCircular()) {
                        omegaL = 0.0;
                    }
                } else {
                    // equatorial
                    if (!IsCircular()) {
                        omegaU = 0;
                    }
                }

            }

            /// <summary>
            /// Determine apoapsis (farthest point from center) and periapsis (closest
            /// point from center) for this COE. 
            /// </summary>
            /// <returns>(apoapsis, periapsis) in same length units as COE</returns>
            public (double apo, double peri) ApoPeri()
            {
                return (a * (1 + e), a * (1 - e));
            }

            public void ComputeRotation()
            {
                rotation = RotationMatrix();
            }

            /// <summary>
            /// Compute the rotation matrix per p119 in Vallado.
            /// 
            /// This rotates a vector in the X,Y plane into the correct orbit orientation.
            /// </summary>
            /// <returns></returns>
            ///
            private double3x3 RotationMatrix()
            {
                double cU = math.cos(omegaU);
                double sU = math.sin(omegaU);
                double cl = math.cos(omegaL);
                double sl = math.sin(omegaL);
                double ci = math.cos(i);
                double si = math.sin(i);

                return new double3x3(cU * cl - sU * sl * ci,
                                    -cU * sl - sU * cl * ci,
                                    sU * si,
                                    sU * cl + cU * sl * ci,
                                    -sU * sl + cU * cl * ci,
                                    -cU * si,
                                    sl * si,
                                    cl * si,
                                    ci);
            }

            /// <summary>
            /// Create a string with the COE values expressed as degress. 
            /// </summary>
            /// <returns></returns>

            public string LogStringDegrees()
            {
                return string.Format("p={0} a={1:F5} e={2:F5} i={3:F5} Om={4:F5} om={5:F5} nu={6:F5} type={7} mu={8:0.0e+0}",
                    p, a, e, i,
                    GravityMath.RAD2DEG * omegaU,
                    GravityMath.RAD2DEG * omegaL,
                    GravityMath.RAD2DEG * nu,
                    orbitType,
                    mu);
            }


        } // end class COE


        /// <summary>
        /// Light weight struct to hold COE core data for the job system and propagators.
		///
		/// Application code should generaly use the COE *class* above.
        ///
        /// In the job system structs are immutable! Fields in this struct cannot be changed
        /// (but modified copies can be assigned back)
        /// </summary>
        public struct COEStruct {
            public double p;        // semi-latus rectum NOT periapsis
            public double a;        // semi-major axis (redundant with p)
            public double e;        // eccentricity
            public double i;        // inclination
            public double omegaL;   // lower case omega, aka argument of perigee
            public double omegaU;   // upper case omega, also known as right ascension of ascending node
            public double nu;       // phase in the orbit from the periapsis
            // PKEPLER used these in every iteration so compute them once on init
            public double meanAnom;
            public double eccAnom;  // eccentric, parabloic or hyperbolic anomoly depending on e
            public double mu;

            public double n; // mean motion, only defined for e < 1.0
            public double3x3 rotation;

            public COEStruct(COE coe)
            {
                p = coe.p;
                a = coe.a;
                e = coe.e;
                i = coe.i;
                omegaL = coe.omegaL;
                omegaU = coe.omegaU;
                nu = coe.nu;
                mu = coe.mu;
                (meanAnom, eccAnom) = coe.ComputeMandE();
                rotation = new double3x3();     // must assign all fields before can call a function
                n = 0.0;
                if (e < 1.0) {
                    n = Math.Sqrt(mu / (a * a * a));
                }
                rotation = RotationMatrix();
            }

            public void ScaleLength(double scale)
            {
                a *= scale;
                p *= scale;
            }

            public void ComputeRotation()
            {
                rotation = RotationMatrix();
            }

            private double3x3 RotationMatrix()
            {
                double cU = math.cos(omegaU);
                double sU = math.sin(omegaU);
                double cl = math.cos(omegaL);
                double sl = math.sin(omegaL);
                double ci = math.cos(i);
                double si = math.sin(i);

                return new double3x3(cU * cl - sU * sl * ci,
                                    -cU * sl - sU * cl * ci,
                                    sU * si,
                                    sU * cl + cU * sl * ci,
                                    -sU * sl + cU * cl * ci,
                                    -cU * si,
                                    sl * si,
                                    cl * si,
                                    ci);
            }

            public string LogStringDegrees()
            {
                return string.Format("p={0} a={1:F5} e={2:F5} i={3:F5} Om={4:F5} om={5:F5} nu={6:F5} mu={7:0.0e+0}",
                    p, a, e, i,
                    math.degrees(omegaU),
                    math.degrees(omegaL),
                    math.degrees(nu),
                    mu);
            }
        }

        /// <summary>
        /// Given a COE and phase (radians) determine the relative position in the
        /// orbit. The length scale used will match the length scale used in the 
        /// COE initialization.
        /// 
        /// </summary>
        /// <param name="coe"></param>
        /// <param name="phaseRad"></param>
        /// <returns>relative position in orbit</returns>
        public static double3 PositionForCOE(COE coe, double phaseRad)
        {
            double denom = 1 + coe.e * Math.Cos(phaseRad);
            double3 r = new double3(coe.p * Math.Cos(phaseRad) / denom,
                                    coe.p * Math.Sin(phaseRad) / denom,
                                    0);
            return GravityMath.Matrix3x3TimesVector(coe.rotation, r);
        }

        public enum OrbitPoint { APOAPSIS, PERIAPSIS, ASC_NODE, DESC_NODE, TRUEANOM_DEG };

        /// <summary>
        /// Determine the phase for a specific OrbitPoint in an orbit (e.g. APOAPSIS)
        /// </summary>
        /// <param name="coe"></param>
        /// <param name="point"></param>
        /// <param name="deg"></param>
        /// <returns>phase in radians</returns>
        public static double PhaseForOrbitPoint(COE coe, OrbitPoint point, double deg = 0)
        {
            double phaseRad = 0.0;
            switch (point) {
                case OrbitPoint.APOAPSIS:
                    // only valid for e < 1.0
                    phaseRad = Math.PI;
                    break;
                case OrbitPoint.PERIAPSIS:
                    phaseRad = 0;
                    break;
                case OrbitPoint.ASC_NODE:
                    phaseRad = 0 - coe.omegaL;
                    break;
                case OrbitPoint.DESC_NODE:
                    phaseRad = Math.PI - coe.omegaL;
                    break;
                case OrbitPoint.TRUEANOM_DEG:
                    phaseRad = math.radians(deg);
                    break;
            }
            return phaseRad;
        }

        /// <summary>
        /// Determine phase for a given radius. There are two valid responses, 
        /// phase and -phase (or 2 Pi - phase). This return +phase
        /// </summary>
        /// <param name="coe"></param>
        /// <param name="radius"></param>
        /// <returns>phase in radians</returns>
        public static double PhaseForRadius(COE coe, double radius)
        {
            double ecc = coe.e;
            // Nudge eccentricity if needed to avoid a divide by zero
            if (math.abs(ecc - 1.0) < 1E-6)
                ecc = 1.0 + 1E-5;
            double theta = 0;
            if (ecc > 1E-8) {
                double thetaEqn = (coe.p / radius - 1) / ecc;
                theta = math.acos(math.clamp(thetaEqn, -1.0, 1.0));
            }
            return theta;
        }

        /// <summary>
        /// Determine the two possible 3D positions in an orbit that are at the
        /// specified radius.
        /// </summary>
        /// <param name="coe"></param>
        /// <param name="r"></param>
        /// <returns>(pos1, pos2) two 3D positions</returns>
        public static (double3, double3) PositionsForR(COE coe, double r)
        {
            double theta = PhaseForRadius(coe, r);
            (double3 r1, double3 v1) = COEWithPhasetoRVRelative(coe, theta);
            (double3 r2, double3 v2) = COEWithPhasetoRVRelative(coe, -theta);
            return (r1, r2);
        }

        /// <summary>
        /// Determine the 3D position and velocity (relative to the center) for 
        /// the specified type of OrbitPoint
        /// </summary>
        /// <param name="coe"></param>
        /// <param name="point"></param>
        /// <param name="deg"></param>
        /// <returns>(pos, vel) relative 3D position and velocity</returns>
        public static (double3, double3) RVForOrbitPoint(COE coe, OrbitPoint point, double deg = 0)
        {
            double phaseRad = PhaseForOrbitPoint(coe, point, deg);
            return COEWithPhasetoRVRelative(coe, phaseRad);
        }

        /// <summary>
        /// Determine the 3D velocity vector for a COE at the specified phase.
        /// </summary>
        /// <param name="coe"></param>
        /// <param name="phaseRad"></param>
        /// <returns>3D velocity</returns>
        public static double3 VelocityForCOE(COE coe, double phaseRad)
        {
            double factor = Math.Sqrt(coe.mu / coe.p);
            double3 v = new double3(-factor * Math.Sin(phaseRad),
                                    factor * (coe.e + Math.Cos(phaseRad)),
                                    0);
            return GravityMath.Matrix3x3TimesVector(coe.rotation, v);
        }

        /// <summary>
        /// Determine the orbit axis (normalized)
        /// </summary>
        /// <param name="coe"></param>
        /// <returns>orbit axis</returns>
        public static double3 OrbitAxis(COE coe)
        {
            double3 r = new double3(0, 0, 1);
            return GravityMath.Matrix3x3TimesVector(coe.rotation, r);
        }

        /// <summary>
        /// Rotate a 3D vector specified with respect to the default orbit
        /// frame (XY orbital plane) into the 3D space where the orientation
        /// is adjusted by inclination, omegaU and omegaL.
        /// </summary>
        /// <param name="coe"></param>
        /// <param name="r"></param>
        /// <returns>rotated vector</returns>
        public static double3 RotateToOrbitFrame(COE coe, double3 r)
        {
            return GravityMath.Matrix3x3TimesVector(coe.rotation, r);
        }

        /// <summary>
        /// Determine the phase in radians for the specified direction. This
        /// can be a point on the orbit but this is not required as only the 
        /// direction of the given vector is used. 
        /// 
        /// </summary>
        /// <param name="pos">direction relative to the orbit center</param>
        /// <param name="coe">orbital elements for the orbit</param>
        /// <returns>phase in the specified direction (radians)</returns>
        public static double PhaseAngleRadiansForDirection(double3 pos, COE coe)
        {
            double3 x_unit = new double3(1, 0, 0);
            double3 y_unit = new double3(0, 1, 0);
            double3 x_axis = RotateToOrbitFrame(coe, x_unit);
            double3 y_axis = RotateToOrbitFrame(coe, y_unit);
            double3 pos_unit = math.normalize(pos);
            double x_component = math.dot(math.project(pos_unit, x_axis), x_axis);
            double y_component = math.dot(math.project(pos_unit, y_axis), y_axis);
            double phaseRad = math.atan2(y_component, x_component);
            if (phaseRad < 0) {
                phaseRad += 2.0 * Math.PI;
            }
            return phaseRad;
        }

        /// <summary>
        /// Determine if two COEs are equal within the indicated tolerances. 
        /// 
        /// Phase comparison can be excluded (Useful when getting COE for display orbit, 
        /// where the phase does not matter)
        /// 
        /// </summary>
        /// <param name="coe1"></param>
        /// <param name="coe2"></param>
        /// <param name="includePhase"></param>
        /// <param name="sizeTol"></param>
        /// <param name="angleRadTol"></param>
        /// <returns></returns>
        public static bool COEEqual(COE coe1, COE coe2, bool includePhase, double sizeTol = 1E-2, double angleRadTol = 1E-3)
        {
            if (coe2 == null)
                return false;
            if (math.abs(coe1.a - coe2.a) > sizeTol)
                return false;
            // not technically an angle, but use angle tol for eccentricity
            if (math.abs(coe1.e - coe2.e) > angleRadTol)
                return false;
            if (GravityMath.MinAngleDeltaRadians(coe1.omegaL, coe2.omegaL) > angleRadTol)
                return false;
            if (GravityMath.MinAngleDeltaRadians(coe1.omegaU, coe2.omegaU) > angleRadTol)
                return false;
            if (GravityMath.MinAngleDeltaRadians(coe1.i, coe2.i) > angleRadTol)
                return false;
            if (includePhase && GravityMath.MinAngleDeltaRadians(coe1.nu, coe2.nu) > angleRadTol)
                return false;
            return true;
        }

        /// <summary>
        /// Determine the relative R and V for a specified set of classical orbital elements
        /// and a GM (mu) for the body at the center of the orbit.
        /// 
        /// </summary>
        /// <param name="oe"></param>
        /// <param name="mu"></param>
        /// <returns></returns>
        /// 
        public static (double3 r, double3 v) COEtoRVRelative(COE oe)
        {
            return COEWithPhasetoRVRelative(oe, oe.nu);
        }

        /// <summary>
        /// Determine the relative position and velocity for the point in an orbit
        /// specified by the COE at the given phase in radians.
        /// </summary>
        /// <param name="oe"></param>
        /// <param name="phaseRad"></param>
        /// <returns>(pos, vel) relative position and velocity</returns>
        public static (double3 r, double3 v) COEWithPhasetoRVRelative(COE oe, double phaseRad)
        {
            // modelled after Vallado COEtoRV

            // ensure this is redone in case angles have changed
            oe.ComputeRotation();

            // special cases: zero any residual omega values in special cases
            // Vallada caries info in argp, longper and truelon. Need to document what to do with this
            // Expect this may cause some issues if values come from RVtoCOE
            bool equatorial = (oe.i < SMALL_ANGLE) || (Math.Abs(oe.i - Math.PI) < SMALL_ANGLE);
            if (oe.e < SMALL_E) {
                if (equatorial) {
                    // circular, equitorial 
                    oe.omegaL = 0.0;
                    oe.omegaU = 0.0;
                } else {
                    // circular inclined
                    oe.omegaL = 0.0;

                }
            } else if (equatorial) {
                oe.omegaU = 0.0;
            }

            // Form r, v vectors in the XY plane
            double cosnu = Math.Cos(phaseRad);
            double sinnu = Math.Sin(phaseRad);
            double temp = oe.p / (1.0 + oe.e * cosnu);
            double3 rpqw = new double3(temp * cosnu, temp * sinnu, 0.0);
            if (Math.Abs(oe.p) < SMALL_P) {
                oe.p = SMALL_P;
            }
            double sqrtmup = Math.Sqrt(oe.mu / oe.p);
            double3 vpqw = new double3(-sinnu * sqrtmup,
                                         (oe.e + cosnu) * sqrtmup,
                                         0.0);
            // rotate into position
            double3 r = GravityMath.Matrix3x3TimesVector(oe.rotation, rpqw);
            double3 v = GravityMath.Matrix3x3TimesVector(oe.rotation, vpqw);
            return (r, v);
        }

        /// <summary>
        /// COEStruct version to determine relative position and velocity for a given COE
        /// and phase.
        ///
        /// Used in job system (PKEPLER) to determine R, V
        /// 
        /// </summary>
        /// <param name="coe"></param>
        /// <param name="phaseRad"></param>
        /// <returns>(pos, vel) relative position and velocity</returns>
        public static (double3 r, double3 v) COEStructWithPhasetoRVRelative(COEStruct coe, double phaseRad)
        {
            // modelled after Vallado COEtoRV

            // special cases: zero any residual omega values in special cases
            // Vallada caries info in argp, longper and truelon. Need to document what to do with this
            // Expect this may cause some issues if values come from RVtoCOE
            bool equatorial = (coe.i < SMALL_ANGLE) || (Math.Abs(coe.i - Math.PI) < SMALL_ANGLE);
            if (coe.e < SMALL_E) {
                if (equatorial) {
                    // circular, equitorial 
                    coe.omegaL = 0.0;
                    coe.omegaU = 0.0;
                } else {
                    // circular inclined
                    coe.omegaL = 0.0;

                }
            } else if (equatorial) {
                coe.omegaU = 0.0;
            }

            // Form r, v vectors in the XY plane
            double cosnu = Math.Cos(phaseRad);
            double sinnu = Math.Sin(phaseRad);
            double temp = coe.p / (1.0 + coe.e * cosnu);
            double3 rpqw = new double3(temp * cosnu, temp * sinnu, 0.0);
            if (Math.Abs(coe.p) < SMALL_P) {
                coe.p = SMALL_P;
            }
            double sqrtmup = Math.Sqrt(coe.mu / coe.p);
            double3 vpqw = new double3(-sinnu * sqrtmup,
                                         (coe.e + cosnu) * sqrtmup,
                                         0.0);
            // rotate into position
            double3 r = GravityMath.Matrix3x3TimesVector(coe.rotation, rpqw);
            double3 v = GravityMath.Matrix3x3TimesVector(coe.rotation, vpqw);
            return (r, v);
        }

        /// <summary>
        /// Determine state from COE and a time from current phase in COE. This used a KeplerPropagator and it
        /// might be better to use this approach directly if this will be used more than once. 
        /// </summary>
        /// <param name="coe"></param>
        /// <param name="time"></param>
        /// <returns></returns>
        public static (double3 r, double3 v) COEtoRVatTime(COE coe, double time)
        {
            (double3 r0, double3 v0) = COEtoRVRelative(coe);
            KeplerPropagator.RVT rvt = new KeplerPropagator.RVT(r0, v0, t0: 0.0, mu: coe.mu);
            (int status, double3 r, double3 v) = KeplerPropagator.RVforTime(rvt, time);
            return (r, v);
        }


        private const double verySmall = 1e-9;

        /// <summary>
        /// Given state information (r,v) and mu=GM determine the classical
        /// orbital elements (COE).
        ///
        /// Classical orbital elements have inherent singularities for:
        /// circular orbits (e=0)
        /// flat orbits (i=0 or Pi radians)
        ///
        /// In this case the sense of how small is zero is set by the optional parameter `small`.
        ///
        /// In the case of circular orbits there is no closest approach point from which to measure phase nu.
        /// For circular orbits the X-axis defines nu=0.
        ///
        /// In the case of a flat orbit, there is no ascending node to provide a reference for omegaU.
        /// It is set to zero.
        ///
        /// See the discussion in the Orbits section of the documentation for other information.
        /// 
        /// </summary>
        /// <param name="r"></param>
        /// <param name="v"></param>
        /// <param name="mu"></param>
        /// <param name="small (optional) threshold for detecting circular/flat orbits"></param>
        /// <returns>COE (struct containing orbit elements)</returns>
        public static COE RVtoCOE(double3 r,
                            double3 v,
                            double mu,
                            double small = 1e-6)
        {

            double magr, magv, magn, sme, rdotv, temp, c1, hk, magh;

            COE oe = new COE {
                defined = true,
                mu = mu,
                e = 0.0
            };

            // -------------------------  implementation   -----------------
            magr = math.length(r);
            magv = math.length(v);

            // ------------------  find h n and e vectors   ----------------
            double3 hbar = math.cross(r, v);
            magh = math.length(hbar);
            if (magh > verySmall) {
                double3 nbar = new double3(-hbar.y, hbar.x, 0.0);
                magn = math.length(nbar);
                c1 = magv * magv - mu / magr;
                rdotv = math.dot(r, v);
                temp = 1.0 / mu;
                double3 ebar = new double3((c1 * r.x - rdotv * v.x) * temp,
                                             (c1 * r.y - rdotv * v.y) * temp,
                                             (c1 * r.z - rdotv * v.z) * temp);
                oe.e = math.length(ebar);

                // ------------  find a e and semi-latus rectum   ----------
                sme = (magv * magv * 0.5) - (mu / magr);
                // was check vs small, but really care about > 0
                //if (Math.Abs(sme) > verySmall)
                //    oe.a = -mu / (2.0 * sme);
                //else
                //    oe.a = double.NaN;
                oe.p = magh * magh * temp;

                // -----------------  find inclination   -------------------
                hk = hbar.z / magh;
                oe.i = Math.Acos(Math.Clamp(hk, -1.0, 1.0));

                oe.orbitType = OrbitType.ELLIPSE_INCLINED; ;

                if (oe.e < SMALL_E) {
                    // ----------------  circular equatorial ---------------
                    if ((oe.i < SMALL_ANGLE) || (Math.Abs(oe.i - Math.PI) < SMALL_ANGLE)) {
                        oe.orbitType = OrbitType.CIRCULAR_EQUATORIAL;
                    } else {
                        oe.orbitType = OrbitType.CIRCULAR_INCLINED;
                    }
                } else {
                    // - elliptical, parabolic, hyperbolic equatorial --
                    if ((oe.i < SMALL_ANGLE) || (Math.Abs(oe.i - Math.PI) < SMALL_ANGLE)) {
                        oe.orbitType = OrbitType.ELLIPSE_EQUATORIAL;
                    }
                }

                // ----------  find right ascension of the ascending node ------------
                if (magn > verySmall) {
                    temp = nbar.x / magn;
                    if (Math.Abs(temp) > 1.0)
                        temp = Math.Sign(temp);
                    // NBP: change raan to omega_U
                    oe.omegaU = Math.Acos(Math.Clamp(temp, -1.0, 1.0));
                    if (nbar.y < 0.0)
                        oe.omegaU = 2.0 * Math.PI - oe.omegaU;
                } else {
                    oe.omegaU = 0;
                }

                // ---------------- find argument of perigee ---------------
                if (oe.orbitType == OrbitType.ELLIPSE_INCLINED) {
                    // argp -> omegaL
                    oe.omegaL = GravityMath.Angle(nbar, ebar);
                    if (ebar.z < 0.0)
                        oe.omegaL = 2.0 * Math.PI - oe.omegaL;
                } else
                    oe.omegaL = 0;

                // ------------  find true anomaly at epoch    -------------
                if (!oe.IsCircular()) {
                    oe.nu = GravityMath.Angle(ebar, r);
                    if (rdotv < 0.0)

                        oe.nu = 2.0 * Math.PI - oe.nu;
                } else
                    oe.nu = double.NaN;

                // ----  find argument of latitude - circular inclined -----
                if (oe.orbitType == OrbitType.CIRCULAR_INCLINED) {
                    // arglat -> nu
                    oe.nu = GravityMath.Angle(nbar, r);
                    if (r.z < 0.0)
                        oe.nu = 2.0 * Math.PI - oe.nu;
                    oe.m = oe.nu;
                }
                //else
                //    oe.arglat = double.NaN;

                // -- find longitude of perigee - elliptical equatorial ----
                if ((oe.e > small) && (oe.orbitType == OrbitType.ELLIPSE_EQUATORIAL)) {
                    temp = ebar.x / oe.e;
                    if (Math.Abs(temp) > 1.0)
                        temp = Math.Sign(temp);
                    // longper -> omegaL
                    oe.omegaL = Math.Acos(Math.Clamp(temp, -1.0, 1.0));
                    if (ebar.y < 0.0)
                        oe.omegaL = 2.0 * Math.PI - oe.omegaL;
                    if (oe.i > 0.5 * Math.PI)
                        oe.omegaL = 2.0 * Math.PI - oe.omegaL;
                    oe.omegaU = 0.0;
                }
                //else
                //    oe.lonper = double.NaN;

                //// -------- find true longitude - circular equatorial ------
                if ((magr > verySmall) && (oe.orbitType == OrbitType.CIRCULAR_EQUATORIAL)) {
                    temp = r.x / magr;
                    if (Math.Abs(temp) > 1.0)
                        temp = Math.Sign(temp);
                    double truelon = Math.Acos(Math.Clamp(temp, -1.0, 1.0));
                    if (r.y < 0.0)
                        truelon = 2.0 * Math.PI - truelon;
                    if (oe.i > 0.5 * Math.PI)
                        truelon = 2.0 * Math.PI - truelon;
                    oe.nu = truelon;
                    // NBP: zero out omegaU and omegaL
                    oe.omegaL = 0.0;
                    oe.omegaU = 0.0;
                }
                oe.a = oe.p / (1 - oe.e * oe.e);
                //else
                //    oe.truelon = double.NaN;

                // compute period
                //oe.period = double.NaN;
                //if (oe.e < 1.0) {
                //    oe.period = 2.0 * Math.PI * Math.Sqrt(oe.a * oe.a * oe.a / mu);
                //}
            } else {
                oe.p = double.NaN;
                oe.e = double.NaN;
                oe.i = double.NaN;
                oe.omegaU = double.NaN;
                oe.omegaL = double.NaN;
                oe.nu = double.NaN;
                oe.orbitType = OrbitType.FREEFALL;
                //Debug.LogWarning(string.Format("h too small. r0={0} v0={1} center={2}", 
                //    r, v, centerBody.gameObject.name));
            }
            oe.ComputeRotation();
            oe.ComputeType();
            return oe;
        }  // rv2coe

        /// <summary>
        /// Taken from Vallado source code site. (Also used in LambertUniversal)
        /// Internal use only.
        /// </summary>
        /// <param name="znew"></param>
        /// <param name="c2new"></param>
        /// <param name="c3new"></param>
        public static void FindC2C3(double znew, out double c2new, out double c3new)
        {
            double small, sqrtz;
            small = 0.00000001;

            // -------------------------  implementation   -----------------
            if (znew > small) {
                sqrtz = Math.Sqrt(znew);
                c2new = (1.0 - Math.Cos(sqrtz)) / znew;
                c3new = (sqrtz - Math.Sin(sqrtz)) / (sqrtz * sqrtz * sqrtz);
            } else {
                if (znew < -small) {
                    sqrtz = Math.Sqrt(-znew);
                    c2new = (1.0 - Math.Cosh(sqrtz)) / znew;
                    c3new = (Math.Sinh(sqrtz) - sqrtz) / (sqrtz * sqrtz * sqrtz);
                } else {
                    c2new = 0.5;
                    c3new = 1.0 / 6.0;
                }
            }
        }  // findc2c3


        /// <summary>
        /// Determine the time of flight in COE relevant units that it takes for the body
        /// in orbit to go from position r0 to position r1 in an orbit with given COE.
        ///
        /// </summary>
        /// <param name="r0">from position in orbit</param>
        /// <param name="r1">to position in orbit</param>
        /// <param name="coe">COE struct describing the orbit</param>
        /// <returns>time to move from r0 to r1</returns>
        public static double TimeOfFlight(double3 r0, double3 r1, COE coe)
        {
            const double SMALL = 0; // 1E-7;  parabola less likely than very large units. 

            double3 normal = GravityMath.Matrix3x3TimesVector(coe.rotation, new double3(0, 0, 1));
            // Vallado, Algorithm 11, p126
            double tof;
            double r0r1 = math.length(r0) * math.length(r1);
            double3 r0n = math.normalize(r0);
            double3 r1n = math.normalize(r1);
            double cos_dnu = math.dot(r0n, r1n);
            cos_dnu = Math.Clamp(cos_dnu, -1.0, 1.0);
            double sin_dnu = math.length(math.cross(r0n, r1n));
            sin_dnu = Math.Clamp(sin_dnu, -1.0, 1.0);
            // use the normal to determine if angle is > 180
            //// sin_nu: Need to use direction of flight to pick sign per Algorithm 53
            if (math.dot(math.cross(r0, r1), normal) < 0.0) {
                sin_dnu *= -1.0;
            }
            // GE - precision issue at 180 degrees. Simply return 1/2 the orbit period.
            double r0m = math.length(r0);
            double r1m = math.length(r1);
            if (coe.e < 1.0 && Math.Abs(1.0 + cos_dnu) < 1E-10) {
                double a180 = 0.5f * (r0m + r1m);
                return Math.Sqrt(a180 * a180 * a180 / coe.mu) * Math.PI;
            }
            double k = r0r1 * (1.0 - cos_dnu);
            double l = r0m + r1m;
            double m = r0r1 * (1.0 + cos_dnu);
            double a = (m * k * coe.p) / ((2.0 * m - l * l) * coe.p * coe.p + 2.0 * k * l * coe.p - k * k);
            double f = 1.0 - (r1m / coe.p) * (1.0 - cos_dnu);
            double g = r0r1 * sin_dnu / Math.Sqrt(coe.mu * coe.p);

            double alpha = 1 / a;
            double delta_nu = Math.Atan2(sin_dnu, cos_dnu);
            double fdot = Math.Sqrt(coe.mu / coe.p) * Math.Tan(0.5 * delta_nu) *
                ((1 - cos_dnu) / coe.p - (1 / r0m) - (1.0 / r1m));

            if (alpha > SMALL) {
                // ellipse
                double cos_deltaE = 1 - r0m / a * (1.0 - f);
                cos_deltaE = Math.Clamp(cos_deltaE, -1.0, 1.0);
                double sin_deltaE = -r0r1 * fdot / (Math.Sqrt(coe.mu * a));
                sin_deltaE = Math.Clamp(sin_deltaE, -1.0, 1.0);
                double deltaE = Math.Atan2(sin_deltaE, cos_deltaE);
                tof = g + Math.Sqrt(a * a * a / coe.mu) * (deltaE - sin_deltaE);
            } else if (alpha < -SMALL) {
                // hyperbola
                // NBP: Use fdot to get sign for deltaH Vallado eqns (2-68) and (2-65)
                double sinh_deltaH = -fdot * r0m * r1m / math.sqrt(-coe.mu * coe.a);
                double deltaH = Math.Asinh(sinh_deltaH);
                tof = g + Math.Sqrt(-a * a * a / coe.mu) * (Math.Sinh(deltaH) - deltaH);
                tof = math.abs(tof);
                //UnityEngine.Debug.LogFormat("**Remove ME** tof={0} deltaH={1} cosh_deltaH={2} a={3}", tof, deltaH, cosh_deltaH, a);
            } else {
                // parabola
                double c = Math.Sqrt(r0m * r0m + r1m * r1m - 2.0 * r0r1 * cos_dnu);
                double s = 0.5 * (r0m + r1m + c);
                tof = 2 / 3 * Math.Sqrt(s * s * s / (2.0 * coe.mu)) * (1 - Math.Pow(((s - c) / s), 1.5));
            }
            if ((tof < 0) && (coe.e < 1.0) && (coe.mu > 0)) {
                tof += coe.GetPeriod();
            }
            return tof;
        }

        /// <summary>
        /// Determine if the given velocities v_now and v_next are both oriented in the same
        /// direction around the orbit. 
        /// 
        /// This is used in Lambert transfer calculations to select the transfer option that 
        /// matches the current direction in the orbit
        /// </summary>
        /// <param name="r">position in the orbit</param>
        /// <param name="v_now">current velocity</param>
        /// <param name="v_next">velocity after possible maneuver</param>
        /// <returns>true if v_next is in same direction as v_now</returns>
        public static bool VelocityIsPrograde(double3 r, double3 v_now, double3 v_next)
        {
            // Check that v_next has same direction of angular momentum as initial conditions
            double3 h_initial = math.normalize(math.cross(r, v_now));
            double3 h_man1 = math.normalize(math.cross(r, v_next));
            return (math.dot(h_initial, h_man1) > 0);
        }

        /// <summary>
        /// Compute the 3D velocity for a circular velocity for the given 3D body 
        /// state (r, v) around a mass with specified mu (=GM)
        /// </summary>
        /// <param name="body">struct with r, v state</param>
        /// <param name="mu">center body mass</param>
        /// <returns>3D circular velocity vector</returns>
        public static double3 VelocityForCircularOrbitRelative(GEBodyState body, double mu)
        {
            // determine the magnitude of the velocity vector
            double v_mag = Math.Sqrt(mu / math.length(body.r));

            // get a vector that is in the right direction
            double3 h = math.normalize(math.cross(body.r, body.v));
            double3 v_new = v_mag * math.normalize(math.cross(h, body.r));
            return v_new;
        }

        /// <summary>
        /// Determine the magnitude of a circular orbit for a given radius
        /// around a body of given mu. 
        /// </summary>
        /// <param name="r">agnitude of radius</param>
        /// <param name="mu">mu of center body (GM)</param>
        /// <returns></returns>
        public static double VelocityMagnitudeForCircular(double r, double mu)
        {
            return math.sqrt(mu / r);
        }

        //**************************************************************
        // ANAMOLY stuff
        //**************************************************************

        /// <summary>
        /// Convert the eccentric anomaly (angle from center of ellipse wrt x-axis) to the true anomoly (angle from focus
        /// wrt periapsis/x-axis if e=0). 
        /// 
        /// Equations from front cover of Vallado. 
        /// </summary>
        /// <param name="eValue"></param>
        /// <param name="ecc"></param>
        /// <returns></returns>
        public static double ConvertEtoTrueAnomoly(double eValue, double ecc)
        {
            double taValue = 0.0;
            if (ecc <= 1.0) {
                double denom = 1 - ecc * Math.Cos(eValue);
                double sinTA = Math.Sin(eValue) * Math.Sqrt(1 - ecc * ecc) / denom;
                double cosTA = (Math.Cos(eValue) - ecc) / denom;
                taValue = Math.Atan2(sinTA, cosTA);
            } else {
                throw new System.Exception("Hyperbola not implemented");
            }
            return taValue;
        }

        public static double ConvertEtoMeanAnomoly(double eValue, double ecc)
        {
            return eValue - ecc * Math.Sin(eValue);
        }

        public static double ConvertBToTrueAnomoly(double B, double p, double r)
        {
            // Vallado Algo 6 p 78
            double sinnu = p * B / r;
            double cosnu = (p - r) / r;
            return math.atan2(sinnu, cosnu);
        }

        public static double ConvertTrueAnomolytoE(double nu, double ecc)
        {
            double eValue = 0.0;
            if (ecc <= 1.0) {
                double denom = 1 + ecc * Math.Cos(nu);
                double sinE = Math.Sin(nu) * Math.Sqrt(1 - ecc * ecc) / denom;
                double cosE = (Math.Cos(nu) + ecc) / denom;
                eValue = Math.Atan2(sinE, cosE);
            } else {
                throw new System.Exception("Hyperbola not implemented");
            }
            if (eValue < 0) {
                eValue += 2.0 * Math.PI;
            }
            return eValue;
        }

        private static int LOOP_LIMIT = 200;
        public static double ConvertMeanAnomolyToE(double M, double ecc)
        {
            // Vallado Algorithm 2, p65
            double En = M + ecc;
            if ((M > Math.PI) || ((M > -Math.PI) && (M < 0))) {
                En = M - ecc;
            }
            double Enext = En;
            int loops = 0;
            do {
                En = Enext;
                Enext = Enext + (M - Enext + ecc * Math.Sin(Enext)) / (1 - ecc * Math.Cos(Enext));
            } while ((Math.Abs(Enext - En) > 1E-6) && (loops++ < LOOP_LIMIT));
            if (loops >= LOOP_LIMIT) {
                UnityEngine.Debug.LogWarningFormat("Value M={0} did not converge Enext={1}", M, Enext);
            }
            return Enext;
        }

        public static double ConvertMeanToTrueAnomoly(double M, double ecc)
        {
            double nu = ConvertEtoTrueAnomoly(
                            ConvertMeanAnomolyToE(M, ecc), ecc);
            if (nu < 0)
                nu += 2.0 * math.PI;
            return nu;
        }

        public static double KeplerEqnParabolicForB(double dt, double p, double mu)
        {
            double s = 0.5 * (0.5 * math.PI - Math.Atan(3.0 * math.sqrt(mu / (p * p * p)) * dt));
            double w = Math.Atan(Math.Pow(Math.Tan(s), 1.0 / 3.0));
            double B = 2.0 / math.tan(2.0 * w);
            return B;
        }

        public static double KeplerEqnHyperbolicForH(double M, double e)
        {
            double H;
            if (e < 1.6) {
                if (M > -math.PI && M < 0) {
                    H = M - e;
                } else {
                    H = M + e;
                }
            } else if (e < 3.6) {
                if (math.abs(M) > math.PI) {
                    H = M - math.sign(M) * e;
                } else {
                    H = M / (e - 1.0);
                }
            } else {
                H = M / (e - 1.0);
            }
            double Hnext = H;
            int loops = 0;
            do {
                H = Hnext;
                Hnext = H + (M - e * math.sinh(H) + H) / (e * math.cosh(H) - 1.0);
            } while ((math.abs(Hnext - H) > 1E-6) && (loops++ < LOOP_LIMIT));
            if (loops >= LOOP_LIMIT) {
                H = double.NaN; // not converged, so ripple through a NaN for debugging
            }
            return H;
        }

        public static double ConvertHToTrueAnomoly(double H, double e)
        {
            double sinnu = -math.sinh(H) * math.sqrt(e * e - 1.0) / (1.0 - e * math.cosh(H));
            double cosnu = (math.cosh(H) - e) / (1.0 - e * math.cosh(H));
            return math.atan2(sinnu, cosnu);
        }

        /* -----------------------------------------------------------------------------
         *
         *                           function newtonnu
         *
         *  this function solves keplers equation when the true anomaly is known.
         *    the mean and eccentric, parabolic, or hyperbolic anomaly is also found.
         *    the parabolic limit at 168ø is arbitrary. the hyperbolic anomaly is also
         *    limited. the hyperbolic sine is used because it's not double valued.
         *
         *  author        : david vallado                  719-573-2600   27 may 2002
         *
         *  revisions
         *    vallado     - fix small                                     24 sep 2002
         *
         *  inputs          description                    range / units
         *    ecc         - eccentricity                   0.0  to
         *    nu          - true anomaly                   -2pi to 2pi rad
         *
         *  outputs       :
         *    e0          - eccentric anomaly              0.0  to 2pi rad       153.02 deg
         *    m           - mean anomaly                   0.0  to 2pi rad       151.7425 deg
         *
         *  locals        :
         *    e1          - eccentric anomaly, next value  rad
         *    sine        - sine of e
         *    cose        - cosine of e
         *    ktr         - index
         *
         *  coupling      :
         *    arcsinh     - arc hyperbolic sine
         *    sinh        - hyperbolic sine
         *
         *  references    :
         *    vallado       2013, 77, alg 5
         * --------------------------------------------------------------------------- */

        public static (double M, double E) NewtonNu(double ecc, double nu, double small)
        {
            double sine, cose, cosnu, temp;

            double e0, m;

            // ---------------------  implementation   ---------------------
            e0 = 999999.9;
            m = 999999.9;
            //small = 0.00000001;

            // --------------------------- circular ------------------------
            if (math.abs(ecc) < small) {
                m = nu;
                e0 = nu;
            } else
            // ---------------------- elliptical -----------------------
            if (ecc < 1.0 - small) {
                cosnu = math.cos(nu);
                temp = 1.0 / (1.0 + ecc * cosnu);
                sine = math.sqrt(1.0 - ecc * ecc) * math.sin(nu) * temp;
                cose = (ecc + cosnu) * temp;
                e0 = math.atan2(sine, cose);
                m = e0 - ecc * math.sin(e0);
            } else
            // -------------------- hyperbolic  --------------------
            if (ecc > 1.0 + small) {
                if ((ecc > 1.0) && (math.abs(nu) + 0.00001 < math.PI - math.acos(math.clamp(1.0 / ecc, -1.0, 1.0)))) {
                    sine = math.sqrt(ecc * ecc - 1.0) * math.sin(nu) / (1.0 + ecc * math.cos(nu));
                    e0 = GravityMath.Asinh(sine);
                    m = ecc * math.sinh(e0) - e0;
                }
            } else
            // ----------------- parabolic ---------------------
            if (math.acos(math.clamp(nu, -1.0, 1.0)) < 168.0 * math.PI / 180.0) {
                e0 = math.tan(nu * 0.5);
                m = e0 + (e0 * e0 * e0) / 3.0;
            }

            if (ecc < 1.0) {
                m = m % (2.0 * math.PI);
                if (m < 0.0)
                    m = m + 2.0 * math.PI;
                e0 = e0 % (2.0 * math.PI);
            }
            return (m, e0);
        }  // newtonnu


        /* -----------------------------------------------------------------------------
        *
        *                           function gstime
        *
        *  this function finds the greenwich sidereal time (iau-82).
        *
        *  author        : david vallado                  719-573-2600    1 mar 2001
        *
        *  revisions
        *    vallado     - conversion to c#                              16 Nov 2011
        *   
        *  inputs          description                    range / units
        *    jdut1       - julian date in ut1             days from 4713 bc
        *
        *  outputs       :
        *    gstime      - greenwich sidereal time        0 to 2pi rad
        *
        *  locals        :
        *    temp        - temporary variable for doubles   rad
        *    tut1        - julian centuries from the
        *                  jan 1, 2000 12 h epoch (ut1)
        *
        *  coupling      :
        *    none
        *
        *  references    :
        *    vallado       2013, 188, eq 3-47
        * --------------------------------------------------------------------------- */

        // Sidereal time tells us the Earth position of 0 degrees longitude at the give Julian time
        // This establishes the absolute Earth rotation with respect to the astronomical x-axis
        // Useful when we need to establish an Earth texture or map position in time to plot
        // real-world satellite data from TLEs. 

        /// <summary>
        /// Greenwich sidereal angle offset (radians) at the given Julian date
        /// </summary>
        /// <param name="jdate"></param>
        /// <returns></returns>
        public static double GreenwichSiderealTime(double jdate)
        {
            const double twopi = 2.0 * Math.PI;
            const double deg2rad = Math.PI / 180.0;
            double temp, tut1;

            tut1 = (jdate - 2451545.0) / 36525.0;
            temp = -6.2e-6 * tut1 * tut1 * tut1 + 0.093104 * tut1 * tut1 +
                    (876600.0 * 3600 + 8640184.812866) * tut1 + 67310.54841;  // sec
            temp = (temp * deg2rad / 240.0 % twopi); //360/86400 = 1/240, to deg, to rad

            // ------------------------ check quadrants ---------------------
            if (temp < 0.0)
                temp += twopi;

            return temp;
        }  // gstime


        /// <summary>
		/// Given the state of a ship and planet determine the state info in the RSW frame in which:
		/// R is the radial vector to the planet
		/// S is along track
		/// W is cross track
		/// (see Vallado 3.3 p159 (5th ed))
		///
		/// The position vector defines R
		/// The orbit normal is used to define the axis for cross track.
		/// The cross of R and W defines S. 
		/// </summary>
		/// <param name="ship"></param>
		/// <param name="planet"></param>
		/// <param name="rswState"></param>
        public static void RSWState(ref GEBodyState ship,
                                    ref GEBodyState planet,
                                    double3 orbitNormal,
                                    ref GEBodyState rswState)
        {
            double3 r_xyz = ship.r - planet.r;
            double3 v_xyz = ship.v - planet.v;
            double3 R = math.normalize(r_xyz);
            double3 W = orbitNormal;
            double3 S = math.normalize(math.cross(W, R));
            rswState.r = new double3(math.dot(r_xyz, R), math.dot(r_xyz, S), math.dot(r_xyz, W));
            rswState.v = new double3(math.dot(v_xyz, R), math.dot(v_xyz, S), math.dot(v_xyz, W));
            rswState.t = ship.t;
        }

        /// <summary>
        /// Given a FPA and two position vectors, return the velocity vector direction.
        /// </summary>
        /// <param name="fpa"></param>
        /// <param name="r1"></param>
        /// <param name="r2"></param>
        /// <returns></returns>
        public static double3 VDirFromFPAandRR(double fpa, double3 r1, double3 r2)
        {            // get direction based on FPA. 
            double3 R = math.normalize(r1);
            double3 W = math.normalize(math.cross(r1, r2)); // orbit normal
            double3 S = math.normalize(math.cross(W, R));
            return math.normalize(R * math.sin(fpa) + S * math.cos(fpa));
        }

        /* ------------------------------------------------------------------------------
  //
  //                           function checkhitearth
  //
  //  this function checks to see if the trajectory hits the earth during the
  //    transfer.  It may calculate quicker if done in canonical units.
  //
  //  author        : david vallado                  719-573-2600   14 aug 2017
  //
  //  inputs          description                    range / units
  //    altPad      - pad for alt above surface       er  (km if 3 code changes below)
  //    r1c         - initial position vector of int  er   if km, need to be consitent with all inputs
  //    v1tc        - initial velocity vector of trns er/tu
  //    r2c         - final position vector of int    er
  //    v2tc        - final velocity vector of trns   er/tu
  //    nrev        - number of revolutions           0, 1, 2, ...
  //
  //  outputs       :
  //    hitearth    - is earth was impacted           'y' 'n'
  //    hitearthstr - is earth was impacted           "y - radii" "no"
  //
  //  locals        :
  //    sme         - specific mechanical energy
  //    rp          - radius of perigee               er
  //    a           - semimajor axis of transfer      er
  //    ecc         - eccentricity of transfer
  //    p           - semi-paramater of transfer      er
  //    hbar        - angular momentum vector of
  //                  transfer orbit
  //
  //  coupling      :
  //    dot         - dot product of vectors
  //    mag         - magnitude of a vector
  //    cross       - cross product of vectors
  //
  //  references    :
  //    vallado       2013, 503, alg 60
  //
  // ------------------------------------------------------------------------------*/

        public static bool CheckHitPlanet(
               double planetRadius, double planetMass, double3 r1c, double3 v1tc, double3 r2c, double3 v2tc, int nrev)
        {
            double rp, magh, magv1c, v1c2, a, ainv, ecc, ecosea1, esinea1, ecosea2;

            rp = 0.0;
            ecc = 0.0;
            ainv = 0.0;

            double magr1c = math.length(r1c);
            double magr2c = math.length(r2c);

            bool hitPlanet = false;

            // check whether Lambert transfer trajectory hits the Earth
            if (magr1c < planetRadius || magr2c < planetRadius) {
                // hitting earth already at start or stop point
                hitPlanet = true;
            } else {
                // canonical units  
                double rdotv1c = math.dot(r1c, v1tc);
                double rdotv2c = math.dot(r2c, v2tc);

                // Solve for a 
                magv1c = math.length(v1tc);
                v1c2 = magv1c * magv1c;
                ainv = 2.0 / magr1c - v1c2 / planetMass; // v1c2/3.986004418e5  if non-canonical

                // Find ecos(E) 
                ecosea1 = 1.0 - magr1c * ainv;
                ecosea2 = 1.0 - magr2c * ainv;

                UnityEngine.Debug.LogFormat("rdotv1={0} rdotv2={1}", rdotv1c, rdotv2c);

                // Determine radius of perigee
                // 4 distinct cases pass thru perigee (ignoring apsidal cases already checked above):
                // heading to perigee and ending after perigee
                // nrev > 0
                // both headed away from perigee, but end is closer to perigee
                // both headed toward perigee, but start is closer to perigee
                if ((rdotv1c < 0.0 && rdotv2c > 0.0) || nrev > 0 || (rdotv1c > 0.0 && rdotv2c > 0.0 && ecosea1 < ecosea2) || (rdotv1c < 0.0 && rdotv2c < 0.0 && ecosea1 > ecosea2)) {
                    if (Math.Abs(ainv) <= 1.0e-10) {
                        double3 hbar = math.cross(r1c, v1tc);
                        magh = math.length(hbar); // find h magnitude
                        rp = magh * magh * 0.5;    // parabola (from DAV code)
                    } else {
                        a = 1.0 / ainv;  // elliptical or hyperbolic orbit
                        if (ainv > 0.0) {
                            esinea1 = rdotv1c / Math.Sqrt(planetMass * Math.Abs(a)); // for elliptical (3.986004418e5 * Math.Abs(a)) if non-cannonical
                            ecc = Math.Sqrt(ecosea1 * ecosea1 + esinea1 * esinea1);
                        } else {
                            esinea1 = rdotv1c / Math.Sqrt(planetMass * Math.Abs(-a)); // for hyperbolic  (3.986004418e5 * Math.Abs(-a)) if non-cannonical
                            ecc = Math.Sqrt(ecosea1 * ecosea1 - esinea1 * esinea1);
                        }
                        rp = a * (1.0 - ecc);

                        if (ecc < 1.0) {
                            rp = a * (1.0 - ecc);
                        } else {
                            // hyperbolic heading towards the earth
                            if (rdotv1c < 0.0 && rdotv2c > 0.0) {
                                rp = a * (1.0 - ecc);
                            } else if (rdotv1c < 0.0 && rdotv2c < 0.0) {
                                // NBP - both heading inwards
                                rp = magr2c;
                            } else if (rdotv1c > 0.0 && rdotv2c > 0.0) {
                                // NBP - both heading outwards
                                rp = magr1c;
                            }
                        }
                    }
                    if (rp < planetRadius) {
                        hitPlanet = true;
                    }
                }// end of perigee check
            } // end of "hitting Earth surface?" tests
              //Debug.LogFormat("Hit:{0} :r1={1} v1={2} r2={3} v2={4} ecc={5} rp={6} ainv={7}", 
              //    hitPlanet, r1c, v1tc, r2c, v2tc, ecc, rp, ainv);
            return hitPlanet;

        } // checkhitearth

        /// <summary>
        /// Determine the orbit normal for a launch from an point at the given latitude
        /// into an orbit of specified inclination. 
        /// 
        /// This assumes that a check that inclination > latitude has already been done. 
        /// </summary>
        /// <param name="targetInclDeg"></param>
        /// <param name="r"></param>
        /// <param name="latitudeDeg"></param>
        /// <returns>normal to orbit plane</returns>
        public static double3 LaunchPlane(double targetInclDeg, double3 r, double latitudeDeg)
        {
            double3 earthAxis = new double3(0, 0, 1); // XY mode
            r = math.normalize(r); ;

            double incl = math.radians(targetInclDeg);
            // spherical trig. Vallado p338 and Appendix C
            double latAbs = math.abs(math.radians(latitudeDeg));
            double sinBeta = math.cos(incl) / math.cos(latAbs);
            sinBeta = math.clamp(sinBeta, -1.0f, 1.0f);
            double beta = math.asin(sinBeta);
            // get local x, n on surface of earth (y is same as r, local vertical)
            double3 r_y = math.project(r, earthAxis);
            double3 n;
            if (math.length(r_y) < SMALL_E) {
                n = earthAxis;
            } else {
                n = r_y - math.dot(r_y, r) * r;
            }
            n = math.normalize(n);
            // find plane of launch. Rotate n frame by betaCompl
            double betaComlp = 0.5 * math.PI - beta;
            float3 n_f3 = new float3(n);
            float3 r_f3 = new float3(r);
            quaternion q = quaternion.AxisAngle(r_f3, (float)(-betaComlp));

            float3 n_vec3 = math.mul(q, n_f3);

            //Debug.LogFormat("y={0} n={1} beta={2} (deg) |n|={3} angle={4}",
            //    r, n.normalized, betaDeg, n.magnitude, Vector3.Angle(n,r));

            return math.normalize(n_vec3);
        }

        public static double FlightPathAngle(double3 r, double3 v)
        {
            // Adapted from AstroLib rv2flt
            double3 h = math.cross(r, v);
            double hmag = math.length(h);
            double rdotv = math.dot(r, v);
            double fpav = math.atan2(hmag, rdotv);
            return math.PI * 0.5 - fpav;
        }

        /// <summary>
        /// Determine the SOI radius in internal physics units.
        /// </summary>
        /// <param name="planetMass"></param>
        /// <param name="moonMass"></param>
        /// <param name="moonSemiMajorAxis"></param>
        /// <returns></returns>
        public static double SoiRadius(double planetMass, double moonMass, double moonSemiMajorAxis)
        {
            // mass scaling will cancel in this ratio
            return math.pow(moonMass / planetMass, 0.4) * moonSemiMajorAxis;
        }
    }
}

