using System;
using System.Diagnostics;
using Unity.Mathematics;


namespace GravityEngine2 {
    /// <summary>
    /// Gravity engine uses NBody units (nbu) (https://en.wikipedia.org/wiki/N-body_units) internally.
    /// This allows the numerical integration to work with values without worring about loss
    /// of precision due to odd choices of scale (e.g. galaxy simulation in cm).
    ///
    /// These units rescale such that inside GECore: M=1 R=1 G=1
    ///   M => All world masses are scaled by the total mass of the system
    ///   R => Divide all distance by the size of the reference orbit
    ///        (in true nbu this would be the virial radius of a cluster of bodies, since the
    ///         lineage of nbu is stellar cluster modelling)
    ///   G => this is a choice. By choosing G=1, time scale is affected.
    ///
    /// See https://nbodyphysics.com/ge2/html/scaling.html
    /// 
    /// Since this is always somewhat confusing, consider a concrete example:
    /// Earth satellite, in orbit of 7000km in SI units.
    /// Me = 5.972E24 kg, R=7E6 m, V=7531 m/s G_si = 6.674E-11 m^3/(kg s^2)
    ///
    /// World => GE
    ///     Earth mass => Me/Me = 1
    ///     Orbit radius = Rorbit/Rref = 1
    ///     V * Sqrt(R/(G_si M)) = 7512 * 1.33E-4 = 1
    ///
    /// GE => World
    ///    Mass => * M
    ///    Distance => *R
    ///    Velocity => * Sqrt(G_si M/R)
    ///    Time => * Sqrt(R^3/(G_si M)) 1 s => 931.8 world sec
    ///
    /// One orbit period for R=1, M=1, G=1 takes T=2 Pi. The integration step size in GE is typically
    /// chosen as 1000 steps per orbit or 2 Pi/1000 = 0.00628. This corresponds to a world time of 5.85 sec.
    ///
    /// The orbit above has a period of 5851 sec. or about 1.63 hours.
    /// 
    /// </summary>
    public class GBUnits {
        /// <summary>
        /// Units       Length Unit     Time Unit   Velocity
        /// DL          
        /// SI              m               sec.    m/sec
        /// SI_km           km              sec.    km/sec
        /// AU              1 AU            sec.    km/sec
        /// CR3BP           1.0 
        /// 
        /// CR3BP is scaled for a specific circular restricted 3-body system in which the
        /// scale length has been set to one, and the timescale (inverse of angular freq.
        /// of the system, which is period/2Pi). The position and velocity are defined in the
        /// co-rotating coordinate system.
        /// 
        /// </summary>
        public enum Units { DL, SI, SI_km, AU, CR3BP_SCALE };

        public const double earthMassKg = 5.9722E24;
        public const double earthRadiusKm = 6371.0; // average radius

        public const double earthRadiusM = earthRadiusKm * 1E3;

        public const double g_earth = 9.80665; // m/s^2

        public const double earthRotationRadSec = 2.0 * math.PI / SECS_PER_SIDEREAL_DAY;
        public const double moonRadiusKm = 1735.5;

        public const double moonRadiusM = moonRadiusKm * 1E3;
        public const double moonMassKg = 7.342E22;
        public const double moon_g = 1.625; // m/s^2 
        public const double moonMeanOrbitRadiusKm = 384400;

        public const double SUN_MASS = 1.98841E30; // kg
        public const double SUN_RADIUS_KM = 6.957E5;
        public const double SGP_radiusearthkm = 6378.135;
        public const double MARS_MASS = 6.41691e23;
        public const double JUPITER_MASS = 1.89812E27;
        public const double SATURN_MASS = 5.68317e26;

        // G (SI) is in units of N L^2/kg^2 or L^3/(kg^2 s^2)
        public const double NEWTONS_G_SI = 6.67430E-11;
        public const double NEWTONS_G_KM = NEWTONS_G_SI * 1E-9;   // m -> km is  (m * km/m)
        public const double NEWTONS_G_KMHR = 864989280000;
        public const double NEWTONS_G_AUYR = 1.98534641318525E-05;

        public const double earthMu = earthMassKg * NEWTONS_G_SI;
        public const double moonMu = moonMassKg * NEWTONS_G_SI;
        public const double earthMuKm = earthMu * 1E-9;
        public const double moonMuKm = moonMu * 1E-9;

        public const double SEC_PER_MIN = 60.0;
        public const double MIN_PER_SEC = 1.0 / 60.0;

        public const double SECS_PER_SIDEREAL_DAY = 86164.1;
        public const double SECS_PER_SOLAR_DAY = 86400.0;

        public const double kmToSI = 1E3;
        public const double SItoKm = 1E-3;

        // Earth radius units??
        private static double[] gForUnits = new double[] { 1.0, NEWTONS_G_SI, NEWTONS_G_KM, NEWTONS_G_AUYR, 1.0 };


        public struct GEScaler {
            // Keep track of world units. 
            private Units worldUnits;

            // conversion factors
            private double worldToGELen, worldToGEMass, worldToGEVelocity, worldToGETime;
            private double geToWorldLen, geToWorldMass, geToWorldVelocity, geToWorldTime;

            public GEScaler(Units defaultWorldUnits, double massScale, double lenScale)
            {
                worldUnits = defaultWorldUnits;
                // precompute the scaling factors
                double g = gForUnits[(int)worldUnits];
                worldToGELen = 1.0 / lenScale;
                geToWorldLen = lenScale;
                worldToGEMass = 1.0 / massScale;
                geToWorldMass = massScale;
                worldToGEVelocity = math.sqrt(lenScale / (g * massScale));
                geToWorldVelocity = 1.0 / worldToGEVelocity;
                worldToGETime = math.sqrt((g * massScale) / (lenScale * lenScale * lenScale));
                geToWorldTime = 1.0 / worldToGETime;
            }

            public GEScaler(Units defaultWorldUnits, CR3BP.CR3BPSystemData cr3bpSys)
            {
                worldUnits = defaultWorldUnits;

                if (defaultWorldUnits == Units.CR3BP_SCALE) {
                    worldToGELen = 1.0;
                    geToWorldLen = 1.0;
                    worldToGEMass = 1.0;
                    geToWorldMass = 1.0;
                    worldToGEVelocity = 1.0;
                    geToWorldVelocity = 1.0;
                    worldToGETime = 1.0 / cr3bpSys.t_unit;
                    geToWorldTime = cr3bpSys.t_unit;
                } else if (defaultWorldUnits == Units.SI_km) {
                    // precompute the scaling factors
                    double g = gForUnits[(int)worldUnits];
                    worldToGELen = 1.0 / cr3bpSys.len_unit;
                    geToWorldLen = cr3bpSys.len_unit;
                    worldToGEMass = 1.0 / cr3bpSys.totalMass;
                    geToWorldMass = cr3bpSys.totalMass;
                    worldToGEVelocity = math.sqrt(geToWorldLen / (g * geToWorldMass));
                    geToWorldVelocity = 1.0 / worldToGEVelocity;
                    worldToGETime = 1.0 / cr3bpSys.t_unit;
                    geToWorldTime = cr3bpSys.t_unit;
                } else {
                    throw new Exception("Must be CR3BP or SI_km as world units");
                }
            }

            public Units WorldUnits()
            {
                return worldUnits;
            }

            public double ScaleMassWorldToGE(double m)
            {
                return m * worldToGEMass;
            }

            public double ScaleMassGEToWorld(double m)
            {
                return m * geToWorldMass;
            }

            public double ScaleMuGEToWorld(double m)
            {
                return m * geToWorldMass * gForUnits[(int)worldUnits];
            }

            public double ScaleTimeWorldToGE(double t)
            {
                return t * worldToGETime;
            }

            public double ScaleTimeGEToWorld(double t)
            {
                return t * geToWorldTime;
            }

            public double ScaleLenWorldToGE(double r)
            {
                return r * worldToGELen;
            }

            public double ScaleLenGEToWorld(double r)
            {
                return r * geToWorldLen;
            }

            public double ScaleVelocityWorldToGE(double v)
            {
                return v * worldToGEVelocity;
            }

            public double ScaleVelocityGEToWorld(double v)
            {
                return v * geToWorldVelocity;
            }

            public double ScaleAccelWorldToGE(double a)
            {
                return a * worldToGEVelocity / worldToGETime;
            }

            public double ScaleAccelGEToWorld(double a)
            {
                return a * geToWorldVelocity / geToWorldTime;
            }


            public void ScaleStateWorldToGE(ref GEBodyState state)
            {
                state.r *= worldToGELen;
                state.v *= worldToGEVelocity;
                state.t *= worldToGETime;
            }

            public void ScaleStateGEToWorld(ref GEBodyState state)
            {
                state.r *= geToWorldLen;
                state.v *= geToWorldVelocity;
                state.t *= geToWorldTime;
            }

            override
            public string ToString()
            {
                string s = "";
                s += string.Format("         Len       Vel        Time        Mass\n");
                s += string.Format("W2G  {0:G8} {1:G8} {2:G8}  {3:G8}\n",
                    worldToGELen, worldToGEVelocity, worldToGETime, worldToGEMass);
                s += string.Format("G2W  {0:G8} {1:G8} {2:G8}  {3:G8}\n\n",
                    geToWorldLen, geToWorldVelocity, geToWorldTime, geToWorldMass);

                return s;
            }

        }

        public static double GForUnits(Units units)
        {
            return gForUnits[(int)units];
        }

        public static (double gravG, double geTimePerGameSec)
                      GetGandTimescale(Units units, double orbitScale, double orbitMass, double gameSecPerOrbit)
        {
            // F = m1 a = G m1 m2/r^2  =>  G has units of (L^3)/(time^2 mass)
            double g = gForUnits[(int)units];
            // Length adjustment: Want units where a typical orbit has length 1. 
            // orbitScale gives the length we want to be 1 in GE distance, so will be dividing every position
            // by this quantity. Say it's 7000km then we want to convert G in km to 
            g *= 1.0 / (orbitScale * orbitScale * orbitScale);
            double period = 2.0 * Math.PI * Math.Sqrt(1.0 / (g * orbitMass));

            // Time adjustment: We want a game time of gameSecPerOrbit
            // The natural orbit period (in seconds) is T = 2 Pi sqrt(a^3/(GM))
            // where we have chosen to make a=1 by normalizing orbit scale. 
            // With current G adjusted for L=1 (say G_L) T_L = 2 Pi sqrt( 1/G_L M)
            //
            // Concrete example: DL with massScale = 1000, orbitScale of 100 and gameSecPerOrbit of 30
            //
            // Natural orbit period in DL units is T = 2 Pi Sqrt( 100^3/1000) = 2 Pi Sqrt(10^6) = 2000 Pi = 6283
            //
            // In GE we want the distance to be 100x smaller
            //  G goes like  a (L/T^2) = G M/L^2 => L^3 M/T^2
            // so the corresponding xform for G is (1/orbitScale)^3  => g = 1E-9
            //
            // Since we have normalized L the period eqn is now T = 2 Pi Sqrt(1/(g M));
            //
            // We want this to take 30 game seconds, so each game second is T/30 
            // 
            // We want this scaled to be equal to gameSecPerOrbit. i.e. gameSecPerOrbit = gameSecPerGETime * t_L

            double geTimePerGameSec = period / gameSecPerOrbit; // want to report the inverse, so flip
            return (g, geTimePerGameSec);
        }

        private static string[] distanceSF = { "(DL)", "(m)", "(km)", "(AU)", "(CR3BP)" };
        public static string DistanceShortForm(Units unit)
        {
            return distanceSF[(int)unit];
        }

        private static string[] velocitySF = { "(DL/s)", "(m/s)", "(km/s)", "(AU/s)", "(CR3BP)" };
        public static string VelocityShortForm(Units unit)
        {
            return velocitySF[(int)unit];
        }

        private static string[] massSF = { "(DL)", "(kg)", "(kg)", "(kg)", "(CR3BP)" };
        public static string MassShortForm(Units unit)
        {
            return massSF[(int)unit];
        }

        public const double mTokm = 0.001;
        public const double AU_m = 1.495979e+11;

        //! conversion matrix from (row) to (column)
        private static double[,] distanceConvert = new double[4, 4] {
                        { 1.0, 1.0, 1.0, 1.0 },        // from DL
                        { 1.0, 1.0, mTokm, 1.0/AU_m },      // from m
                        { 1.0, 1.0/mTokm, 1.0, 1.0/(AU_m*mTokm) },  // from km
                        { 1.0, AU_m, AU_m*mTokm, 1.0 } // AU
            };

        public static double DistanceConversion(Units from, Units to, CR3BP.CR3BPSystemData crsbpSysData = null)
        {
            double d = 1.0;
            if (from == to)
                return d;
            if (crsbpSysData != null) {
                if (from == Units.CR3BP_SCALE) {
                    // all CR3BP systems map to km
                    d = crsbpSysData.len_unit * distanceConvert[(int)Units.SI_km, (int)to];
                } else if (to == Units.CR3BP_SCALE) {
                    d = distanceConvert[(int)from, (int)Units.SI_km] / crsbpSysData.len_unit;
                } else {
                    d = distanceConvert[(int)from, (int)to];
                }
            } else {
                d = distanceConvert[(int)from, (int)to];
            }
            return d;
        }

        // all in seconds for now
        private static double[,] velocityConvert = new double[4, 4] {
                        { 1.0, 1.0, 1.0, 1.0 },        // from DL
                        { 1.0, 1.0, mTokm, 1.0/AU_m },      // from m
                        { 1.0, 1.0/mTokm, 1.0, 1.0/(AU_m*mTokm) },  // from km
                        { 1.0, AU_m, AU_m*mTokm, 1.0 } // AU
            };

        public static double VelocityConversion(Units from, Units to, CR3BP.CR3BPSystemData crsbpSysData = null)
        {
            double v = 1.0;
            if (from == to)
                return v;
            if (crsbpSysData != null) {
                UnityEngine.Debug.LogWarning("Need correct conversion");
                if (from == Units.CR3BP_SCALE) {
                    // all CR3BP systems map to km
                    v = crsbpSysData.len_unit * velocityConvert[(int)Units.SI_km, (int)to];
                } else if (to == Units.CR3BP_SCALE) {
                    v = velocityConvert[(int)from, (int)Units.SI_km] / crsbpSysData.len_unit;
                } else {
                    v = velocityConvert[(int)from, (int)to];
                }
            } else {
                v = velocityConvert[(int)from, (int)to];
            }
            return v;
        }

        private static double[,] muConvert = new double[4, 4] {
                        { 1.0, 0.0, 0.0, 0.0 },  // from DL
                        { 0.0, 1.0, 1E-9, 0.0 },  // from m
                        { 0.0, 1E9, 1.0, 0.0 },  // from km
                        { 0.0, 0.0, 0.0, 1.0 }   // TODO: AU
            };

        public static double MuConversion(Units from, Units to)
        {
            double mu = muConvert[(int)from, (int)to];
            if (mu == 0)
                throw new Exception("Mu conversion is 0");

            return mu;
        }

    }
}
