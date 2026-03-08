using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace GravityEngine2 {
    /// <summary>
	/// Circular Restricted Three Body Problem
    ///
    /// Utility functions to assist in converting from the inertial coordinates into the
    /// co-rotating coordinates.
    ///
    /// The CR3BP integrator follows the usual convention of normalizing the separation between the bodies
    /// to 1 and the total mass to one, with a mass fraction of the secondary body being mu. (Hence the
    /// primary is 1-mu). 
    ///
    /// In the corotating coordinates the CM of the system is at zero and the position of the primary is
    /// (-mu, 0, 0) and the secondary is (1-mu, 0, 0).
    ///
    /// Unit of mass: m1 + m2
    /// Unit of Length: d(m1, m2)
    /// Unit of Time: 2 Pi for one orbit of the secondary
    /// This results in G=1.
    ///
    /// A good reference is 2.2 of Koon, Lo, Marsden & Ross (KLMR). Available as a free PDF at
    /// https://ross.aoe.vt.edu/books/Ross_3BodyProblem_Book_2022.pdf
    /// 
	/// </summary>
    public class CR3BP {
        // Systems from JPL 3B database. enums must align with order below
        public enum TB_System {
            EARTH_MOON, SUN_EARTH, SUN_MARS,
            MARS_PHOBIS, JUPITER_EUROPA, SATURN_ENCELADUS, SATURN_TITAN,
            CUSTOM
        };

        public class CR3BPSystemData {
            public string name;
            // mass ratio of the primaries, m2 / (m1 + m2).
            public double massRatio;
            public double radiusSecondary;
            public double3 L1;
            public double3 L2;
            public double3 L3;
            public double3 L4;
            public double3 L5;
            // length unit in km (distance between the primaries).
            public double len_unit;
            // time unit in seconds (inverse of the angular frequency of the system). This is not the period
            // to get period multiply by 2 Pi or use Period()
            public double t_unit;
            // total mass in kg
            public double totalMass;

            public double Period()
            {
                return 2.0 * math.PI * t_unit;
            }

            public double3 ReferencePoint(ReferencePoint refPoint)
            {
                double3 p = double3.zero;
                switch (refPoint) {
                    case CR3BP.ReferencePoint.L1:
                        p = L1;
                        break;
                    case CR3BP.ReferencePoint.L2:
                        p = L2;
                        break;
                    case CR3BP.ReferencePoint.L3:
                        p = L3;
                        break;
                    case CR3BP.ReferencePoint.L4:
                        p = L4;
                        break;
                    case CR3BP.ReferencePoint.L5:
                        p = L1;
                        break;
                    case CR3BP.ReferencePoint.PRIMARY:
                        p = new double3(-massRatio, 0, 0);
                        break;
                    case CR3BP.ReferencePoint.SECONDARY:
                        p = new double3(1.0 - massRatio, 0, 0);
                        break;
                }
                return p;
            }
        }

        public enum ReferencePoint { NONE, PRIMARY, SECONDARY, L1, L2, L3, L4, L5 };

        // Data from JPL Web Requests
        private static CR3BPSystemData[] cr3bpSystems = new CR3BPSystemData[]{
            new CR3BPSystemData() {
            name = "Earth-Moon",
            massRatio = 0.012150585609624,
            radiusSecondary = 1737.1,
            len_unit = 389703.264829278,
            t_unit = 382981.289129055,
            L1 = new double3(0.836915125772357,0,0),
            L2 = new double3(1.15568216544488,0,0),
            L3 = new double3(-1.00506264581028,0,0),
            L4 = new double3(0.487849414390376,0.866025403784439,0),
            L5 = new double3(0.487849414390376,-0.866025403784439,0),
            totalMass = GBUnits.moonMassKg + GBUnits.earthMassKg
            },
            new CR3BPSystemData() {
            name = "Sun-EarthMoon", // KLMR
            massRatio = 3.036-06,
            radiusSecondary = 6378,
            len_unit = 149597870.7,
            t_unit = 5022635.34820215,
            L1 = new double3(0.989970922056916,0,0),
            L2 = new double3(1.01009043578556,0,0),
            L3 = new double3(-1.00000127258333,0,0),
            L4 = new double3(0.4999969458,0.866025403784439,0),
            L5 = new double3(0.4999969458,-0.866025403784439,0),
            totalMass = GBUnits.SUN_MASS
            },
            new CR3BPSystemData() {
            name = "Sun-Mars",
            massRatio = 3.22715499610172E-07,
            radiusSecondary = 3389.5,
            len_unit = 208321281.703548,
            t_unit = 8253621.92588451,
            // v_unit = 
            L1 = new double3(0.995251326846092,0,0),
            L2 = new double3(1.00476310673934,0,0),
            L3 = new double3(-1.00000013446479,0,0),
            L4 = new double3(0.4999996772845,0.866025403784439,0),
            L5 = new double3(0.4999996772845,-0.866025403784439,0),
            totalMass = GBUnits.SUN_MASS
            },
            new CR3BPSystemData() {
            name = "Mars-Phobos",
            massRatio = 1.61108140440963E-08,
            radiusSecondary = 11.267,
            len_unit = 9468.25503898377,
            t_unit = 4451.83899462989,
            L1 = new double3(0.998249821501471,0,0),
            L2 = new double3(1.00175219070903,0,0),
            L3 = new double3(-1.00000000671284,0,0),
            L4 = new double3(0.499999983889186,0.866025403784439,0),
            L5 = new double3(0.499999983889186,-0.866025403784439,0),
            totalMass = GBUnits.MARS_MASS
            },
            new CR3BPSystemData() {
            name = "Jupiter-Europa",
            massRatio = 2.52801752854E-05,
            radiusSecondary = 1560.8,
            len_unit = 668518.69872684,
            t_unit = 48562.3417930319,
            L1 = new double3(0.979764104589418,0,0),
            L2 = new double3(1.02046138598162,0,0),
            L3 = new double3(-1.00001053340637,0,0),
            L4 = new double3(0.499974719824715,0.866025403784439,0),
            L5 = new double3(0.499974719824715,-0.866025403784439,0),
            totalMass = GBUnits.JUPITER_MASS
            },
            new CR3BPSystemData() {
            name = "Saturn-Enceladus",
            massRatio = 1.9011097358926E-07,
            radiusSecondary = 252.1,
            len_unit = 238529.333386019,
            t_unit = 18913.2798604104,
            L1 = new double3(0.99601827654077,0,0),
            L2 = new double3(1.00399193980005,0,0),
            L3 = new double3(-1.00000007921291,0,0),
            L4 = new double3(0.499999809889026,0.866025403784439,0),
            L5 = new double3(0.499999809889026,-0.866025403784439,0),
            totalMass = GBUnits.SATURN_MASS
            },
            new CR3BPSystemData() {
            name = "Saturn-Titan",
            massRatio = 0.000236639315833148,
            radiusSecondary = 2574.7,
            len_unit = 1195677.15191758,
            t_unit = 212238.272684231,
            L1 = new double3(0.957496173324114,0,0),
            L2 = new double3(1.04325642134739,0,0),
            L3 = new double3(-1.00009859971421,0,0),
            L4 = new double3(0.499763360684167,0.866025403784439,0),
            L5 = new double3(0.499763360684167,-0.866025403784439,0),
            totalMass = GBUnits.SATURN_MASS
            },
        };


        public static CR3BPSystemData SystemData(TB_System cr3bpType)
        {
            CR3BPSystemData cr3bpData;
            if (cr3bpType != TB_System.CUSTOM) {
                // fill in masses and set units to SI_km
                cr3bpData = cr3bpSystems[(int)cr3bpType];
                return cr3bpData;
            } else {
                return null;
            }
        }

        /// <summary>
        /// Configure the parameters for the GSBodies for a CR3BP system based on the default units of the
        /// GSController. 
        /// </summary>
        /// <param name="cr3bp"></param>
        /// <param name="primary"></param>
        /// <param name="secondary"></param>
        public static void GSBodiesSetup(TB_System cr3bp, GSBody primary, GSBody secondary)
        {
            CR3BPSystemData cr3bpSysData = SystemData(cr3bp);
            // primary
            primary.bodyInitData.initData = BodyInitData.InitDataType.RV_ABSOLUTE;
            primary.bodyInitData.units = GBUnits.Units.CR3BP_SCALE;
            double mu = cr3bpSysData.massRatio;
            primary.mass = 1.0 - mu;
            primary.bodyInitData.r = new double3(-mu, 0, 0);
            primary.bodyInitData.v = double3.zero;
            // secondary
            secondary.bodyInitData.initData = BodyInitData.InitDataType.RV_ABSOLUTE;
            secondary.bodyInitData.units = GBUnits.Units.CR3BP_SCALE;
            secondary.mass = mu;
            secondary.bodyInitData.r = new double3(1.0 - mu, 0, 0);
            secondary.bodyInitData.v = double3.zero;
        }

        /// <summary>
        /// Convert normalized CR3BP state value to SI_km
        /// 
        /// </summary>
        /// <param name="bodyState"></param>
        /// <param name="fromUnits"></param>
        /// <param name="sysData"></param>
        public static void NormToSIkm(ref GEBodyState bodyState, CR3BPSystemData sysData)
        {
            // scale to DL
            bodyState.r *= sysData.len_unit;
            bodyState.v *= sysData.len_unit / sysData.t_unit; ;
            bodyState.t *= sysData.t_unit;
        }

        /// <summary>
        /// Convert SIkm to normalized CR3BP dimensionless units. 
        /// </summary>
        /// <param name="bodyState"></param>
        /// <param name="toUnits"></param>
        /// <param name="sysData"></param>
        public static void SIkmToNorm(ref GEBodyState bodyState, CR3BPSystemData sysData)
        {
            double v_unit = sysData.len_unit / sysData.t_unit;
            bodyState.r /= sysData.len_unit;
            bodyState.v /= v_unit;
            bodyState.t /= sysData.t_unit;
        }

        /// <summary>
        /// Transform rotating NORMALIZED R, V to inertial R, V
        /// 
        /// See KLMR
        /// 
        /// This assumes CR3BP scaling in which the period of orbit for the secondary is 2 Pi
        /// </summary>
        /// <param name="r"></param>
        /// <param name="v"></param>
        /// <param name="t"></param>
        /// <returns></returns>
        public static (double3 r, double3 v) FrameRotatingToInertial(double3 r, double3 v, double t)
        {
            double cost = math.cos(t);
            double sint = math.sin(t);
            // (2.3.2)
            double3 r_i = new double3(cost * r.x - sint * r.y,
                                       sint * r.x + cost * r.y,
                                       r.z);
            // (2.3.3)
            double3 v_tmp = new double3(v.x - r.y, v.y + r.x, v.z);
            double3 v_i = new double3(cost * v_tmp.x - sint * v_tmp.y,
                                      sint * v_tmp.x + cost * v_tmp.y,
                                      v_tmp.z);
            return (r_i, v_i);
        }

        /// <summary>
        /// Transform rotating NORMALIZED R, V to inertial R, V
        /// 
        /// See KLMR and then do the inversion. 
        /// 
        /// This assumes CR3BP scaling in which the period of orbit for the secondary is 2 Pi
        ///
        ///   A(t)^-1 R = r
        /// </summary>
        /// <param name="r"></param>
        /// <param name="v"></param>
        /// <param name="t"></param>
        /// <returns></returns>
        public static (double3 r, double3 v) FrameInertialToRotating(double3 r, double3 v, double t)
        {
            double cost = math.cos(t);
            double sint = math.sin(t);
            // Inverse of A(t)
            double3 r_i = new double3(cost * r.x + sint * r.y,
                                       -sint * r.x + cost * r.y,
                                       r.z);
            // take derivative of A(t)^-1 R = r
            double3 vterm2 = new double3(cost * v.x + sint * v.y,
                                       -sint * v.x + cost * v.y,
                                       v.z);
            double3 vterm1 = new double3(-sint * r.x + cost * r.y,
                                      -cost * r.x - sint * r.y,
                                      0);
            return (r_i, vterm1 + vterm2);
        }

        public static void InertialToRotatingVec3(ref UnityEngine.Vector3[] r, double t)
        {
            float cost = Mathf.Cos((float)t);
            float sint = Mathf.Sin((float)t);
            float x, y;
            for (int i = 0; i < r.Length; i++) {
                x = cost * r[i].x + sint * r[i].y;
                y = -sint * r[i].x + cost * r[i].y;
                r[i].x = x;
                r[i].y = y;
            }
        }

        //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        // RK4
        // Use the RK4Data from the Integrator class
        // By construction there can be NO massive bodies other than the primary/secondary
        // due to the CR3BP
        //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        public static double RK4Step(ref GEPhysicsCore.GEPhysicsJob geJob)
        {
            double dt = geJob.parms[GECore.DT_PARAM];
            double t = geJob.parms[GECore.T_PARAM];
            geJob.parms[GECore.T_PARAM] += dt;
            double dtHalf = 0.5 * dt;
            int i;

            double mu = geJob.bodies.mu[1];

            // Massless
            if (geJob.lenMasslessBodies.Value > 0) {
                EvaluateMasslessRK4(t, dt, mu,
                        ref geJob, ref geJob.bodies.r, ref geJob.bodies.v, ref geJob.rk4Data.r_k0, ref geJob.rk4Data.v_k0);
                for (int a = 0; a < geJob.lenMasslessBodies.Value; a++) {
                    i = geJob.masslessBodies[a];
                    geJob.rk4Data.r_tmp[i] = geJob.bodies.r[i] + dtHalf * geJob.rk4Data.r_k0[i];
                    geJob.rk4Data.v_tmp[i] = geJob.bodies.v[i] + dtHalf * geJob.rk4Data.v_k0[i];
                }
                EvaluateMasslessRK4(t + dtHalf, dt, mu,
                        ref geJob, ref geJob.rk4Data.r_tmp, ref geJob.rk4Data.v_tmp, ref geJob.rk4Data.r_k1, ref geJob.rk4Data.v_k1);
                for (int a = 0; a < geJob.lenMasslessBodies.Value; a++) {
                    i = geJob.masslessBodies[a];
                    geJob.rk4Data.r_tmp[i] = geJob.bodies.r[i] + dtHalf * geJob.rk4Data.r_k1[i];
                    geJob.rk4Data.v_tmp[i] = geJob.bodies.v[i] + dtHalf * geJob.rk4Data.v_k1[i];
                }
                EvaluateMasslessRK4(t + dtHalf, dt, mu,
                        ref geJob, ref geJob.rk4Data.r_tmp, ref geJob.rk4Data.v_tmp, ref geJob.rk4Data.r_k2, ref geJob.rk4Data.v_k2);
                for (int a = 0; a < geJob.lenMasslessBodies.Value; a++) {
                    i = geJob.masslessBodies[a];
                    geJob.rk4Data.r_tmp[i] = geJob.bodies.r[i] + dt * geJob.rk4Data.r_k2[i];
                    geJob.rk4Data.v_tmp[i] = geJob.bodies.v[i] + dt * geJob.rk4Data.v_k2[i];
                }
                EvaluateMasslessRK4(t + dt, dt, mu, ref geJob,
                        ref geJob.rk4Data.r_tmp, ref geJob.rk4Data.v_tmp, ref geJob.rk4Data.r_k3, ref geJob.rk4Data.v_k3);
                for (int a = 0; a < geJob.lenMasslessBodies.Value; a++) {
                    i = geJob.masslessBodies[a];
                    geJob.bodies.r[i] += dt / 6.0 * (geJob.rk4Data.r_k0[i] + 2.0 * geJob.rk4Data.r_k1[i] + 2.0 * geJob.rk4Data.r_k2[i] + geJob.rk4Data.r_k3[i]);
                    geJob.bodies.v[i] += dt / 6.0 * (geJob.rk4Data.v_k0[i] + 2.0 * geJob.rk4Data.v_k1[i] + 2.0 * geJob.rk4Data.v_k2[i] + geJob.rk4Data.v_k3[i]);
                }
            }
            return geJob.parms[GECore.T_PARAM];
        }


        private static void EvaluateMasslessRK4(double t, double dt,
                                        double mu,
                                        ref GEPhysicsCore.GEPhysicsJob geJob,
                                        ref NativeArray<double3> r,
                                        ref NativeArray<double3> v,
                                        ref NativeArray<double3> dr_out,
                                        ref NativeArray<double3> dv_out)
        {
            for (int a = 0; a < geJob.lenMasslessBodies.Value; a++) {
                int i = geJob.masslessBodies[a];
                // JPL equations (https://ssd.jpl.nasa.gov/tools/periodic_orbits.html#/intro)
                double d = math.sqrt((r[i].x + mu) * (r[i].x + mu) + r[i].y * r[i].y + r[i].z * r[i].z);
                double rr = math.sqrt((r[i].x - 1.0 + mu) * (r[i].x - 1.0 + mu) + r[i].y * r[i].y + r[i].z * r[i].z);
                dr_out[i] = v[i]; //  xdot = v
                double xddot = 2.0 * v[i].y + r[i].x - (1.0 - mu) * (r[i].x + mu) / (d * d * d) - mu * (r[i].x - 1.0 + mu) / (rr * rr * rr);
                double yddot = -2.0 * v[i].x + r[i].y - (1.0 - mu) * r[i].y / (d * d * d) - mu * r[i].y / (rr * rr * rr);
                double zddot = -(1.0 - mu) * r[i].z / (d * d * d) - mu * r[i].z / (rr * rr * rr);
                dv_out[i] = new double3(xddot, yddot, zddot);
            }

            // ExternalAcceleration(t, 0.25 * dt, ref geJob, ref dv_out, EAFilter.MASSLESS);
        }

    }
}

