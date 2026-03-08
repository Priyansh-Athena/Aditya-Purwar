using Unity.Mathematics;
using Unity.Collections;
using System;

namespace GravityEngine2 {
    /// <summary>
    /// Integrates the effect of Earth atmospheric drag on a spacecraft. 
    /// 
    /// This is a self-integrating external acceleration that can be used to model the reentry of a spacecraft
    /// into the Earth's atmosphere. It can also be used to model the path of a discarded stage of a multi-stage
    /// booster. 
    /// 
    /// This block can be added in a "dormant" mode (e.g. a first stage prior to staging) and then activated
    /// when the stage is ready to reenter.  In this mode it has a reference to the booster object and will
    /// take it's R and V from the booster object. 
    /// 
    /// The data[] block is initialized with the following parameters:
    /// 
    /// </summary>
    public class EarthAtmosphereReentry {

        private static double[] densityTablePer10km;

        // Data block:
        // [0] = (h0, massKg, Cd)
        // [1] = (area, mu_SI, dtSec)
        // [2] = (r)
        // [3] = (v)
        // [4] = (tSec, 0, 0)

        private const int H0_OFFSET = 0;
        private const int H0_F = 0;
        private const int MASS_OFFSET = 0;
        private const int MASS_F = 1;
        private const int CD_OFFSET = 0;
        private const int CD_F = 2;
        private const int AREA_OFFSET = 1;
        private const int AREA_F = 0;
        private const int MU_OFFSET = 1;
        private const int MU_F = 1;
        private const int R_OFFSET = 2;
        private const int V_OFFSET = 3;
        private const int DT_OFFSET = 1;
        private const int DT_F = 2;
        private const int T_OFFSET = 4;
        private const int T_F = 0;

        public const int DATA_SIZE = 5;

        /// <summary>
        /// Create a data[] block for core physics parameters/data and initialize. 
        /// </summary>
        /// <param name="h0Km">radius of the body being reentered (used for height to deterine the atmo density)</param>
        /// <param name="Cd">coefficient of drag</param>
        /// <param name="area">cross-sectional area (for drag calculation)</param>
        /// <param name="massKg">spacecraft mass</param>
        /// <param name="mu_SI">GM for the body being reentered</param>
        /// <param name="dtSec">time step of the self-integrating semi-implicit Euler method</param>
        /// <returns></returns>
        public static double3[] Alloc(double h0Km, double Cd, double area, double massKg, double mu_SI, double dtSec, bool shadowMode = false)
        {
            // lazy init of static density table
            LoadDensityProfile();
            double3[] data = new double3[DATA_SIZE];
            data[0] = new double3(h0Km, massKg, Cd);
            data[1] = new double3(area, mu_SI, dtSec);
            // use a negative t value to indicate shadow mode
            if (shadowMode) {
                data[4] = new double3(-1, 0, 0);
            }
            return data;
        }

        public const int STATUS_OK = 0;
        public const int STATUS_IMPACT = 1;
        public const int STATUS_NAN = 2;

        public static (int status, double3 r, double3 v) EvolveSelfIntegrated(ref ExternalAccel.EAStateData eaState,
                                int extAccelId,
                                double tGE,
                                double dt,
                                ref GEPhysicsCore.GEPhysicsJob gePhysJob)
        {
            NativeArray<ExternalAccel.EADesc> eaDescs = gePhysJob.extADesc;
            NativeArray<double3> data = gePhysJob.extAData;
            double scaleTtoSec = gePhysJob.ScaleTtoSec();
            double scaleLtoSI = gePhysJob.ScaleLtoSI();
            int b = eaDescs[extAccelId].paramBase;
            double3 r = data[b + R_OFFSET];
            double3 v = data[b + V_OFFSET];
            // check if in shadow mode (indicated by negative t)
            double tFromSec = data[b + T_OFFSET][T_F];
            if (tFromSec < 0) {
                r /= scaleLtoSI;
                v *= scaleTtoSec / scaleLtoSI;
                return (STATUS_OK, r, v);
            }

            double massKg = data[b + MASS_OFFSET][MASS_F];

            // dumb test to see if r has been set yet. If not, use the initial state
            if (math.lengthsq(r) < 1E-9) {
                r = eaState.r_from * scaleLtoSI;
                v = eaState.v_from * scaleTtoSec / scaleLtoSI;
            }
            double mu_SI = data[b + MU_OFFSET][MU_F];
            double Cd = data[b + CD_OFFSET][CD_F];
            double area = data[b + AREA_OFFSET][AREA_F];
            double dtSec = data[b + DT_OFFSET][DT_F];

            // determine height above Earth's surface in km
            // due to dynamic add/delete cannot in general keep a cached value of the index for ship or Earth
            // TODO: This assumes Earth is at origin!!
            double h0_SI = data[b + H0_OFFSET][H0_F] * GBUnits.kmToSI;

            double t = tFromSec;
            double t_until = (tGE + dt) * scaleTtoSec;
            while (t < t_until) {
                t += dtSec;

                double h = math.length(r) - h0_SI;
                if (double.IsNaN(h)) {
                    return (STATUS_NAN, r, v);
                }
                h *= GBUnits.SItoKm;

                if (h < 0) {
                    // Generate a collision event
                    GEPhysicsCore.PhysEvent collisionEvent = new GEPhysicsCore.PhysEvent {
                        bodyId = eaDescs[extAccelId].bodyId,
                        type = GEPhysicsCore.EventType.EA_REENTRY,
                        statusCode = STATUS_IMPACT,
                        r = r,
                        v = v,
                        t = t
                    };
                    gePhysJob.gepcEvents.Add(collisionEvent);
                    // IMPACT status will trigger removal of the external acceleration from physics list
                    return (STATUS_IMPACT, r, v);
                }
                // retrieve the density from the pre-calculated table
                double hdiv10 = h / 10.0;
                int densityIndex = (int)hdiv10;
                double density = 0.0;
                if (densityIndex < 0)
                    densityIndex = 0;
                if (densityIndex < densityTablePer10km.Length - 2) {
                    // interpolate.
                    density = densityTablePer10km[densityIndex] +
                                   (hdiv10 - densityIndex) *
                                   (densityTablePer10km[densityIndex + 1] - densityTablePer10km[densityIndex]);
                }
                // determine accel due to drag
                double v_rel_sq = math.lengthsq(v);
                // guard against divide by zero for norm
                if (v_rel_sq < 1E-9) {
                    return (STATUS_OK, r, v);
                }

                double a_mag = 0.5 * Cd * area * density * v_rel_sq / massKg;
                double3 a_drag = -a_mag * math.normalize(v);

                double r_mag = math.length(r);
                double3 a_gravity = -mu_SI / (r_mag * r_mag) * math.normalize(r);
                // v then r => semi-implicit Euler method
                v += (a_gravity + a_drag) * dtSec;
                r += v * dtSec;
                // UnityEngine.Debug.LogFormat("EAR: h_km={0} a_drag={1} a_gravity={2} |r|={3} |v|={4} density={5}", h,
                //         math.length(a_drag), math.length(a_gravity), math.length(r), math.length(v), density);
            }
            data[b + R_OFFSET] = r;
            data[b + V_OFFSET] = v;
            // convert back to GE scale
            r /= scaleLtoSI;
            v *= scaleTtoSec / scaleLtoSI;
            // record time in seconds
            double3 tmp = data[b + T_OFFSET];
            tmp[T_F] = t;
            data[b + T_OFFSET] = tmp;

            if (GravityMath.HasNaN(r) || GravityMath.HasNaN(v)) {
                return (STATUS_NAN, r, v);
            }

            return (STATUS_OK, r, v);
        }

        public static void UpdateShadow(double3 r, double3 v, double tSec, int paramBase, NativeArray<double3> data)
        {
            data[paramBase + R_OFFSET] = r;
            data[paramBase + V_OFFSET] = v;
            data[paramBase + T_OFFSET] = new double3(tSec, 0, 0);
        }

        /// <summary>
        /// Compute the density of the atmosphere at a given height in km.
        /// </summary>
        /// <param name="heightKm"></param>
        /// <returns>density in kg/m^3</returns>
        public static double Density(double heightKm)
        {
            if (densityTablePer10km == null) {
                LoadDensityProfile();
            }
            double hdiv10 = heightKm / 10.0;
            int densityIndex = (int)hdiv10;
            if (densityIndex < 0)
                densityIndex = 0;
            if (densityIndex > densityTable.Length - 2) {
                // too far away, no drag
                return 0.0;
            }
            // interpolate.
            // interpolate.
            double density = densityTablePer10km[densityIndex] +
                            (hdiv10 - (double)densityIndex) *
                            (densityTablePer10km[densityIndex + 1] - densityTablePer10km[densityIndex]);
            return density;
        }

        /// <summary>
        /// Determining the density for each call to acceleration is quite expensive. Instead create a 
        /// table indexed by integer km number/10 and then interpolate the density from there. Instead of doing this
        /// at run time, take from a table of values computed from the DensityTable method below. 
        /// 
        /// (Original code to build table is in Gravity Engine version)
        /// </summary>
        private static void LoadDensityProfile()
        {
            if (densityTablePer10km != null) {
                return;
            }
            densityTablePer10km = new double[101];

            densityTablePer10km[0] = 1.225;
            densityTablePer10km[1] = 0.308337666482878;
            densityTablePer10km[2] = 0.0776098910792704;
            densityTablePer10km[3] = 0.01774;
            densityTablePer10km[4] = 0.003972;
            densityTablePer10km[5] = 0.001057;
            densityTablePer10km[6] = 0.0003206;
            densityTablePer10km[7] = 8.77E-05;
            densityTablePer10km[8] = 1.905E-05;
            densityTablePer10km[9] = 3.396E-06;
            densityTablePer10km[10] = 5.297E-07;
            densityTablePer10km[11] = 9.661E-08;
            densityTablePer10km[12] = 2.438E-08;
            densityTablePer10km[13] = 8.484E-09;
            densityTablePer10km[14] = 3.845E-09;
            densityTablePer10km[15] = 2.07E-09;
            densityTablePer10km[16] = 1.32784591953683E-09;
            densityTablePer10km[17] = 8.51775258951991E-10;
            densityTablePer10km[18] = 5.464E-10;
            densityTablePer10km[19] = 3.90373444167217E-10;
            densityTablePer10km[20] = 2.789E-10;
            densityTablePer10km[21] = 2.13011858346484E-10;
            densityTablePer10km[22] = 1.62689321607108E-10;
            densityTablePer10km[23] = 1.24255126312868E-10;
            densityTablePer10km[24] = 9.49007363391221E-11;
            densityTablePer10km[25] = 7.248E-11;
            densityTablePer10km[26] = 5.81922633011631E-11;
            densityTablePer10km[27] = 4.67210197035306E-11;
            densityTablePer10km[28] = 3.7511063469739E-11;
            densityTablePer10km[29] = 3.01166346873302E-11;
            densityTablePer10km[30] = 2.418E-11;
            densityTablePer10km[31] = 2.00665869406376E-11;
            densityTablePer10km[32] = 1.66529326487248E-11;
            densityTablePer10km[33] = 1.3819996725071E-11;
            densityTablePer10km[34] = 1.14689894873021E-11;
            densityTablePer10km[35] = 9.518E-12;
            densityTablePer10km[36] = 7.88971838521758E-12;
            densityTablePer10km[37] = 6.53999329670523E-12;
            densityTablePer10km[38] = 5.4211709762781E-12;
            densityTablePer10km[39] = 4.4937499811882E-12;
            densityTablePer10km[40] = 3.725E-12;
            densityTablePer10km[41] = 3.13983578401641E-12;
            densityTablePer10km[42] = 2.64659563774227E-12;
            densityTablePer10km[43] = 2.23083911119595E-12;
            densityTablePer10km[44] = 1.8803942200581E-12;
            densityTablePer10km[45] = 1.585E-12;
            densityTablePer10km[46] = 1.34472083341642E-12;
            densityTablePer10km[47] = 1.14086695257044E-12;
            densityTablePer10km[48] = 9.67916441184711E-13;
            densityTablePer10km[49] = 8.21184481682875E-13;
            densityTablePer10km[50] = 6.967E-13;
            densityTablePer10km[51] = 5.95659455100653E-13;
            densityTablePer10km[52] = 5.09272551242725E-13;
            densityTablePer10km[53] = 4.35414109905211E-13;
            densityTablePer10km[54] = 3.72267161546252E-13;
            densityTablePer10km[55] = 3.18278246875997E-13;
            densityTablePer10km[56] = 2.72119200666783E-13;
            densityTablePer10km[57] = 2.32654477955506E-13;
            densityTablePer10km[58] = 1.98913218839821E-13;
            densityTablePer10km[59] = 1.70065364642521E-13;
            densityTablePer10km[60] = 1.454E-13;
            densityTablePer10km[61] = 1.26504851358249E-13;
            densityTablePer10km[62] = 1.10065181686194E-13;
            densityTablePer10km[63] = 9.57618944218064E-14;
            densityTablePer10km[64] = 8.33173605200477E-14;
            densityTablePer10km[65] = 7.24900296296442E-14;
            densityTablePer10km[66] = 6.30697415629518E-14;
            densityTablePer10km[67] = 5.48736470538128E-14;
            densityTablePer10km[68] = 4.7742658624674E-14;
            densityTablePer10km[69] = 4.15383626737414E-14;
            densityTablePer10km[70] = 3.614E-14;
            densityTablePer10km[71] = 3.22855174742338E-14;
            densityTablePer10km[72] = 2.88421316706989E-14;
            densityTablePer10km[73] = 2.5765997400346E-14;
            densityTablePer10km[74] = 2.30179457473695E-14;
            densityTablePer10km[75] = 2.05629853250599E-14;
            densityTablePer10km[76] = 1.836985672481E-14;
            densityTablePer10km[77] = 1.64106344850035E-14;
            densityTablePer10km[78] = 1.46603715115895E-14;
            densityTablePer10km[79] = 1.30967814226946E-14;
            densityTablePer10km[80] = 1.17E-14;
            densityTablePer10km[81] = 1.07979659273866E-14;
            densityTablePer10km[82] = 9.96547591188049E-15;
            densityTablePer10km[83] = 9.19716832022883E-15;
            densityTablePer10km[84] = 8.48809488463849E-15;
            densityTablePer10km[85] = 7.83368883356844E-15;
            densityTablePer10km[86] = 7.22973547954024E-15;
            densityTablePer10km[87] = 6.6723450745379E-15;
            densityTablePer10km[88] = 6.15792775817317E-15;
            densityTablePer10km[89] = 5.68317043727025E-15;
            densityTablePer10km[90] = 5.245E-15;
            densityTablePer10km[91] = 4.96315625900571E-15;
            densityTablePer10km[92] = 4.69645758842852E-15;
            densityTablePer10km[93] = 4.44409015732391E-15;
            densityTablePer10km[94] = 4.20528386652199E-15;
            densityTablePer10km[95] = 3.97930999867005E-15;
            densityTablePer10km[96] = 3.76547899455162E-15;
            densityTablePer10km[97] = 3.56313834889675E-15;
            densityTablePer10km[98] = 3.37167061926219E-15;
            densityTablePer10km[99] = 3.19049154190597E-15;
            densityTablePer10km[100] = 3.019E-15;

        }

        internal static (bool remove, bool stopSelfIntegrating) SelfIntegratingStatusActions(int status)
        {
            switch (status) {
                case STATUS_IMPACT:
                    return (true, true);
                default:
                    return (false, false);
            }
        }

        private struct DensityEntry {
            double baseAltitudeKm;
            double density;
            double scaleHeightKm;
            public DensityEntry(double baseAltitudeKm, double density, double scaleHeightKm)
            {
                this.baseAltitudeKm = baseAltitudeKm;
                this.density = density;
                this.scaleHeightKm = scaleHeightKm;
            }
        }

        // Vallado 5th p572 raw values (above by 10km is precomputed from this!)
        private static DensityEntry[] densityTable = {
            new DensityEntry(0, 1.225, 7.249),
            new DensityEntry(25, 3.899E-2, 6.349),
            new DensityEntry(30, 1.774E-2, 6.682),
            new DensityEntry(40, 3.972E-3, 7.554),
            new DensityEntry(50, 1.057E-3, 8.382),
            new DensityEntry(60, 3.206E-4, 7.714),
            new DensityEntry(70, 8.770E-5, 6.549),
            new DensityEntry(80, 1.905E-5, 5.799),
            new DensityEntry(90, 3.396E-6, 5.382),
            new DensityEntry(100, 5.297E-7, 5.877),
            new DensityEntry(110, 9.661E-8, 7.263),
            new DensityEntry(120, 2.438E-8, 9.473),
            new DensityEntry(130, 8.484E-9, 12.636),
            new DensityEntry(140, 3.845E-9, 16.149),
            new DensityEntry(150, 2.070E-9, 22.523),
            new DensityEntry(180, 5.464E-10, 29.740),
            new DensityEntry(200, 2.789E-10, 37.943),
            new DensityEntry(250, 7.248E-11, 45.546),
            new DensityEntry(300, 2.418E-11, 53.628),
            new DensityEntry(350, 9.518E-12, 53.298),
            new DensityEntry(400, 3.725E-13, 58.515),
            new DensityEntry(450, 1.585E-12, 60.828),
            new DensityEntry(500, 6.967E-13, 63.822),
            new DensityEntry(600, 1.454E-13, 71.835),
            new DensityEntry(700, 3.614E-14, 88.667),
            new DensityEntry(800, 1.170E-14, 124.64),
            new DensityEntry(900, 5.245E-15, 181.05),
            new DensityEntry(1000, 3.019E-15, 268.0)
        };
    }
}
