using System.Diagnostics;
using Unity.Mathematics;


namespace GravityEngine2 {
    // based off of the "typedef struct elsetrec" in the CSSI's sgp4unit.h file
    // conatins all the data needed for a SGP4 propogated satellite
    // holds all initialization info, etc.

    //package sgp4_cssi;

    /**
    * 19 June 2009
    * converted to Java by:
    * @author Shawn E. Gano, shawn@gano.name
    * 
    */
    // Extensions made for GE 2021
    public struct SGP4SatData {
        public int satnum; // changed to int SEG
        public int epochyr, epochtynumrev;
        public int error; // 0 = ok, 1= eccentricity (sgp4),   6 = satellite decay, 7 = tle data
        public char operationmode;
        public char init, method;
        /*
                // Due to issue with assigning fields in structs need a handful of setters

                public void SetError(int error)
                {
                    this.error = error;
                }

                public void SetT(double t)
                {
                    this.t = t;
                }

                public void SetXLcof(double x)
                {
                    xlcof = x;
                }

                public void SetAYcof(double x)
                {
                    aycof = x;
                }

                public void SetCon41(double x)
                {
                    con41 = x;
                }

                public void SetX1mth2f(double x)
                {
                    x1mth2 = x;
                }

                public void SetX7thm1(double x)
                {
                    x7thm1 = x;
                }

                public void SetLiveCOE(double el, double am, double mrt, double xinc, double xnode, double su, double argp)
                {
                    _el = el;
                    _am = am;
                    _mrt = mrt;
                    _xinc = xinc;
                    _xnode = xnode;
                    _su = su;
                    _argpp = argp;
                    Debug.LogFormat("su={0} argpp={1}", _su, _argpp);
                }
          */

        // NB - keep year month day
        public int year, month, day;

        private const double SMALL = 1E-4;

        //    *  return code - non-zero on error.
        //*                   1 - mean elements, ecc >= 1.0 or ecc< -0.001 or a < 0.95 er
        //*                   2 - mean motion less than 0.0
        //*                   3 - pert elements, ecc< 0.0  or  ecc> 1.0
        //*                   4 - semi-latus rectum < 0.0
        //*                   5 - epoch elements are sub-orbital
        //*                   6 - satellite has decayed

        public static string ErrorString(int errorNum)
        {
            string s = "unknown";
            switch (errorNum) {
                case 0:
                    s = "ok";
                    break;
                case 1:
                    s = "eccentricity (sgp4)";
                    break;
                case 2:
                    s = "mean motion less than 0.0";
                    break;
                case 3:
                    s = "pert elements, ecc< 0.0  or  ecc> 1.0";
                    break;
                case 4:
                    s = "semi-latus rectum < 0.0";
                    break;
                case 5:
                    s = "epoch elements are sub-orbital";
                    break;
                case 6:
                    s = "satellite decay";
                    break;
                case 7:
                    s = "tle data";
                    break;
                default:
                    s = "code=" + errorNum.ToString();
                    break;

            }
            return s;
        }

        public string EpochDateString()
        {
            return string.Format("d/m/y {0}/{1}/{2} JDS={3}", day, month, year, jdsatepoch);
        }

        public SGP4unit.Gravconsttype gravconsttype; // gravity constants to use - SEG

        /* Near Earth */
        public int isimp;
        public double aycof, con41, cc1, cc4, cc5, d2, d3, d4,
                      delmo, eta, argpdot, omgcof, sinmao, t, t2cof, t3cof,
                      t4cof, t5cof, x1mth2, x7thm1, mdot, nodedot, xlcof, xmcof,
                      nodecf;

        /* Deep Space */
        public int irez;
        public double d2201, d2211, d3210, d3222, d4410, d4422, d5220, d5232,
                      d5421, d5433, dedt, del1, del2, del3, didt, dmdt,
                      dnodt, domdt, e3, ee2, peo, pgho, pho, pinco,
                      plo, se2, se3, sgh2, sgh3, sgh4, sh2, sh3,
                      si2, si3, sl2, sl3, sl4, gsto, xfact, xgh2,
                      xgh3, xgh4, xh2, xh3, xi2, xi3, xl2, xl3,
                      xl4, xlamo, zmol, zmos, atime, xli, xni;

        public double a, altp, alta, epochdays, jdsatepoch, nddot, ndot,
                      bstar, rcse, inclo, nodeo, ecco, argpo, mo,
                      no;

        // Extra Data added by SEG - from TLE and a name variable (and save the lines for future use)
        // public string name = "", line1 = "", line2 = "";

        public bool tleDataOk;
        // public string classification, intldesg;
        public int nexp, ibexp, numb; // numb is the second number on line 1
        public long elnum, revnum;

        // NBP - add some fields for tracking internal calculation
        public double _xinc, _xnode, _su, _el, _am, _mrt, _argpp;

        /// <summary>
        /// Initialize an SGP4 prop structure to propagate from a given COE. 
        /// Need to provide the correct scale factor to convert GE physics units
        /// into km. 
        /// 
        /// Note immediatly after evolution to time=0 there WILL be small changes in 
        /// the orbit parameters that result from the r, v at time=0. This is par for the
        /// course with the SGP4 secular variation code. 
        /// 
        /// </summary>
        /// <param name="coeKm">Classical orbit elements, distance in km</param>
        /// <param name="timeJD"></param>
        public void InitFromCOE(Orbital.COE coeKm, double timeJD)
        {
            // do the other way (derive no from a)
            //satrec.a = System.Math.Pow(satrec.no * tumin, (-2.0 / 3.0));
            double[] temp = SGP4unit.getgravconst(SGP4unit.Gravconsttype.wgs72);
            double tumin = temp[0];
            double radiusearthkm = temp[2];
            coeKm.mu = GBUnits.earthMuKm;
            // ---- convert to sgp4 units ----
            a = coeKm.a / radiusearthkm; // SGP4 used a as a fraction of Earth radius
            no = System.Math.Pow(a, -3.0 / 2.0) / tumin;

            // ---- find standard orbital elements ----
            inclo = coeKm.i;
            nodeo = coeKm.omegaU;
            argpo = coeKm.omegaL;
            double Eanom = Orbital.ConvertTrueAnomolytoE(coeKm.nu, coeKm.e);
            mo = Orbital.ConvertEtoMeanAnomoly(Eanom, coeKm.e);

            alta = a * (1.0 + ecco) - 1.0;
            altp = a * (1.0 - ecco) - 1.0;

            jdsatepoch = timeJD;
            UpdateEpochYearDay();

            //----------------initialize the orbit at sgp4epoch -------------------
            bool result = SGP4unit.sgp4init(SGP4unit.Gravconsttype.wgs72, SGP4utils_GE2.OPSMODE_IMPROVED,
                  satn: 999,
                  jdsatepoch - 2433281.5,
                  xbstar: 0.0,
                  coeKm.e, coeKm.omegaL, coeKm.i, mo, no, coeKm.omegaU,
                  ref this);
            // Evolve for 0 to get initial r, v
            //(int status, double3 r, double3 v) = SGP4unit.SGP4Prop(ref this, tsince: 0.0);
            (int status, double3 r, double3 v, Orbital.COEStruct coe) =
                                        SGP4unit.SGP4Prop(ref this, tsince: 0.0);
            // Debug.LogFormat("InitFromCOE = {0}", LogString());
            // Debug.LogFormat("Evolve for 0: err={0} r={1} v={2}", status, r, v);
        }

        public void FillInCOEKmWithInitialData(ref Orbital.COE coeKm)
        {
            double[] temp = SGP4unit.getgravconst(SGP4unit.Gravconsttype.wgs72);
            double radiusearthkm = temp[2];
            coeKm.a = a * radiusearthkm;
            coeKm.e = ecco;
            coeKm.p = coeKm.a * (1 - coeKm.e * coeKm.e);
            coeKm.i = inclo;
            coeKm.omegaU = nodeo;
            coeKm.omegaL = argpo;
            coeKm.nu = Orbital.ConvertEtoTrueAnomoly(Orbital.ConvertMeanAnomolyToE(mo, ecco), ecco);
            coeKm.ComputeRotation();
            coeKm.ComputeType();
        }

#if WHA
        public void UpdateOrbitInfo(Orbital.COE coe, double timeJD, double scaleToKm)
        {
            // do the other way (derive no from a)
            //satrec.a = System.Math.Pow(satrec.no * tumin, (-2.0 / 3.0));
            double[] temp = SGP4unit.getgravconst(SGP4unit.Gravconsttype.wgs72);
            double tumin = temp[0];
            double radiusearthkm = temp[2];

            // ---- convert to sgp4 units ----
            double orbitU_a = scaleToKm * coe.a; // SI -> km
            a = orbitU_a / radiusearthkm; // SGP4 used a as a fraction of Earth radius
            tumin = temp[0];
            no = System.Math.Pow(a, -3.0 / 2.0) / tumin;

            ecco = coe.e;
             // ---- find standard orbital elements ----
            inclo = coe.i;
            nodeo = coe.omegaU;
                argpo = coe.omegaL;
            double Eanom = Orbital.ConvertTrueAnomolytoE(coe.nu , coe.e);
            mo = Orbital.ConvertEtoMeanAnomoly(Eanom, coe.e);

            alta = a * (1.0 + ecco) - 1.0;
            altp = a * (1.0 - ecco) - 1.0;

            jdsatepoch = timeJD;
            UpdateEpochYearDay();

        }
#endif

        private void UpdateEpochYearDay()
        {
            double[] dateInfo = SGP4utils_GE2.InvJday(jdsatepoch);
            year = (int)dateInfo[0];
            month = (int)dateInfo[1];
            day = (int)dateInfo[2];
            epochyr = year % 100;
            double jdDay0 = SGP4utils_GE2.JulianDate(year, 1, 0, 0, 0, 0);
            epochdays = jdsatepoch - jdDay0;
        }

        public string LogString()
        {
            string s = "SGP4 Details:\n";
            s += string.Format("mo={0:0.000E0} no={1:0.000E0} a={2:0.000E0} jdsatepoch={3}",
                mo, no, a,
                jdsatepoch);
            return s;
        }

        const double rad2deg = 180.0 / System.Math.PI;

        /// <summary>
        /// Return the classic TLE format for that "punch card" look and feel.
        /// </summary>
        /// <returns></returns>
        public (string, string) CreateTLELines()
        {
            string line1 = "1 ";
            const double xpdotp = 1440.0 / (2.0 * System.Math.PI);  // 229.1831180523293


            line1 += string.Format("{0,5}{1,1} ", satnum, ""); // classification);
            line1 += string.Format("{0,8} ", "");// intldesg);
            line1 += string.Format("{0:00}{1:000.00000000} ", epochyr, epochdays);

            if (ndot >= 0)
                line1 += " ";
            else
                line1 += "-";
            // undo unit conversion done in SGP4utils and trim minus so length is constant
            line1 += string.Format("{0:.00000000} ", ndot * (xpdotp * 1440.0)).Replace("-", "");

            if (nddot >= 0)
                line1 += "+";
            else
                line1 += "-";
            // need in scientific mode, but without the "E"
            // exponent will always be negative or zero, must put in a dash
            string nddotString = string.Format("{0:.00000E0}", nddot * (xpdotp * 1440.0 * 1440.0));
            // drop leading minus sign if present
            if (nddotString.StartsWith("-")) {
                nddotString = nddotString.Substring(1);
            }
            line1 += string.Format("{0} ", nddotString.Replace("E-", "-").Replace("E", "-").Replace(".", ""));
            string bstarString = string.Format(" {0:.00000E0}", bstar).Replace("E", "-").Replace(".", "");
            line1 += string.Format("{0} ", bstarString.Replace("E-", "-").Replace(".", ""));
            line1 += string.Format("{0,1} {1:0000}", numb, elnum);
            // Java code ignore checksum

            string line2 = "2 ";
            line2 += string.Format("{0,5} ", satnum);
            line2 += string.Format("{0:000.0000} ", inclo * rad2deg);
            line2 += string.Format("{0:000.0000} ", nodeo * rad2deg);
            line2 += string.Format("{0:.0000000} ", ecco).Replace(".", ""); // decimal point is implied
            line2 += string.Format("{0:000.0000} ", argpo * rad2deg);
            line2 += string.Format("{0:000.0000} ", mo * rad2deg);
            line2 += string.Format("{0:00.00000000}", no * xpdotp);
            line2 += string.Format("{0:00000}", revnum);
            // Java code ignore checksum
            return (line1, line2);
        }

        // icky globals for secant - beware
        private static SGP4SatData satData;
        private static double el_target;
        public static double EpExpression(double ecco)
        {
            double temp = 1.0 / (satData._am * (1.0 - ecco * ecco));
            double axnl = ecco * System.Math.Cos(satData._argpp);
            double aynl = ecco * System.Math.Sin(satData._argpp) + temp * satData.aycof;
            double el2 = axnl * axnl + aynl * aynl;
            return el_target - System.Math.Sqrt(el2);
        }

        public static double SolveForEcc(SGP4SatData data, double ecco1, double ecco2, double el)
        {
            satData = data;
            el_target = el;
            double root = SecantRootFind_GE2.Secant(EpExpression, ecco1, ecco2);
            return root;
        }

        /// <summary>
        /// The Unity console does not have a fixed width font BUT if open log in Notepad then this can be useful
        /// </summary>
        /// <param name="tleLine"></param>
        /// <returns></returns>
        public static string AddCols(string tleLine)
        {
            return ".........1.........2.........3.........4.........5.........6.........\n" +
                   "1234567890123456789012345678901234567890123456789012345678901234567890\n" +
                   tleLine;
        }
    }


}
