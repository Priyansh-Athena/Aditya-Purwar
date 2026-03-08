using UnityEngine.Networking;
using System.Collections.Generic;
using UnityEngine;

namespace GravityEngine2 {
    public class JPLHorizonTools {
        /// <summary>
        /// Utilities used to interact with the JPL Horizons API
        /// </summary>
        public static string SOE = "$$SOE";

        public static BodyInitData HorizonResponseToBID(string response)
        {
            string[] lines = response.Split('\n', System.StringSplitOptions.RemoveEmptyEntries);
            BodyInitData bid = new BodyInitData();
            bid.initData = BodyInitData.InitDataType.COE;

            bid.units = GBUnits.Units.SI_km;

            int line = 0;
            while (!lines[line].Contains(SOE) && line < lines.Length - 1)
                line++;

            if (line >= lines.Length) {
                Debug.LogError("Did not find start tag " + SOE);
                return null;
            }
            // Data will be of the form: (all angles are in degrees!)
            //
            //2460325.500000000 = A.D. 2024 - Jan - 16 00:00:00.0000 TDB
            // EC = 9.329173880807723E-02 QR = 2.066732673871531E+08 IN = 1.847868172173823E+00
            // OM = 4.948998228742837E+01 W = 2.866565693570110E+02 Tp = 2460438.910300316289
            // N = 6.065315833230202E-06 MA = 3.005680933302082E+02 TA = 2.908320107257045E+02
            // A = 2.279379997216178E+08 AD = 2.492027320560826E+08 PR = 5.935387536254233E+07

            line += 2;  // skip JD for now
            // Brittle parsing based on field location!
            char[] delim = { ' ', '=' }; // some are EC= and some are W =
            string[] fields = lines[line].Split(delim, System.StringSplitOptions.RemoveEmptyEntries);
            bid.eccentricity = I18N.DoubleParse(fields[1]);
            bid.inclination = I18N.DoubleParse(fields[5]);
            line++;
            fields = lines[line].Split(delim, System.StringSplitOptions.RemoveEmptyEntries);
            bid.omega_uc = I18N.DoubleParse(fields[1]);
            bid.omega_lc = I18N.DoubleParse(fields[3]);
            line++;
            fields = lines[line].Split(delim, System.StringSplitOptions.RemoveEmptyEntries);
            bid.nu = I18N.DoubleParse(fields[5]);
            line++;
            fields = lines[line].Split(delim, System.StringSplitOptions.RemoveEmptyEntries);
            bid.a = I18N.DoubleParse(fields[1]);
            return bid;
        }



        private static string TB_NAME = "Target body name:";

        public static string BodyName(string jplData)
        {
            string[] lines = jplData.Split('\n', System.StringSplitOptions.RemoveEmptyEntries);
            foreach (string line in lines) {
                if (line.Contains(TB_NAME)) {
                    // have: Target body name: Mars (499)                      {source: mar097}
                    char[] delim = { ' ' }; // some are EC= and some are W =
                    string[] fields = line.Split(delim, System.StringSplitOptions.RemoveEmptyEntries);
                    return fields[3];
                }
            }
            return "no name";
        }

        // Mass is not reported consistently in all response (and not at all for somesatellites)
        // So look for the first GM occurance (and assume GM 1-sigma will not be first)
        public static double Mass(string jplData)
        {
            string[] lines = jplData.Split('\n', System.StringSplitOptions.RemoveEmptyEntries);
            foreach (string line in lines) {
                if (line.Contains("Keplerian"))
                    continue;
                if (line.Contains("GM")) {

                    // have something like
                    //   GM (km^3/s^2)         = 42828.375214    Mass ratio (Sun/Mars) = 3098703.59
                    // Earth
                    //   GM, km ^ 3 / s ^ 2 = 398600.435436   Inner core rad = 1215 km
                    // Callisto:
                    //   GM(km ^ 3 / s ^ 2) = 7179.2834 + -0.0097 Geometric Albedo = 0.17 + -0.02 
                    // find index of GM, then next =, then split & parse
                    int gmIndex = line.IndexOf("GM");
                    int eqIndex = line.IndexOf("=", gmIndex);
                    if (eqIndex == -1) {
                        eqIndex = line.IndexOf(":", gmIndex);
                    }
                    // may have a +- tacked on, so convert to a space
                    string fromEquals = line.Substring(eqIndex + 1);
                    fromEquals = fromEquals.Replace("+-", " ");
                    // sometimes get km^3/s^2 at end of line
                    fromEquals = fromEquals.Replace("km^3/s^2", "");
                    string[] fields = fromEquals.Split(' ', System.StringSplitOptions.RemoveEmptyEntries);
                    double mass = 0;
                    // Mass value may be empty (so pick up text from next field e.g. Jupiter sat Elara
                    try {
                        double gm = I18N.DoubleParse(fields[0]);
                        if (double.IsNaN(gm)) {
                            Debug.Log("skipped parse of mass " + fromEquals + " was " + fields[0]);
                        } else {
                            mass = gm / GBUnits.NEWTONS_G_KM;
                        }

                    } catch (System.Exception e) {
                        // just leave mass as zero
                        Debug.Log("skipped parse of mass " + e.ToString() + " fromEquals=" + fromEquals + " was " + fields[0]);
                    }
                    // want result in kg
                    return mass;
                }
            }
            return 0;
        }

        public static double Radius(string jplData)
        {
            string[] lines = jplData.ToLower().Split('\n', System.StringSplitOptions.RemoveEmptyEntries);
            string target = "radius";
            foreach (string line in lines) {
                if (line.Contains(target)) {

                    // have something like
                    //   Vol. mean radius (km) = 3389.92+-0.04   Density (g/cm^3)      =  3.933(5+-4)
                    //  Equ. radius, km          = 6378.137        Mass layers:
                    // Vol.Mean Radius(km) = 69911 + -6          Flattening = 0.06487
                    // Mean radius(km)      = 2410.3 + -1.5       Density(g cm ^ -3) = 1.834 + -0.004
                    //   Radius (km)           = 581.1x577.9x577.7 Density (g cm^-3)   = 1.67 +- 0.15

                    int rIndex = line.IndexOf(target);
                    int eqIndex = line.IndexOf("=", rIndex);
                    // may have a +- tacked on, so convert to a space
                    string fromEquals = line.Substring(eqIndex + 1);
                    fromEquals = fromEquals.Replace("+-", " ");
                    string[] fields = fromEquals.Split(' ', System.StringSplitOptions.RemoveEmptyEntries);
                    // was the response tri-axial e.g. ariel: 581.1x577.9x577.7
                    double radius = 0;
                    try {
                        if (fields[0].Contains("x")) {
                            radius = I18N.DoubleParse(fields[0].Substring(0, fields[0].IndexOf("x")));
                        } else {
                            radius = I18N.DoubleParse(fields[0]);
                        }
                    } catch (System.Exception e) {
                        Debug.Log("skipped parse of radius " + e.ToString());
                    }
                    return radius;
                }
            }
            return 0;
        }

        public static double RotationRate(string jplData)
        {
            string[] lines = jplData.ToLower().Split('\n', System.StringSplitOptions.RemoveEmptyEntries);
            string target = "rot. rate";
            foreach (string line in lines) {
                if (line.Contains(target)) {

                    // have something like
                    //    Rot. Rate (rad/s)        = 0.00007292115   Surface area:
                    //   Sidereal rot. period  =   24.622962 hr  Sid. rot. rate, rad/s =  0.0000708822 
                    //   Sid. rot. period (III)= 9h 55m 29.71 s    Sid. rot. rate (rad/s)= 0.00017585
                    int rIndex = line.IndexOf(target);
                    int eqIndex = line.IndexOf("=", rIndex);
                    // may have a +- tacked on, so convert to a space
                    string fromEquals = line.Substring(eqIndex + 1);
                    fromEquals = fromEquals.Replace("+-", " ");
                    string[] fields = fromEquals.Split(' ', System.StringSplitOptions.RemoveEmptyEntries);
                    // was the response tri-axial e.g. ariel: 581.1x577.9x577.7
                    double rotRate = 0;
                    try {
                        rotRate = I18N.DoubleParse(fields[0]);
                    } catch (System.Exception e) {
                        Debug.Log("skipped parse of rot. rate " + e.ToString());
                    }
                    return rotRate;
                }
            }
            return 0;
        }

        public static double AxisTiltDegrees(string jplData)
        {
            string[] lines = jplData.ToLower().Split('\n', System.StringSplitOptions.RemoveEmptyEntries);
            string target = "obliquity to orbit";
            foreach (string line in lines) {
                if (line.Contains(target)) {

                    // have something like
                    //    Rot. Rate (rad/s)        = 0.00007292115   Surface area:
                    //   Sidereal rot. period  =   24.622962 hr  Sid. rot. rate, rad/s =  0.0000708822 
                    //   Sid. rot. period (III)= 9h 55m 29.71 s    Sid. rot. rate (rad/s)= 0.00017585
                    int rIndex = line.IndexOf(target);
                    int eqIndex = line.IndexOf("=", rIndex);
                    // may have a +- tacked on, so convert to a space
                    string fromEquals = line.Substring(eqIndex + 1);
                    fromEquals = fromEquals.Replace("+-", " ");
                    string[] fields = fromEquals.Split(' ', System.StringSplitOptions.RemoveEmptyEntries);
                    // was the response tri-axial e.g. ariel: 581.1x577.9x577.7
                    double tilt = 0;
                    try {
                        tilt = I18N.DoubleParse(fields[0]);
                    } catch (System.Exception e) {
                        Debug.Log("skipped parse of rot. rate " + e.ToString());
                    }
                    return tilt;
                }
            }
            return 0;
        }

        // Process a small body search result and return the possible names for use in a JPL query
        public static List<string> SmallBodyNames(string jplData, string matchName)
        {
            List<string> choices = new List<string>();
            string[] lines = jplData.Split('\n');
            int line = 0;
            string lastChoice = "";
            while (line < lines.Length) {
                if (lines[line].Contains("Primary")) {
                    int pIndex = lines[line].IndexOf("Primary");
                    // skip over -----
                    line += 2;
                    while (lines[line].Length > 1) {
                        string choice = lines[line].Substring(pIndex).Replace(matchName, "").Trim();
                        if (!choice.Equals(lastChoice)) {
                            choices.Add(choice);
                            lastChoice = choice;
                        }
                        line++;
                    }
                    break;
                }
                line++;
            }
            return choices;
        }
    }

}
