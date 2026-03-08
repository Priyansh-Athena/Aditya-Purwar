using Unity.Mathematics;
using UnityEditor;
using UnityEngine;

namespace GravityEngine2 {
    /// <summary>
    /// This is a static class to provide fields to edit a BodyInitData class.
    ///
    /// Currently this class is used in GSBody and GSDisplayOrbit.
    /// 
    /// </summary>
    public class BodyInitDataEditor : Editor {


        public enum COEOrbitDataType { COE, COE_ApoPeri };
        /// <summary>
        /// Editor block for initial data. Very modal
        /// </summary>
        /// <param name="bid"></param>
        /// <param name="name"></param>
        /// <param name="gsBody">optional so can get ref to cr3bp when needed</param>
        public static void Edit(BodyInitData bid, string name, GSBody gsBody = null, bool coeInput = false)
        {
            BodyInitData.InitDataType dataType;
            if (bid.units == GBUnits.Units.CR3BP_SCALE) {
                dataType = BodyInitData.InitDataType.RV_ABSOLUTE;
                EditorGUILayout.LabelField("Input Format: RV_ABSOLUTE *required* by CR3BP units");
            } else {
                if (!coeInput) {
                    dataType = (BodyInitData.InitDataType)EditorGUILayout.EnumPopup("Input Format", bid.initData);
                } else {
                    // Allow only COE and COE_APOPeri. Typical use is for orbit of a GSBinary
                    COEOrbitDataType coeType = (COEOrbitDataType)(bid.initData - BodyInitData.InitDataType.COE);
                    coeType = (COEOrbitDataType)EditorGUILayout.EnumPopup("Input Format", coeType);
                    dataType = (BodyInitData.InitDataType)((int)BodyInitData.InitDataType.COE + coeType);
                }
            }
            double rx = bid.r.x;
            double ry = bid.r.y;
            double rz = bid.r.z;
            double vx = bid.v.x;
            double vy = bid.v.y;
            double vz = bid.v.z;

            bool orientationPrompt = true;
            double a = bid.a;
            double ecc = bid.eccentricity;
            double p_semi = bid.p_semi;
            double apo = bid.apoapsis;
            double peri = bid.periapsis;
            double incl = bid.inclination;
            double o_uc = bid.omega_uc;
            double o_lc = bid.omega_lc;
            double nu = bid.nu;
            string tleData = bid.tleData;


            GBUnits.Units units_old = bid.units;
            GBUnits.Units units_new;
            if (dataType == BodyInitData.InitDataType.TWO_LINE_ELEMENT) {
                units_new = GBUnits.Units.SI_km;
                EditorGUILayout.LabelField("Units set to SI_KM for TWO_LINE_ELEMENT");
            } else {
                units_new = (GBUnits.Units)EditorGUILayout.EnumPopup("Units", units_old);
            }

            if (units_new != units_old) {
                // do conversion of r and v, plus ask any attached gsbody to do the same
                // might be hard to find if gsbody is not attached here. Can that be the case?
                double dConvert = 1.0;
                double vConvert = 1.0;
                if ((units_new == GBUnits.Units.DL) || (units_old == GBUnits.Units.DL)) {
                    // no known conversion, so leave as is
                    // Debug.Log("Not converting to/from DL units, leave values the same " + name);
                } else if ((units_new == GBUnits.Units.CR3BP_SCALE) || (units_old == GBUnits.Units.CR3BP_SCALE)) {
                    if (gsBody != null) {
                        // need extra info about the system to do the conversion
                        GSController gsc = GSCommon.GSControllerForBody(gsBody);
                        dConvert = GBUnits.DistanceConversion(units_old, units_new, gsc.EditorCR3BPSysData());
                        vConvert = GBUnits.VelocityConversion(units_old, units_new, gsc.EditorCR3BPSysData());
                    }
                } else {
                    dConvert = GBUnits.DistanceConversion(units_old, units_new);
                    vConvert = GBUnits.VelocityConversion(units_old, units_new);
                }

                if (bid.IsRVType()) {
                    rx *= dConvert;
                    ry *= dConvert;
                    rz *= dConvert;
                    vx *= vConvert;
                    vy *= vConvert;
                    vz *= vConvert;
                } else if (bid.IsCOEType()) {
                    bid.a *= dConvert;
                    bid.apoapsis *= dConvert;
                    bid.periapsis *= dConvert;
                    bid.p_semi *= dConvert;
                } else if (bid.initData == BodyInitData.InitDataType.LATLONG_POS) {
                    bid.altitude *= dConvert;
                }

            }

            string rvType = "Absolute";
            switch (dataType) {
                case BodyInitData.InitDataType.COE:
                    EditorGUILayout.LabelField("Semi-major axis");
                    // for hyperbola expect users to use COE_P
                    a = EditorGUILayout.DelayedDoubleField("|a|", bid.a);
                    EditorGUILayout.LabelField("Eccentricity (< 1 ellipse, 0 circle)");
                    ecc = EditorGUILayout.DelayedDoubleField("eccentricity", bid.eccentricity);
                    if (ecc < 1.0) {
                        // in case user switches modes
                        apo = a * (1 + ecc);
                        peri = a * (1 - ecc);
                    }
                    break;

                case BodyInitData.InitDataType.COE_HYPERBOLA:
                    EditorGUILayout.LabelField("Periapsis");
                    peri = EditorGUILayout.DelayedDoubleField("Periapsis (closest)", bid.periapsis);
                    EditorGUILayout.LabelField("Eccentricity (must be > 1.0)");
                    ecc = EditorGUILayout.DelayedDoubleField("eccentricity", bid.eccentricity);
                    if (ecc < 1.0) {
                        EditorGUILayout.LabelField("!!!Require e > 1.0 for hyperbola");
                    }
                    a = peri / (1.0 - ecc);   // will be negative since e > 1
                    p_semi = a * (1 - ecc * ecc); // +ve, since a < 0, e > 1
                    EditorGUILayout.LabelField(string.Format("Computed a={0} psemi={1}", a, p_semi));
                    break;

                case BodyInitData.InitDataType.COE_ApoPeri:
                    EditorGUILayout.LabelField("Apo/Peri for an elliptical orbit");
                    // for hyperbola really should specify a as negative. 
                    apo = EditorGUILayout.DelayedDoubleField("Apoapsis (farthest)", bid.apoapsis);
                    EditorGUILayout.LabelField("Periapsis");
                    peri = EditorGUILayout.DelayedDoubleField("Periapsis (closest)", bid.periapsis);
                    // determine a, e and display this as text info
                    if (peri > apo)
                        peri = apo;
                    a = 0.5 * (apo + peri);
                    ecc = apo / a - 1;
                    EditorGUILayout.LabelField(string.Format("Computed a={0} ecc={1}", a, ecc));
                    break;

                case BodyInitData.InitDataType.TWO_LINE_ELEMENT:
                    EditorGUILayout.LabelField("Two-Line Element Set (TLE)");
                    orientationPrompt = false;
                    string tleDataOld = bid.tleData;
                    tleData = EditorGUILayout.TextArea(bid.tleData);
                    // need to parse the TLE, so on startup in editor we have the values (greedy)
                    SGP4utils_GE2.TLEtoSatData(tleData, ref bid.satData);
                    if (bid.satData.error != 0) {
                        Debug.LogError("Could not init TLE data err=" + SGP4SatData.ErrorString(bid.satData.error));
                        return;
                    }
                    bid.haveSatData = true;
                    if (bid.haveSatData) {
                        // Pull internal details of TLE AT it's start epoch (and not GE time)
                        double R2D = GravityMath.RAD2DEG;
                        EditorGUILayout.LabelField("Orbit Params at TLE start epoch (SI km)");
                        EditorGUILayout.LabelField("   " + bid.satData.EpochDateString());
                        EditorGUILayout.LabelField(string.Format("   a={0:G7} km",
                                                            bid.satData.a * GBUnits.earthRadiusKm));
                        EditorGUILayout.LabelField(string.Format("   e={0:G6}", bid.satData.ecco));
                        EditorGUILayout.LabelField(string.Format("   i={0:G7} deg", bid.satData.inclo * R2D));
                        EditorGUILayout.LabelField(string.Format("   argp={0:G7} deg", bid.satData.argpo * R2D));
                        EditorGUILayout.LabelField(string.Format("   raan={0:G7} deg", bid.satData.nodeo * R2D));
                        double nu0 = Orbital.ConvertMeanToTrueAnomoly(bid.satData.mo, bid.satData.ecco);
                        EditorGUILayout.LabelField(string.Format("   nu={0:G7} deg", nu0 * R2D));
                        if (bid.useTleEpoch) {
                            bid.startEpochType = BodyInitData.StartEpochType.DMY_UTC;
                            bid.startEpochYear = bid.satData.year;
                            bid.startEpochMonth = bid.satData.month;
                            bid.startEpochDay = bid.satData.day;
                            bid.startEpochUtc = math.fmod(bid.satData.jdsatepoch, 1.0) * 24.0;
                        }
                    }
                    break;

                case BodyInitData.InitDataType.RV_RELATIVE:
                    rvType = "Relative";
                    goto case BodyInitData.InitDataType.RV_ABSOLUTE;
                // Wow, I got to use a GOTO!!! (no fallthru in C#). Send hate mail to /dev/null

                case BodyInitData.InitDataType.RV_ABSOLUTE:
                    string dunits = GBUnits.DistanceShortForm(units_new);
                    EditorGUILayout.LabelField(rvType + " Position");
                    rx = EditorGUILayout.DelayedDoubleField("pos[x] " + dunits, rx);
                    ry = EditorGUILayout.DelayedDoubleField("pos[y] " + dunits, ry);
                    rz = EditorGUILayout.DelayedDoubleField("pos[z] " + dunits, rz);

                    string vunits = GBUnits.VelocityShortForm(units_new);
                    EditorGUILayout.LabelField(rvType + " Velocity");
                    vx = EditorGUILayout.DelayedDoubleField("vel[x] " + vunits, vx);
                    vy = EditorGUILayout.DelayedDoubleField("vel[y] " + vunits, vy);
                    vz = EditorGUILayout.DelayedDoubleField("vel[z] " + vunits, vz);
                    orientationPrompt = false;
                    break;

                case BodyInitData.InitDataType.LATLONG_POS:
                    EditorGUILayout.LabelField(rvType + " Position");
                    bid.latLongType = (BodyInitData.LatLongType)EditorGUILayout.EnumPopup("Position Type", bid.latLongType);
                    if (bid.latLongType == BodyInitData.LatLongType.EARTH_SURFACE) {
                        bid.site = (EarthLaunchSite.Site)EditorGUILayout.EnumPopup("Site", bid.site);
                        (bid.latitude, bid.longitude) = EarthLaunchSite.SiteLatLon(bid.site);
                        EditorGUILayout.LabelField(string.Format("Latitude: {0:G7} Longitude: {1:G7}",
                                                                bid.latitude, bid.longitude));
                    } else {
                        bid.latitude = EditorGUILayout.DelayedDoubleField("latitude (deg.)", bid.latitude);
                        bid.longitude = EditorGUILayout.DelayedDoubleField("longitude (deg.) ", bid.longitude);
                    }
                    EditorGUILayout.LabelField("Earth rotation offset based on GSC start time");
                    bid.earthRotation = EditorGUILayout.Toggle("Earth Rotation Offset", bid.earthRotation);
                    string aunits = GBUnits.DistanceShortForm(units_new);
                    bid.altitude = EditorGUILayout.DelayedDoubleField("altitude " + aunits, bid.altitude);
                    orientationPrompt = false;
                    break;

            }

            if (orientationPrompt) {
                EditorGUILayout.LabelField("Inclination (0..180)");
                incl = EditorGUILayout.DelayedDoubleField("inclination", bid.inclination);
                EditorGUILayout.LabelField("\x03c9/arg. of perigee (deg.)");
                o_lc = EditorGUILayout.DelayedDoubleField("omega_lc", bid.omega_lc);
                EditorGUILayout.LabelField("\x03a9/Right Asc. of Ascending Node (deg.)");
                o_uc = EditorGUILayout.DelayedDoubleField("omega_uc", bid.omega_uc);
                EditorGUILayout.LabelField("\x03bd Initial phase in orbit from peri. or X-axis if circular (degrees)");
                nu = EditorGUILayout.DelayedDoubleField("nu", bid.nu);
            }

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);
            if (GECore.PropAllowsEpochTime(gsBody.propagator)) {
                bool promptForEpoch = true;
                if (bid.initData == BodyInitData.InitDataType.TWO_LINE_ELEMENT) {
                    bid.useTleEpoch = EditorGUILayout.Toggle("Use TLE Epoch", bid.useTleEpoch);
                    if (bid.useTleEpoch) {
                        EditorGUILayout.LabelField("Epoch Time From TLE", EditorStyles.boldLabel);
                        EditorGUILayout.LabelField($"Epoch Time D/M/Y {bid.startEpochYear}/{bid.startEpochMonth}/{bid.startEpochDay} {bid.startEpochUtc:G6} UTC");
                        promptForEpoch = false;
                    } else {
                        EditorGUILayout.LabelField("Epoch Time From TLE will be ignored", EditorStyles.boldLabel);
                    }
                }
                if (promptForEpoch) {
                    EditorGUILayout.LabelField("Epoch Time", EditorStyles.boldLabel);
                    EditorGUILayout.LabelField("Some propagators can optinally define the epoch time for the initial state.");
                    EditorGUILayout.LabelField("If not present, the current time is used.");
                    bid.startEpochType = (BodyInitData.StartEpochType)EditorGUILayout.EnumPopup("Epoch Time Type", bid.startEpochType);
                    if (bid.startEpochType == BodyInitData.StartEpochType.WORLD_TIME) {
                        bid.startEpochWorldTime = EditorGUILayout.DelayedDoubleField("Epoch Time (seconds)", bid.startEpochWorldTime);
                    } else if (bid.startEpochType == BodyInitData.StartEpochType.DMY_UTC) {
                        bid.startEpochYear = EditorGUILayout.DelayedIntField("Epoch Year", bid.startEpochYear);
                        bid.startEpochMonth = EditorGUILayout.DelayedIntField("Epoch Month", bid.startEpochMonth);
                        bid.startEpochDay = EditorGUILayout.DelayedIntField("Epoch Day", bid.startEpochDay);
                        bid.startEpochUtc = EditorGUILayout.DelayedDoubleField("Epoch UTC", bid.startEpochUtc);
                    } else if (bid.startEpochType == BodyInitData.StartEpochType.JULIAN_DATE) {
                        bid.startEpochJD = EditorGUILayout.DelayedDoubleField("Epoch JD", bid.startEpochJD);
                    }
                }
            }
            bid.initData = dataType;
            bid.units = units_new;

            if (dataType == BodyInitData.InitDataType.TWO_LINE_ELEMENT) {
                bid.tleData = tleData;
            } else if (bid.IsRVType()) {
                bid.r = new double3(rx, ry, rz);
                bid.v = new double3(vx, vy, vz);
            } else {
                bid.a = a;
                bid.p_semi = p_semi;
                bid.eccentricity = ecc;
                bid.apoapsis = apo;
                bid.periapsis = peri;
            }
            if (orientationPrompt) {
                bid.inclination = incl;
                bid.omega_lc = o_lc;
                bid.omega_uc = o_uc;
            }
            bid.nu = nu;

        }
    }
}
