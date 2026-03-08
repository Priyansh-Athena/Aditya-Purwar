using Unity.Mathematics;
using UnityEngine;

namespace GravityEngine2 {
    public class SolarSystemTools : MonoBehaviour {
        /* ------------------------------------------------------------------------------
        *
        *                           function sun
        *
        *  this function calculates the geocentric equatorial position vector
        *    the sun given the julian date. Sergey K (2022) has noted that improved results 
        *    are found assuming the oputput is in a precessing frame (TEME) and converting to ICRF. 
        *    this is the low precision formula and is valid for years from 1950 to 2050.  
        *    accuaracy of apparent coordinates is about 0.01 degrees.  notice many of 
        *    the calculations are performed in degrees, and are not changed until later.  
        *    this is due to the fact that the almanac uses degrees exclusively in their formulations.
        *
        *  author        : david vallado           davallado@gmail.com   27 may 2002
        *
        *  revisions
        *    vallado     - fix mean lon of sun                            7 may 2004
        *  
        *  inputs          description                         range / units
        *    jd          - julian date  (UTC)                       days from 4713 bc
        *
        *  outputs       :
        *    rsun        - inertial position vector of the sun      au
        *    rtasc       - right ascension                          rad
        *    decl        - declination                              rad
        *
        *  locals        :
        *    meanlong    - mean longitude
        *    meananomaly - mean anomaly
        *    eclplong    - ecliptic longitude
        *    obliquity   - mean obliquity of the ecliptic
        *    tut1        - julian centuries of ut1 from
        *                  jan 1, 2000 12h
        *    ttdb        - julian centuries of tdb from
        *                  jan 1, 2000 12h
        *    hr          - hours                                    0 .. 24              10
        *    min         - minutes                                  0 .. 59              15
        *    sec         - seconds                                  0.0  .. 59.99          30.00
        *    temp        - temporary variable
        *    deg         - degrees
        *
        *  coupling      :
        *    none.
        *
        *  references    :
        *    vallado       2013, 279, alg 29, ex 5-1
        * --------------------------------------------------------------------------- */



        public static (double3 rsunAU, double ra, double decl) Sun(double jd)
        {
            double twopi, deg2rad;
            double tut1, meanlong, ttdb, meananomaly, eclplong, obliquity, magr;

            // needed since assignments arn't at root level in procedure
            double3 rsun = double3.zero;

            twopi = 2.0 * math.PI;
            deg2rad = math.PI / 180.0;

            // -------------------------  implementation   -----------------
            // -------------------  initialize values   --------------------
            tut1 = (jd - 2451545.0) / 36525.0;

            meanlong = 280.460 + 36000.77 * tut1;
            meanlong = (meanlong % 360.0);  //deg

            ttdb = tut1;
            meananomaly = 357.5277233 + 35999.05034 * ttdb;
            meananomaly = ((meananomaly * deg2rad) % twopi);  //rad
            if (meananomaly < 0.0) {
                meananomaly = twopi + meananomaly;
            }
            eclplong = meanlong + 1.914666471 * math.sin(meananomaly)
                        + 0.019994643 * math.sin(2.0 * meananomaly); //deg
            obliquity = 23.439291 - 0.0130042 * ttdb;  //deg
            meanlong = meanlong * deg2rad;
            if (meanlong < 0.0) {
                meanlong = twopi + meanlong;
            }
            eclplong = eclplong * deg2rad;
            obliquity = obliquity * deg2rad;

            // --------- find magnitude of sun vector, )   components ------
            magr = 1.000140612 - 0.016708617 * math.cos(meananomaly)
                               - 0.000139589 * math.cos(2.0 * meananomaly);    // in au's

            rsun = new double3(magr * math.cos(eclplong),
                                magr * math.cos(obliquity) * math.sin(eclplong),
                                magr * math.sin(obliquity) * math.sin(eclplong));

            double rtasc = math.atan(math.cos(obliquity) * math.tan(eclplong));

            // --- check that rtasc is in the same quadrant as eclplong ----
            if (eclplong < 0.0) {
                eclplong = eclplong + twopi;    // make sure it's in 0 to 2pi range
            }
            if (math.abs(eclplong - rtasc) > math.PI * 0.5) {
                rtasc = rtasc + 0.5 * math.PI * math.round((eclplong - rtasc) / (0.5 * math.PI));
            }
            double decl = math.asin(math.sin(obliquity) * math.sin(eclplong));
            return (rsun, rtasc, decl);
        }  // sun


        /* -----------------------------------------------------------------------------
        *
        *                           function moon
        *
        *  this function calculates the geocentric equatorial (ijk) position vector
        *    for the moon given the julian date.
        *
        *  author        : david vallado           davallado@gmail.com   27 may 2002
        *
        *  revisions
        *                -
        *
        *  inputs          description                             range / units
        *    jd          - julian date                              days from 4713 bc
        *
        *  outputs       :
        *    rmoon       - ijk position vector of moon              km
        *    rtasc       - right ascension                          rad
        *    decl        - declination                              rad
        *
        *  locals        :
        *    eclplong    - ecliptic longitude
        *    eclplat     - eclpitic latitude
        *    hzparal     - horizontal parallax
        *    l           - geocentric direction math.cosines
        *    m           -             "     "
        *    n           -             "     "
        *    ttdb        - julian centuries of tdb from
        *                  jan 1, 2000 12h
        *    hr          - hours                                    0 .. 24
        *    min         - minutes                                  0 .. 59
        *    sec         - seconds                                  0.0  .. 59.99
        *    deg         - degrees
        *
        *  coupling      :
        *    none.
        *
        *  references    :
        *    vallado       2013, 288, alg 31, ex 5-3
        * --------------------------------------------------------------------------- */



        public static (double3 rKm, double rAscRad, double declRad) Moon(double jd)
        {
            double twopi, deg2rad, magr;
            double ttdb, l, m, n, eclplong, eclplat, hzparal, obliquity;
            // needed since assignments arn't at root level in procedure
            double3 rmoon = double3.zero;

            twopi = 2.0 * math.PI;
            deg2rad = math.PI / 180.0;

            // -------------------------  implementation   -----------------
            ttdb = (jd - 2451545.0) / 36525.0;

            eclplong = 218.32 + 481267.8813 * ttdb
                        + 6.29 * math.sin((134.9 + 477198.85 * ttdb) * deg2rad)
                        - 1.27 * math.sin((259.2 - 413335.38 * ttdb) * deg2rad)
                        + 0.66 * math.sin((235.7 + 890534.23 * ttdb) * deg2rad)
                        + 0.21 * math.sin((269.9 + 954397.70 * ttdb) * deg2rad)
                        - 0.19 * math.sin((357.5 + 35999.05 * ttdb) * deg2rad)
                        - 0.11 * math.sin((186.6 + 966404.05 * ttdb) * deg2rad);      // deg

            eclplat = 5.13 * math.sin((93.3 + 483202.03 * ttdb) * deg2rad)
                        + 0.28 * math.sin((228.2 + 960400.87 * ttdb) * deg2rad)
                        - 0.28 * math.sin((318.3 + 6003.18 * ttdb) * deg2rad)
                        - 0.17 * math.sin((217.6 - 407332.20 * ttdb) * deg2rad);      // deg

            hzparal = 0.9508 + 0.0518 * math.cos((134.9 + 477198.85 * ttdb)
                       * deg2rad)
                      + 0.0095 * math.cos((259.2 - 413335.38 * ttdb) * deg2rad)
                      + 0.0078 * math.cos((235.7 + 890534.23 * ttdb) * deg2rad)
                      + 0.0028 * math.cos((269.9 + 954397.70 * ttdb) * deg2rad);    // deg

            eclplong = ((eclplong * deg2rad) % twopi);
            eclplat = ((eclplat * deg2rad) % twopi);
            hzparal = ((hzparal * deg2rad) % twopi);

            obliquity = 23.439291 - 0.0130042 * ttdb;  //deg
            obliquity = obliquity * deg2rad;

            // ------------ find the geocentric direction math.cosines ----------
            l = math.cos(eclplat) * math.cos(eclplong);
            m = math.cos(obliquity) * math.cos(eclplat) * math.sin(eclplong) - math.sin(obliquity) * math.sin(eclplat);
            n = math.sin(obliquity) * math.cos(eclplat) * math.sin(eclplong) + math.cos(obliquity) * math.sin(eclplat);

            // ------------- calculate moon position vector ----------------
            magr = 1.0 / math.sin(hzparal);
            rmoon[0] = magr * l;
            rmoon[1] = magr * m;
            rmoon[2] = magr * n;

            // -------------- find rt ascension and declination ------------
            double rtasc = math.atan2(m, l);
            double decl = math.asin(n);
            return (rmoon, rtasc, decl);
        }  // moon

    }
}
