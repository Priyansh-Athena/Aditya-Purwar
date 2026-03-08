using System;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;

namespace GravityEngine2 {
    /// <summary>
    /// Propagation using the PKEPLER algorithm from Vallado. 
    /// 
    /// This models the effect of the Earth's non-spherical shape and (optionally) uses
    /// a simple atmospheric friction model. 
    /// 
    /// 
    /// </summary>
    public class PKeplerPropagator {
        public struct PropInfo {
            public Orbital.COEStruct coeKm_init;        // initial value of COE at t_start
            public double t_start;
            public int centerId;
            public int status;


            public PropInfo(Orbital.COE coeKm, double t_start, int centerId)
            {
                coeKm_init = new Orbital.COEStruct(coeKm);
                this.t_start = t_start;
                this.centerId = centerId;
                this.status = STATUS_OK;
            }

            public PropInfo(PropInfo copyFrom)
            {
                coeKm_init = copyFrom.coeKm_init;   // copies
                t_start = copyFrom.t_start;
                centerId = copyFrom.centerId;
                status = copyFrom.status;
            }

            public string LogString()
            {
                return string.Format("PKEPLER PropInfo t_start={0} centerId={1} COE={2}", t_start, centerId, coeKm_init.LogStringDegrees());
            }

        }

        public const int STATUS_OK = 0;
        public const int STATUS_EARLY_PROPAGATION_ERROR = 1;

        public static int EvolveAll(double t_to,
                        ref GEPhysicsCore.GEBodies bodies,
                         ref NativeArray<PropInfo> propInfo,
                         in NativeArray<GEPhysicsCore.PatchInfo> patchInfo,
                         in NativeArray<int> indices,
                         int lenIndices,
                         double scaleLtoKm,
                         double scaleKmsecToV,
                         double scaleTtoSec)
        {
            double3 r, v;
            Orbital.COEStruct coe;
            int p;
            int status = STATUS_OK;
            for (int j = 0; j < lenIndices; j++) {
                int i = indices[j];
                if (bodies.self_integrating[i]) {
                    continue;
                }
                if (bodies.patchIndex[i] >= 0) {
                    p = patchInfo[bodies.patchIndex[i]].propIndex;
                } else {
                    p = bodies.propIndex[i];
                }
                // time offset is different for each prop
                double dtsec = scaleTtoSec * (t_to - propInfo[p].t_start);
                if (dtsec < 0) {
                    if (bodies.earlyPropagation[i] == GEPhysicsCore.EarlyPropagation.ERROR_INACTIVATE) {
                        PropInfo pInfo = new PropInfo(propInfo[p]);
                        pInfo.status = STATUS_EARLY_PROPAGATION_ERROR;
                        propInfo[p] = pInfo;
                        status = STATUS_EARLY_PROPAGATION_ERROR;
                        continue;
                    } else if (bodies.earlyPropagation[i] == GEPhysicsCore.EarlyPropagation.HOLD_VALUE) {
                        dtsec = 0.0;
                    }
                }
                (r, v, coe) = PKeplerProp(ref propInfo, p, dtsec);
                r /= scaleLtoKm;
                v *= scaleKmsecToV;
                coe.ScaleLength(1.0 / scaleLtoKm);
                bodies.r[i] = r + bodies.r[propInfo[p].centerId];
                bodies.v[i] = v + bodies.v[propInfo[p].centerId];
                bodies.coe[i] = coe;
            }
            return status;
        }

        public static void EvolveRelative(double t_to, int propId, ref NativeArray<PropInfo> propInfo, ref GEBodyState state, double scaleTtoSec, double scaleLtoKm, double scaleKmsecToV)
        {
            double dtsec = scaleTtoSec * (t_to - propInfo[propId].t_start);
            (double3 r, double3 v, Orbital.COEStruct coe) = PKeplerProp(ref propInfo, propId, dtsec);
            r /= scaleLtoKm;
            v *= scaleKmsecToV;
            state.r = r;
            state.v = v;
            state.t = t_to;
        }

        public static void ManeuverPropagator(int bodyIndex,
                       double time,
                       ref GEPhysicsCore.GEBodies bodies,
                       ref NativeArray<PropInfo> pkProps,
                       double scaleLtoKm,
                       double scaleKmsecToV)
        {
            int p = bodies.propIndex[bodyIndex];
            int centerId = pkProps[p].centerId;
            double3 r = bodies.r[bodyIndex] - bodies.r[centerId];
            double3 v = bodies.v[bodyIndex] - bodies.v[centerId];
            // need to convert to SI units
            double3 r_km = r * scaleLtoKm;
            double3 v_kmsec = v / scaleKmsecToV;
            Orbital.COEStruct coe = new Orbital.COEStruct(Orbital.RVtoCOE(r_km, v_kmsec, pkProps[p].coeKm_init.mu));
            PropInfo pkProp = new PropInfo {
                coeKm_init = coe,
                centerId = pkProps[p].centerId,
                t_start = time
            };
            pkProps[p] = pkProp;
        }

        // actually for astPert, but leave here for now
        /* ----------------------------------------------------------------------------
		*
		*                           function pkepler
		*
		*  this function propagates a satellite's position and velocity vector over
		*    a given time period accounting for perturbations caused by j2.
		*    
		*   NBP: For ONLY J2, can set ndot and nddot to zero.
		*
		*  author        : david vallado                  719-573-2600    1 mar 2001
		*
		*  inputs          description                          range / units
		*    ro          - original position vector                    km
		*    vo          - original velocity vector                    km/sec
		*    ndot        - time rate of change of n                    rad/sec
		*    nddot       - time accel of change of n                   rad/sec2
		*    dtsec       - change in time                              sec
		*
		*  outputs       :
		*    r           - updated position vector                     km
		*    v           - updated velocity vector                     km/sec
		*
		*  locals        :
		*    p           - semi-paramter                               km
		*    a           - semior axis                                 km
		*    ecc         - eccentricity
		*    incl        - inclination                                 rad
		*    argp        - argument of periapsis                       rad
		*    argpdot     - change in argument of perigee               rad/sec
		*    raan       - longitude of the asc node                    rad
		*    raandot    - change in raan                               rad
		*    e0          - eccentric anomaly                           rad
		*    e1          - eccentric anomaly                           rad
		*    m           - mean anomaly                                rad/sec
		*    mdot        - change in mean anomaly                      rad/sec
		*    arglat      - argument of latitude                        rad
		*    arglatdot   - change in argument of latitude              rad/sec
		*    truelon     - true longitude of vehicle                  rad
		*    truelondot  - change in the true longitude                rad/sec
		*    lonper     - longitude of periapsis                      rad
		*    lonperodot  - longitude of periapsis change               rad/sec
		*    n           - mean angular motion                         rad/sec
		*    nuo         - true anomaly                                rad
		*    j2op2       - j2 over p sqyared
		*    sinv,cosv   - sine and cosine of nu
		*
		*  coupling:
		*    rv2coe      - orbit elements from position and velocity vectors
		*    coe2rv      - position and velocity vectors from orbit elements
		*    newtonm     - newton rhapson to find nu and eccentric anomaly
		*
		*  references    :
		*    vallado       2013, 691, alg 65
		*    
		*    NBP: Note comment in Vallado about taking values from TLE. They are not "as is"
		*    From Celestrak FAQ:
		*    "Field 1.9 represents the first derivative of the mean motion divided by two, in units 
		*    of revolutions per day2, and field 1.10 represents the second derivative of the mean motion 
		*    divided by six, in units of revolutions per day3. Together, these two fields give a 
		*    second-order picture of how the mean motion is changing with time. "
		* -------------------------------------------------------------------------- - */

        // NBP - values from SGP4
        static double RADIUS_EARTH = 6378.135;     // km
        static double MU_EARTH = 398600.79964;        // in km3 / s2

        static double pi = Math.PI;
        static double twopi = 2.0 * Math.PI;

        public static double LIMIT = 1e-6;

        public static (double3 r_km, double3 v_kmsec, Orbital.COEStruct coe) PKeplerProp(
                                                        ref NativeArray<PropInfo> propInfo,
                                                        int index,
                                                        double dtsec,
                                                        double ndot = 0.0,
                                                        double nddot = 0.0)
        {
            double truelondot, arglatdot, lonperdot, e0;
            double p, a, ecc, incl, raan, argp, nu, m, /* eccanom, */ arglat, truelon, lonper;
            double sqrtbeta, nbar, mdot, raandot, argpdot, sini, cosi;

            // double halfpi = Math.PI * 0.5;
            double j2 = 0.00108262617;
            // double j3 = -2.53241052e-06;
            double j4 = -1.6198976e-06;
            // double j6 = -5.40666576e-07;

            // coe.mu has been scaled by G to get desired orbit period. Since this model is for Earth, just use Earth's mass
            // rv2coe(r1, v1, p, a, ecc, incl, raan, argp, nu, m, eccanom, arglat, truelon, lonper);
            // Pass in COE

            //  fprintf(1,'          p km       a km      ecc      incl deg     raan deg     argp deg      nu deg      m deg      arglat   truelon    lonper\n');
            //  fprintf(1,'coes %11.4f%11.4f%13.9f%13.7f%11.5f%11.5f%11.5f%11.5f%11.5f%11.5f%11.5f\n',...
            //          p,a,ecc,incl*rad,raan*rad,argp*rad,nu*rad,m*rad, ...
            //          arglat*rad,truelon*rad,lonper*rad );
            // GE copy across to keep code porting simple
            a = propInfo[index].coeKm_init.a;
            ecc = propInfo[index].coeKm_init.e;
            incl = propInfo[index].coeKm_init.i;
            raan = propInfo[index].coeKm_init.omegaU;
            argp = propInfo[index].coeKm_init.omegaL;
            arglat = propInfo[index].coeKm_init.nu;      // this is what RVtoCOE would do
            truelon = propInfo[index].coeKm_init.nu;     // ditto
            lonper = propInfo[index].coeKm_init.omegaL;  // ditto
            nu = propInfo[index].coeKm_init.nu;
            m = propInfo[index].coeKm_init.meanAnom;
            // BUG: Can't see why modifying field in struct is lost
            // m = propInfo[index].coe.m;

            // coe.mu has been scaled by G to get desired orbit period. Since this model is for Earth, just use Earth's mass
            double n1 = (MU_EARTH / (a * a * a));
            double n = Math.Sqrt(n1);


            sini = Math.Sin(incl);
            cosi = Math.Cos(incl);

            // ------------- find the value of j2 perturbations -------------
            // double j2op2 = (n * 1.5 * RADIUS_EARTH * RADIUS_EARTH * j2) / (p * p);
            //     nbar    = n*( 1.0 + j2op2*Math.Sqrt(1.0-ecc*ecc)* (1.0 - 1.5*Math.Sin(incl)*Math.Sin(incl)) );
            //raandot = -j2op2 * Math.Cos(incl);
            //argpdot = j2op2 * (2.0 - 2.5 * Math.Sin(incl) * Math.Sin(incl));
            //mdot = n;

            a = a - 2.0 * ndot * dtsec * a / (3.0 * n);
            ecc = ecc - 2.0 * (1.0 - ecc) * ndot * dtsec / (3.0 * n);
            p = a * (1.0 - ecc * ecc);

            // ------------- find the value of j2, j2^2, j4 from escobal pg 371 perturbations -------------
            sqrtbeta = Math.Sqrt(1.0 - ecc * ecc);
            nbar = n * (1.5 * j2 * Math.Pow(RADIUS_EARTH / p, 2) * sqrtbeta * ((1.0 - 1.5 * sini * sini))
                + 3.0 / 128.0 * j2 * j2 * Math.Pow(RADIUS_EARTH / p, 4) * sqrtbeta * (16.0 * sqrtbeta + 25.0 * (1.0 - ecc * ecc)
                    - 15.0 + (30.0 - 96.0 * sqrtbeta - 90.0 * (1.0 - ecc * ecc)) * cosi * cosi
                    + (105.0 + 144.0 * sqrtbeta + 25.0 * (1.0 - ecc * ecc)) * Math.Pow(cosi, 4))
                - 45.0 / 128.0 * j4 * ecc * ecc * Math.Pow(RADIUS_EARTH / p, 4) * sqrtbeta * (3.0 - 30.0 * cosi * cosi + 35.0 * Math.Pow(cosi, 4)));
            mdot = n + nbar;

            raandot = -1.5 * mdot * j2 * Math.Pow(RADIUS_EARTH / p, 2) * cosi *
                (1.0 + 1.5 * j2 * Math.Pow(RADIUS_EARTH / p, 2) * (1.5 + ecc * ecc / 6.0 - 2.0 * sqrtbeta
                    - (5.0 / 3.0 - 5.0 / 24.0 * ecc * ecc - 3.0 * sqrtbeta) * sini * sini))
                - 35.0 / 8.0 * n * j4 * Math.Pow(RADIUS_EARTH / p, 4) * cosi * (1.0 + 1.5 * ecc * ecc) * ((12.0 - 21.0 * sini * sini) / 14.0);
            //	alt approach - less fractions in equations same
            //double raandote1 = -1.5 * j2 * Math.Pow(RADIUS_EARTH / p, 2) * mdot * cosi
            //	- 9.0 / 96.0 * j2 * j2 * Math.Pow(RADIUS_EARTH / p, 4) * mdot * cosi * (36.0 + 4.0 * ecc * ecc - 48.0 * sqrtbeta
            //		- (40.0 - 5.0 * ecc * ecc - 72.0 * sqrtbeta) * sini * sini)
            //	- 35.0 / 112.0 * j4 * Math.Pow(RADIUS_EARTH / p, 4) * n * cosi * (1.0 + 1.5 * ecc * ecc) * (12.0 - 21.0 * sini * sini);


            argpdot = 1.5 * mdot * j2 * Math.Pow(RADIUS_EARTH / p, 2) * (2.0 - 2.5 * sini * sini) *
                (1.0 + 1.5 * j2 * Math.Pow(RADIUS_EARTH / p, 2) * (2.0 + ecc * ecc / 2.0 - 2.0 * sqrtbeta
                    - (43.0 / 24.0 - ecc * ecc / 48.0 - 3.0 * sqrtbeta) * sini * sini))
                - 45.0 / 36.0 * j2 * j2 * mdot * Math.Pow(RADIUS_EARTH / p, 4) * ecc * ecc * Math.Pow(cosi, 4)
                - 35.0 / 8.0 * n * j4 * Math.Pow(RADIUS_EARTH / p, 4) * (12.0 / 7.0 - 93.0 / 14.0 * sini * sini
                    + 21.0 / 4.0 * Math.Pow(sini, 4) + ecc * ecc * (27.0 / 14.0 - 189.0 / 28.0 * sini * sini + 81.0 / 16.0 * Math.Pow(sini, 4)));
            // same
            //double argpdote1 = 0.75 * j2 * Math.Pow(RADIUS_EARTH / p, 2) * mdot * (4.0 - 5.0 * sini * sini)
            //	+ 9.0 / 192.0 * j2 * j2 * Math.Pow(RADIUS_EARTH / p, 4) * mdot * (2.0 - 2.5 * sini * sini) * (96.0 + 24.0 * ecc * ecc
            //		- 96.0 * sqrtbeta - (86.0 - ecc * ecc - 144.0 * sqrtbeta) * sini * sini)
            //	- 45.0 / 36.0 * j2 * j2 * Math.Pow(RADIUS_EARTH / p, 4) * ecc * ecc * n * Math.Pow(cosi, 4)
            //	- 35.0 / 896.0 * j4 * Math.Pow(RADIUS_EARTH / p, 4) * n * (192.0 - 744.0 * sini * sini
            //		+ 588 * Math.Pow(sini, 4) + ecc * ecc * (216.0 - 756.0 * sini * sini + 567.0 * Math.Pow(sini, 4)));

            // ----- update the orbital elements for each orbit type --------
            if (ecc < LIMIT) {
                //  -------------  circular equatorial  ----------------
                if ((incl < LIMIT) | (Math.Abs(incl - Math.PI) < LIMIT)) {
                    truelondot = raandot + argpdot + mdot;
                    truelon = truelon + truelondot * dtsec;
                    truelon = truelon % twopi;
                } else {
                    //  -------------  circular inclined    --------------
                    raan = raan + raandot * dtsec;
                    raan = raan % twopi;
                    arglatdot = argpdot + mdot;
                    arglat = arglat + arglatdot * dtsec;
                    arglat = (arglat % twopi);
                }
            } else {
                //  ---- elliptical, parabolic, hyperbolic equatorial ---
                if ((incl < LIMIT) | (Math.Abs(incl - pi) < LIMIT)) {
                    lonperdot = raandot + argpdot;
                    lonper = lonper + lonperdot * dtsec;
                    lonper = lonper % twopi;
                    m = m + mdot * dtsec + ndot * dtsec * dtsec + nddot * Math.Pow(dtsec, 3);
                    m = (m % twopi);
                    (e0, nu) = NewtonM(ecc, m);
                } else {
                    //  ---- elliptical, parabolic, hyperbolic inclined --
                    raan = raan + raandot * dtsec;
                    raan = (raan % twopi);
                    argp = argp + argpdot * dtsec;
                    argp = (argp % twopi);
                    m = m + mdot * dtsec + ndot * dtsec * dtsec + nddot * dtsec * dtsec * dtsec;
                    m = (m % twopi);
                    (e0, nu) = NewtonM(ecc, m);
                }
            }
            // ------------- use coe2rv to find new vectors ---------------

            // coe2rv(p, ecc, incl, raan, argp, nu, arglat, truelon, lonper, r2, v2);
            Orbital.COEStruct coe2 = new Orbital.COEStruct();
            coe2.p = p;
            coe2.a = a;
            coe2.e = ecc;
            coe2.i = incl;
            coe2.omegaU = raan;
            coe2.omegaL = argp;
            coe2.nu = nu;
            coe2.meanAnom = m;
            coe2.mu = propInfo[index].coeKm_init.mu;
            // special cases. COE reuses omegaL for lonper
            bool flat = (propInfo[index].coeKm_init.i < LIMIT) || (Math.Abs(propInfo[index].coeKm_init.i - Math.PI) < LIMIT);
            if (coe2.e < LIMIT) {
                if (flat) {
                    // circular, equitorial
                    coe2.nu = truelon;
                    coe2.omegaU = 0.0;
                } else {
                    // circular, inclined
                    coe2.nu = arglat;
                }
            } else if (flat) {
                coe2.omegaL = lonper;
                coe2.omegaU = 0.0;
            }
            coe2.ComputeRotation();
            // Q: What about mu??
            (double3 r2, double3 v2) = Orbital.COEStructWithPhasetoRVRelative(coe2, coe2.nu);
            return (r2, v2, coe2);

        }   // pkepler

        /* ----------------------------------------------------------------------------
		*
		*                           procedure newtonm
		*
		*  this procedure performs the newton rhapson iteration to find the
		*    eccentric anomaly given the mean anomaly.  the true anomaly is also
		*    calculated.
		*
		*  author        : david vallado                  719-573-2600    1 mar 2001
		*
		*  inputs          description                         range / units
		*    ecc         - eccentricity                            0.0 to
		*    m           - mean anomaly                        -2pi to 2pi rad
		*
		*  outputs       :
		*    e0          - eccentric anomaly                    0.0 to 2pi rad
		*    nu          - true anomaly                        0.0 to 2pi rad
		*
		*  locals        :
		*    e1          - eccentric anomaly, next value               rad
		*    sinv        - sine of nu
		*    cosv        - cosine of nu
		*    ktr         - index
		*    r1r         - cubic roots - 1 to 3
		*    r1i         - imaginary component
		*    r2r         -
		*    r2i         -
		*    r3r         -
		*    r3i         -
		*    s           - variables for parabolic solution
		*    w           - variables for parabolic solution
		*
		*  coupling      :
		*    atan2       - arc tangent function which also resloves quadrants
		*    cubic       - solves a cubic polynomial
		*    power       - raises a base number to an arbitrary power
		*    sinh        - hyperbolic sine
		*    cosh        - hyperbolic cosine
		*    sgn         - returns the MathTimeLib::sgn of an argument
		*
		*  references    :
		*    vallado       2013, 65, alg 2, ex 2-1
		* --------------------------------------------------------------------------- */

        private static (double, double) NewtonM
        (
            double ecc, double m
        )
        {
            const int numiter = 50;
            const double small = 0.00000001;       // small value for tolerances
            double e0, nu;

            double e1, sinv, cosv, cose1, coshe1, temp, r1r = 0.0;
            int ktr;

            // -------------------------- hyperbolic  ----------------------- 
            if ((ecc - 1.0) > small) {
                // ------------  initial guess ------------- 
                if (ecc < 1.6)
                    if (((m < 0.0) && (m > -pi)) || (m > pi))
                        e0 = m - ecc;
                    else
                        e0 = m + ecc;
                else
                    if ((ecc < 3.6) && (Math.Abs(m) > pi)) // just edges)
                    e0 = m - Math.Sign(m) * ecc;
                else
                    e0 = m / (ecc - 1.0); // best over 1.8 in middle
                ktr = 1;
                e1 = e0 + ((m - ecc * Math.Sinh(e0) + e0) / (ecc * Math.Cosh(e0) - 1.0));
                while ((Math.Abs(e1 - e0) > small) && (ktr <= numiter)) {
                    e0 = e1;
                    e1 = e0 + ((m - ecc * Math.Sinh(e0) + e0) / (ecc * Math.Cosh(e0) - 1.0));
                    ktr++;
                }
                // ---------  find true anomaly  ----------- 
                coshe1 = Math.Cosh(e1);
                sinv = -(Math.Sqrt(ecc * ecc - 1.0) * Math.Sinh(e1)) / (1.0 - ecc * coshe1);
                cosv = (coshe1 - ecc) / (1.0 - ecc * coshe1);
                nu = Math.Atan2(sinv, cosv);
            } else {
                // ---------------------- parabolic ------------------------- 
                if (Math.Abs(ecc - 1.0) < small) {
                    //kbn      cubic(1.0 / 3.0, 0.0, 1.0, -m, r1r, r1i, r2r, r2i, r3r, r3i);
                    e0 = r1r;
                    //kbn      if (fileout != null)
                    //        fprintf(fileout, "roots %11.7f %11.7f %11.7f %11.7f %11.7f %11.7f\n",
                    //                          r1r, r1i, r2r, r2i, r3r, r3i);
                    /*
						 s  = 0.5 * (halfpi - atan(1.5 * m));
						 w  = atan(power(tan(s), 1.0 / 3.0));
						 e0 = 2.0 * cot(2.0* w );
					*/
                    ktr = 1;
                    nu = 2.0 * Math.Atan(e0);
                } else {
                    // --------------------- elliptical --------------------- 
                    if (ecc > small) {
                        // ------------  initial guess ------------- 
                        if (((m < 0.0) && (m > -pi)) || (m > pi))
                            e0 = m - ecc;
                        else
                            e0 = m + ecc;
                        ktr = 1;
                        e1 = e0 + (m - e0 + ecc * Math.Sin(e0)) / (1.0 - ecc * Math.Cos(e0));
                        while ((Math.Abs(e1 - e0) > small) && (ktr <= numiter)) {
                            ktr++;
                            e0 = e1;
                            e1 = e0 + (m - e0 + ecc * Math.Sin(e0)) / (1.0 - ecc * Math.Cos(e0));
                        }
                        // ---------  find true anomaly  ----------- 
                        cose1 = Math.Cos(e1);
                        temp = 1.0 / (1.0 - ecc * cose1);
                        sinv = (Math.Sqrt(1.0 - ecc * ecc) * Math.Sin(e1)) * temp;
                        cosv = (cose1 - ecc) * temp;
                        nu = Math.Atan2(sinv, cosv);
                    } else {
                        // --------------------- circular --------------------- 
                        ktr = 0;
                        nu = m;
                        e0 = m;
                    }
                }
            }
            if (ktr > numiter)
                Debug.LogWarningFormat("newtonrhapson not converged in {0} iterations\n", numiter);

            return (e0, nu);
        }    // procedure newtonm
    }
}
