using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using Unity.Mathematics;

namespace GravityEngine2 {

    public class LambertBattin {
        private double3 r1, r2, v1, v2;

        private double mu;

        private bool checkPlanetRadius;
        private double planetRadius;

        private Orbital.COE toOrbit;

        public const int IMPACT = 4;

        // private NBody fromBody = null;

        // center position of the orbit. r1 and r2 are with respect to this position
        // private double3 center3d;

        private List<GEManeuver> maneuvers;

        //! Velocity of target when rendezvous is used.
        private double3 targetVelocity;
        private enum ComputeMode { USE_R2, INTERCEPT, RENDEZVOUS };

        public LambertBattin(Orbital.COE fromOrbit, Orbital.COE toOrbit)
        {

            // Fundamentals of Astrodynamics and Applications, Vallado, 4th Ed., Algorithm 56 p475 
            // Take r0, r => a_min, e_min, t_min, v0
            // center3d = ge.GetPositionDoubleV3(_fromOrbit.centralMass);

            mu = fromOrbit.mu;
            this.toOrbit = toOrbit;
            (r1, v1) = Orbital.COEtoRVRelative(fromOrbit);
            (r2, v2) = Orbital.COEtoRVRelative(toOrbit);

            // Test debug
            // Debug.LogFormat("Init: from = {0}\n, to={1}\n r1={2}, r2={3}", fromOrbit.LogString(), toOrbit.LogString(), r1, r2);
        }

        public List<GEManeuver> Maneuvers(int centerId = -1)
        {
            if (centerId >= 0) {
                foreach (GEManeuver m in maneuvers)
                    m.centerId = centerId;
            }
            return maneuvers;
        }

        public LambertBattin(Orbital.COE fromOrbit, double3 r2)
        {
            mu = fromOrbit.mu;
            (r1, v1) = Orbital.COEtoRVRelative(fromOrbit);
            this.r2 = r2;
        }


        private double Fmod(double a, double b)
        {
            return a % b;
        }

        /* ----------------------- lambert techniques -------------------- */

        /* utility functions for lambertbattin, etc */
        /* -------------------------------------------------------------------------- */
        // ------------------------------------------------------------------------------
        //                           function lambhodograph
        //
        // this function accomplishes 180 deg transfer(and 360 deg) for lambert problem.
        //
        //  author        : david vallado                  719 - 573 - 2600   22 may 2017
        //
        //  inputs          description                    range / units
        //    r1 - ijk position vector 1          km
        //    r2 - ijk position vector 2          km
        //    dtsec - time between r1 and r2         s
        //    dnu - true anomaly change            rad
        //
        //  outputs       :
        //    v1t - ijk transfer velocity vector   km / s
        //    v2t - ijk transfer velocity vector   km / s
        //
        //  references :
        //    Thompson JGCD 2013 v34 n6 1925
        // Thompson AAS GNC 2018
        // [v1t, v2t] = lambhodograph(r1, v1, r2, p, a, ecc, dnu, dtsec)
        // ------------------------------------------------------------------------------

        private void LambHodograph
                (
                double p,
                double ecc,
                double dnu,
                double dtsec
                )
        {
            double eps, magr1, magr2, a, b, x1, x2, y2a, y2b, ptx;
            double3 rcrv, rcrr, nvec;

            eps = 1.0e-6;  // Had issue with sin(3.1415..) being more that 1E-8

            magr1 = math.length(r1);
            magr2 = math.length(r2);

            a = mu * (1.0 / magr1 - 1.0 / p);  // not the semi - major axis
            b = Math.Pow(mu * ecc / p, 2) - a * a;
            if (b <= 0.0)
                x1 = 0.0;
            else
                x1 = -Math.Sqrt(b);

            // 180 deg, and multiple 180 deg transfers
            if (math.abs(math.sin(dnu)) < eps) {
                nvec = math.normalize(math.cross(r1, v1));
                if (ecc < 1.0) {
                    ptx = 2.0 * Math.PI * Math.Sqrt(p * p * p / Math.Pow(mu * (1.0 - ecc * ecc), 3));
                    if (Fmod(dtsec, ptx) > ptx * 0.5)
                        x1 = -x1;
                }
            } else {
                y2a = mu / p - x1 * Math.Sin(dnu) + a * Math.Cos(dnu);
                y2b = mu / p + x1 * Math.Sin(dnu) + a * Math.Cos(dnu);
                if (Math.Abs(mu / magr2 - y2b) < Math.Abs(mu / magr2 - y2a))
                    x1 = -x1;

                // depending on the cross product, this will be normal or in plane,
                // or could even be a fan
                rcrr = math.cross(r1, r2);
                nvec = math.normalize(rcrr); // if this is r1, v1, the transfer is coplanar!
                if (Fmod(dnu, 2.0 * Math.PI) > Math.PI) {
                    nvec = -1.0 * nvec;
                }
            }

            rcrv = math.cross(nvec, r1);
            rcrr = math.cross(nvec, r2);
            x2 = x1 * Math.Cos(dnu) + a * Math.Sin(dnu);
            v1t = (Math.Sqrt(mu * p) / magr1) * ((x1 / mu) * r1 + rcrv) / magr1;
            v2t = (Math.Sqrt(mu * p) / magr2) * ((x2 / mu) * r2 + rcrr) / magr2;
        }  // lambhodograph


        private static double KBattin
        (
        double v
        )
        {
            double[] d = new double[21]
            {
        1.0 / 3.0, 4.0 / 27.0,
            8.0 / 27.0, 2.0 / 9.0,
            22.0 / 81.0, 208.0 / 891.0,
            340.0 / 1287.0, 418.0 / 1755.0,
            598.0 / 2295.0, 700.0 / 2907.0,
            928.0 / 3591.0, 1054.0 / 4347.0,
            1330.0 / 5175.0, 1480.0 / 6075.0,
            1804.0 / 7047.0, 1978.0 / 8091.0,
            2350.0 / 9207.0, 2548.0 / 10395.0,
            2968.0 / 11655.0, 3190.0 / 12987.0,
            3658.0 / 14391.0
            };
            double del, delold, term, termold, sum1;
            int i;

            /* ---- process forwards ---- */
            sum1 = d[0];
            delold = 1.0;
            termold = d[0];
            i = 1;
            while ((i <= 20) && (Math.Abs(termold) > 0.00000001)) {
                del = 1.0 / (1.0 - d[i] * v * delold);
                term = termold * (del - 1.0);
                sum1 = sum1 + term;
                i++;
                delold = del;
                termold = term;
            }
            //return sum1;

            int ktr = 20;
            double sum2 = 0.0;
            double term2 = 1.0 + d[ktr] * v;
            for (i = 1; i <= ktr - 1; i++) {
                sum2 = d[ktr - i] * v / term2;
                term2 = 1.0 + sum2;
            }

            return (d[0] / term2);
        }  // double kbattin


        /* -------------------------------------------------------------------------- */

        static double SeeBattin(double v2)
        {
            double[] c = new double[21]

                {
            0.2,
            9.0 / 35.0, 16.0 / 63.0,
            25.0 / 99.0, 36.0 / 143.0,
            49.0 / 195.0, 64.0 / 255.0,
            81.0 / 323.0, 100.0 / 399.0,
            121.0 / 483.0, 144.0 / 575.0,
            169.0 / 675.0, 196.0 / 783.0,
            225.0 / 899.0, 256.0 / 1023.0,
            289.0 / 1155.0, 324.0 / 1295.0,
            361.0 / 1443.0, 400.0 / 1599.0,
            441.0 / 1763.0, 484.0 / 1935.0

                };
            // first term is diff, indices are offset too
            double[] c1 = new double[20]

            {
            9.0 / 7.0, 16.0 / 63.0,
            25.0 / 99.0, 36.0 / 143.0,
            49.0 / 195.0, 64.0 / 255.0,
            81.0 / 323.0, 100.0 / 399.0,
            121.0 / 483.0, 144.0 / 575.0,
            169.0 / 675.0, 196.0 / 783.0,
            225.0 / 899.0, 256.0 / 1023.0,
            289.0 / 1155.0, 324.0 / 1295.0,
            361.0 / 1443.0, 400.0 / 1599.0,
            441.0 / 1763.0, 484.0 / 1935.0

            };

            double term, termold, del, delold, sum1, eta, sqrtopv;
            int i;

            sqrtopv = Math.Sqrt(1.0 + v2);
            eta = v2 / Math.Pow(1.0 + sqrtopv, 2);

            /* ---- process forwards ---- */
            delold = 1.0;
            termold = c[0];  // * eta
            sum1 = termold;
            i = 1;
            while ((i <= 20) && (Math.Abs(termold) > 0.000001)) {
                del = 1.0 / (1.0 + c[i] * eta * delold);
                term = termold * (del - 1.0);
                sum1 = sum1 + term;
                i++;
                delold = del;
                termold = term;
            }

            //   return ((1.0 / (8.0 * (1.0 + sqrtopv))) * (3.0 + sum1 / (1.0 + eta * sum1)));
            // double seebatt = 1.0 / ((1.0 / (8.0 * (1.0 + sqrtopv))) * (3.0 + sum1 / (1.0 + eta * sum1)));

            int ktr = 19;
            double sum2 = 0.0;
            double term2 = 1.0 + c1[ktr] * eta;
            for (i = 0; i <= ktr - 1; i++) {
                sum2 = c1[ktr - i] * eta / term2;
                term2 = 1.0 + sum2;
            }

            return (8.0 * (1.0 + sqrtopv) /
                (3.0 +
                (1.0 /
                (5.0 + eta + ((9.0 / 7.0) * eta / term2)))));
        }  // double seebattin


        /*------------------------------------------------------------------------------
       *
       *                           procedure lamberbattin
       *
       *  this procedure solves lambert's problem using battins method. the method is
       *    developed in battin (1987).
       *
       *  author        : david vallado                  719-573-2600   22 jun 2002
       *
       *  inputs          description                    range / units
       *    r1          - ijk position vector 1          km
       *    r2           - ijk position vector 2          km
       *   dm          - direction of motion            'l','s'
       *    dtsec        - time between r1 and r2         sec
       *
       *  outputs       :
       *    v1          - ijk velocity vector            er / tu
       *    v2           - ijk velocity vector            er / tu
       *    error       - error flag                     1, 2, 3, ... use numbers since c++ is so horrible at strings
       *        error = 1;   // a = 0.0
       *
       *  locals        :
       *    i           - index
       *    loops       -
       *    u           -
       *    b           -
       *    sinv        -
       *    cosv        -
       *    rp          -
       *    x           -
       *    xn          -
       *    y           -
       *    l           -
       *    m           -
       *    cosdeltanu  -
       *    sindeltanu  -
       *    dnu         -
       *    a           -
       *    tan2w       -
       *    ror         -
       *    h1          -
       *    h2          -
       *    tempx       -
       *    eps         -
       *    denom       -
       *    chord       -
       *    k2          -
       *    s           -
       *
       *  coupling      :
       *    arcsin      - arc sine function
       *    arccos      - arc cosine function
       *    astMath::mag         - astMath::magnitude of a vector
       *    arcsinh     - inverse hyperbolic sine
       *    arccosh     - inverse hyperbolic cosine
       *    sinh        - hyperbolic sine
       *    power       - raise a base to a power
       *    atan2       - arc tangent function that resolves quadrants
       *
       *  references    :
       *    vallado       2013, 494, Alg 59, ex 7-5
       -----------------------------------------------------------------------------*/

        //! Initial velocity for maneuver
        private double3 v1t;

        //! velocity when final point is reached
        private double3 v2t;

        public double3 GetTransferVelocity()
        {
            return v1t;
        }

        public double3 GetFinalVelocity()
        {
            return v2t;
        }

        private int ComputeXferInternal(
                // r1, r2, v1 set by constructor
                bool reverse, // was dm 'l' or 's'
                bool df,      // 'r' for retro case "alt approach for high energy(long way, retro multi - rev) case"
                int nrev,
                double xferTime,
                LambertMode mode
                )
        {
            const double small = 0.000001;
            double3 rcrossr;
            int loops;
            double u, b, x, xn, y, L, m, cosdeltanu, sindeltanu, dnu, a,
                ror, h1, h2, tempx, eps, denom, chord, k2, s,
                p, ecc, f, A;
            double magr1, magr2, magrcrossr, lam, temp, temp1, temp2;

            y = 0;

            int error = 0; // PM - added an error code for loop not converging. Seems Vallado did not implement any?
            magr1 = math.length(r1);
            magr2 = math.length(r2);

            cosdeltanu = math.dot(r1, r2) / (magr1 * magr2);
            // make sure it's not more than 1.0
            if (Math.Abs(cosdeltanu) > 1.0)
                cosdeltanu = 1.0 * Math.Sign(cosdeltanu);

            rcrossr = math.cross(r1, r2);
            magrcrossr = math.length(rcrossr);
            if (!reverse)
                sindeltanu = magrcrossr / (magr1 * magr2);
            else
                sindeltanu = -magrcrossr / (magr1 * magr2);

            dnu = Math.Atan2(sindeltanu, cosdeltanu);
            // the angle needs to be positive to work for the long way
            if (dnu < 0.0)
                dnu = 2.0 * math.PI + dnu;

            // these are the same
            //chord = Math.Sqrt(magr1 * magr1 + magr2 * magr2 - 2.0 * magr1 * magr2 * cosdeltanu);
            chord = math.length(r2 - r1);

            s = (magr1 + magr2 + chord) * 0.5;
            ror = magr2 / magr1;
            eps = ror - 1.0;

            lam = 1.0 / s * Math.Sqrt(magr1 * magr2) * Math.Cos(dnu * 0.5);
            L = Math.Pow((1.0 - lam) / (1.0 + lam), 2);
            m = 8.0 * mu * xferTime * xferTime / (s * s * s * Math.Pow(1.0 + lam, 6));

            // initial guess
            if (nrev > 0)
                xn = 1.0 + 4.0 * L;
            else
                xn = L;   //l    // 0.0 for par and hyp, l for ell

            // alt approach for high energy(long way, retro multi - rev) case
            if (df && (nrev > 0)) {
                xn = 1e-20;  // be sure to reset this here!!
                x = 10.0;  // starting value
                loops = 1;
                while ((Math.Abs(xn - x) >= small) && (loops <= 20)) {
                    x = xn;
                    temp = 1.0 / (2.0 * (L - x * x));
                    temp1 = Math.Sqrt(x);
                    temp2 = (nrev * Math.PI * 0.5 + Math.Atan(temp1)) / temp1;
                    h1 = temp * (L + x) * (1.0 + 2.0 * x + L);
                    h2 = temp * m * temp1 * ((L - x * x) * temp2 - (L + x));

                    b = 0.25 * 27.0 * h2 / (Math.Pow(temp1 * (1.0 + h1), 3));
                    if (b < -1.0) // reset the initial condition
                        f = 2.0 * Math.Cos(1.0 / 3.0 * Math.Acos(Math.Sqrt(b + 1.0)));
                    else {
                        A = Math.Pow(Math.Sqrt(b) + Math.Sqrt(b + 1.0), (1.0 / 3.0));
                        f = A + 1.0 / A;
                    }

                    y = 2.0 / 3.0 * temp1 * (1.0 + h1) * (Math.Sqrt(b + 1.0) / f + 1.0);
                    xn = 0.5 * ((m / (y * y) - (1.0 + L)) - Math.Sqrt(Math.Pow(m / (y * y) - (1.0 + L), 2) - 4.0 * L));
                    // fprintf(outfile, " %3i yh %11.6f x %11.6f h1 %11.6f h2 %11.6f b %11.6f f %11.7f \n", loops, y, x, h1, h2, b, f);
                    loops = loops + 1;
                }  // while
                x = xn;
                a = s * Math.Pow(1.0 + lam, 2) * (1.0 + x) * (L + x) / (8.0 * x);
                p = (2.0 * magr1 * magr2 * (1.0 + x) * Math.Pow(Math.Sin(dnu * 0.5), 2)) / (s * Math.Pow(1.0 + lam, 2) * (L + x));  // thompson
                ecc = Math.Sqrt(1.0 - p / a);
                LambHodograph(p, ecc, dnu, xferTime);
                // fprintf(outfile, "high v1t %16.8f %16.8f %16.8f %16.8f\n", v1t, astMath::mag(v1t));
            } else {
                // standard processing
                // note that the dr nrev = 0 case is not represented
                loops = 1;
                x = 10.0;  // starting value
                while ((Math.Abs(xn - x) >= small) && (loops <= 30)) {
                    if (nrev > 0) {
                        x = xn;
                        temp = 1.0 / ((1.0 + 2.0 * x + L) * (4.0 * x));
                        temp1 = (nrev * Math.PI * 0.5 + Math.Atan(Math.Sqrt(x))) / Math.Sqrt(x);
                        h1 = temp * Math.Pow(L + x, 2) * (3.0 * Math.Pow(1.0 + x, 2) * temp1 - (3.0 + 5.0 * x));
                        h2 = temp * m * ((x * x - x * (1.0 + L) - 3.0 * L) * temp1 + (3.0 * L + x));
                    } else {
                        x = xn;
                        tempx = SeeBattin(x);
                        denom = 1.0 / ((1.0 + 2.0 * x + L) * (4.0 * x + tempx * (3.0 + x)));
                        h1 = Math.Pow(L + x, 2) * (1.0 + 3.0 * x + tempx) * denom;
                        h2 = m * (x - L + tempx) * denom;
                    }

                    // ---------------------- - evaluate cubic------------------
                    b = 0.25 * 27.0 * h2 / (Math.Pow(1.0 + h1, 3));

                    u = 0.5 * b / (1.0 + Math.Sqrt(1.0 + b));
                    k2 = KBattin(u);
                    y = ((1.0 + h1) / 3.0) * (2.0 + Math.Sqrt(1.0 + b) / (1.0 + 2.0 * u * k2 * k2));
                    xn = Math.Sqrt(Math.Pow((1.0 - L) * 0.5, 2) + m / (y * y)) - (1.0 + L) * 0.5;

                    loops = loops + 1;
                }  // while

            }

            if (loops < 30) {
                // blair approach use y from solution
                //       lam = 1.0 / s * sqrt(magr1*magr2) * cos(dnu*0.5);
                //       m = 8.0*mu*dtsec*dtsec / (s ^ 3 * (1.0 + lam) ^ 6);
                //       L = ((1.0 - lam) / (1.0 + lam)) ^ 2;
                //a = s*(1.0 + lam) ^ 2 * (1.0 + x)*(lam + x) / (8.0*x);
                // p = (2.0*magr1*magr2*(1.0 + x)*sin(dnu*0.5) ^ 2) ^ 2 / (s*(1 + lam) ^ 2 * (lam + x));  % loechler, not right ?
                p = (2.0 * magr1 * magr2 * y * y * Math.Pow(1.0 + x, 2) * Math.Pow(Math.Sin(dnu * 0.5), 2)) /
                        (m * s * Math.Pow(1.0 + lam, 2));  // thompson
                ecc = Math.Sqrt((eps * eps + 4.0 * magr2 / magr1 * Math.Pow(Math.Sin(dnu * 0.5), 2) *
                        Math.Pow((L - x) / (L + x), 2)) / (eps * eps + 4.0 * magr2 / magr1 * Math.Pow(Math.Sin(dnu * 0.5), 2)));
                LambHodograph(p, ecc, dnu, xferTime); // results put in globals v1t and v2t

                // Battin solution to orbital parameters(and velocities)
                // thompson 2011, loechler 1988
                if (dnu > Math.PI)
                    lam = -Math.Sqrt((s - chord) / s);
                else
                    lam = Math.Sqrt((s - chord) / s);

                // loechler pg 21 seems correct!
                double3 v1dvl = 1.0 / (lam * (1.0 + lam)) * Math.Sqrt(mu * (1.0 + x) / (2.0 * s * s * s * (L + x))) * ((r2 - r1)
                                    + s * Math.Pow(1.0 + lam, 2) * (L + x) / (magr1 * (1.0 + x)) * r1);
                // added v2
                double3 v2dvl = 1.0 / (lam * (1.0 + lam)) * Math.Sqrt(mu * (1.0 + x) / (2.0 * s * s * s * (L + x))) * ((r2 - r1)
                                    - s * Math.Pow(1.0 + lam, 2) * (L + x) / (magr2 * (1.0 + x)) * r2);

                // Seems these are the answer. Not at all sure what the point of the Hodograph calls was...
                // but for FRGeneric the Hodograph answer seems ok. In that case lam = 0 and v1vdl is NaN. Wha??
                // Add code to use hodograph if lam=0
                //Debug.LogFormat("lam={0}", lam);
                // If the transfer is close to 180 degrees then use the LambHodograph value for the xfer velocity
                if (Math.Abs(lam) > 1E-3) {
                    v1t = v1dvl;
                    v2t = v2dvl;
                } else {
                    // we are close to 180 degrees so use the hodograph value
                    if (reverse) {
                        v1t = -v1t;
                        v2t = -v2t;
                    }
                }

                //fprintf(1, 'loe v1t %16.8f %16.8f %16.8f %16.8f\n', v1dvl, mag(v1dvl));
                //fprintf(1, 'loe v2t %16.8f %16.8f %16.8f %16.8f\n', v2dvl, mag(v2dvl));
                // Debug.LogFormat("v1dvl={0} v2dvl={1}", v1dvl, v2dvl);
            } else {
                error = 2;
            }

            // Determine maneuvers needed for ship (fromOrbit)
            if (error == 0) {
                if (checkPlanetRadius) {
                    bool hit = Orbital.CheckHitPlanet(planetRadius, mu, r1, v1t,
                                                            r2, v2t,
                                                            nrev: 0);
                    if (hit) {
                        error = IMPACT;
                        // allow maneuver to proceed, might want to display the orbit that results
                    }
                }
                maneuvers = new List<GEManeuver>();
                //double3 centerVel = GravityEngine.instance.GetVelocityDoubleV3(centerBody);
                //double3 fromVelRelative = GravityEngine.instance.GetVelocityDoubleV3(fromBody) - centerVel;
                // Departure
                GEManeuver departure = new GEManeuver {
                    type = ManeuverType.SET_VELOCITY,
                    info = ManeuverInfo.LBAT_1,
                    //departure.nbody = fromBody;
                    //departure.physPosition = r1 + center3d;
                    velocityParam = v1t,
                    t_relative = 0,
                    // relative info
                    r_relative = r1,
                    v_relative = v1t,
                    hasRelativeRV = true
                };
                //departure.relativeTo = centerBody;
                //departure.dVvector = new double3(v1_vec) - fromVelRelative;
                //departure.dV = (float)departure.dVvector.magnitude;

                maneuvers.Add(departure);
                // deltaV = math.length(departure.velocityParam);

                // Arrival (will not be required if intercept)
                // setv is wrt to centerVel
                if ((toOrbit != null) && (toOrbit.a != 0) && (mode != LambertMode.INTERCEPT)) {
                    GEManeuver arrival = new GEManeuver {
                        info = ManeuverInfo.LBAT_2,
                        //arrival.nbody = fromBody;
                        //arrival.physPosition = r2 + center3d; // wrong if in orbit, centerPos will have moved on
                        t_relative = departure.t_relative + xferTime,
                        type = ManeuverType.SET_VELOCITY
                    };
                    if (mode == LambertMode.RENDEZVOUS) {
                        arrival.velocityParam = targetVelocity;
                    } else {
                        // this is a relative vel.
                        arrival.velocityParam = v2; // toOrbit.GetPhysicsVelocityForEllipse(toOrbit.phase);
                    }
                    // prop initial maneuver to final time to get velocity at that point
                    KeplerPropagator.RVT kProp = new KeplerPropagator.RVT(r1, departure.velocityParam, 0, mu);
                    (int status, double3 r_prop, double3 v_prop) = KeplerPropagator.RVforTime(kProp, xferTime);
                    arrival.dV = v2 - v_prop;
                    arrival.r_relative = r_prop;
                    arrival.v_relative = v2;
                    arrival.hasRelativeRV = true;
                    maneuvers.Add(arrival);
                    //deltaV += arrival.dV;
                } else {
                    // this is an intercept. Add a dummy maneuver
                    GEManeuver intercept = new GEManeuver {
                        info = ManeuverInfo.INTERCEPT,
                        t_relative = departure.t_relative + xferTime,
                        type = ManeuverType.APPLY_DV,
                        velocityParam = double3.zero
                    };
                    maneuvers.Add(intercept);
                }

            }

            return error;
        }  // lambertbattin




        public int ComputeXfer(double xferTime,
                                LambertMode mode,
                                bool requirePrograde = true,
                                bool df = false,
                                int nrev = 0)

        {
            // confirm there is a target NBody (i.e. correct constructor was used)
            if (toOrbit != null && toOrbit.a == 0) {
                Debug.LogError("Cannot compute phasing, no target orbit was specified.");
                return 5;
            }
            if (mode != LambertMode.TO_R2) {
                // Determine where target will be at time=dtsec
                KeplerPropagator.RVT targetRVT = new KeplerPropagator.RVT();
                (targetRVT.r0, targetRVT.v0) = Orbital.COEtoRVRelative(toOrbit);
                targetRVT.t0 = 0.0;
                targetRVT.mu = mu;
                targetRVT.Init();
                int status = 0;
                (status, r2, v2) = KeplerPropagator.RVforTime(targetRVT, xferTime);
                targetVelocity = v2;
                if (status != 0) {
                    Debug.LogError("Could not prop target " + KeplerPropagator.errorText[status]);
                    return 5;
                }
            }
            bool reverse = !requirePrograde;
            Debug.LogFormat("ComputeXfer: requirePrograde={0} reverse={1}", requirePrograde, reverse);
            int xferStatus = ComputeXferInternal(reverse, df, nrev, xferTime, mode);
            if (xferStatus != 0 && xferStatus != IMPACT)
                return xferStatus;
            if (requirePrograde && !Orbital.VelocityIsPrograde(r1, v1, maneuvers[0].velocityParam)) {
                // pointing wrong way so do again with reverse
                reverse = !reverse;
                xferStatus = ComputeXferInternal(reverse, df, nrev, xferTime, mode);
            }
            if (!requirePrograde && Orbital.VelocityIsPrograde(r1, v1, maneuvers[0].velocityParam)) {
                // pointing wrong way so do again with reverse
                reverse = !reverse;
                xferStatus = ComputeXferInternal(reverse, df, nrev, xferTime, mode);
            }
            return xferStatus;
        }

        /// <summary>
        /// Set the radius of the planet around which transfer is to be computed. This is used when calls to 
        /// ComputeTransferWithPhasing(). In the event that the requested trajectory hits the planet 
        /// </summary>
        /// <param name="radius"></param>
        public void HitPlanetRadius(double radius)
        {
            planetRadius = radius;
            checkPlanetRadius = true;
        }

        public void ClearPlanetRadius()
        {
            checkPlanetRadius = false;
        }
    }
}
