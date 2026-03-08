using System.Collections.Generic;
using Unity.Mathematics;

// Vallado has made some changes to the Lambert Universal method.
// The base code comes from the microcosm website pulled in Sept. 2024

// Mods:
// 1. Use factorial array instead of math.factorial
// 2. Use GBUnits.earthRadiusKm
// 3. Use double3 for vectors
//
// Q: What is kbi input param for?
// Q: How does the L/H orbital motion and orbital energy work?
// - says DE only affects nrev >= 1 upper/lower bounds. Ignore?
// Q: dtwait?? Why??
// Q: Is v1 for n=0 same as for n=1, 2 etc??

namespace GravityEngine2 {
    public class Lambert {

        /*------------------------------------------------------------------------------
        *
        *                           procedure lambertminT
        *
        *  this procedure solves lambert's problem and finds the miniumum time for 
        *  multi-revolution cases.
        *
        *  author        : david vallado           davallado@gmail.com   22 mar 2018
        *
        *  inputs          description                        range / units
        *    r1          - ijk position vector 1                km
        *    r2          - ijk position vector 2                km
        *    dm          - direction of motion                  'L', 'S'
        *    de          - orbital energy                       'L', 'H'
        *    nrev        - number of revs to complete           0, 1, 2, 3,  
        *
        *  outputs       :
        *    tmin        - minimum time of flight               sec
        *    tminp       - minimum parabolic tof                sec
        *    tminenergy  - minimum energy tof                   sec
        *
        *  locals        :
        *    i           - index
        *    loops       -
        *    cosdeltanu  -
        *    sindeltanu  -
        *    dnu         -
        *    chord       -
        *    s           -
        *
        *  coupling      :
        *    mag         - magnitude of a vector
        *    dot         - dot product
        *
        *  references    :
        *    vallado       2013, 494, Alg 59, ex 7-5
        *    prussing      JAS 2000
        -----------------------------------------------------------------------------*/

        /// <summary>
        /// Calculate the reference time of flight for a Lambert transfer.
        /// 
        /// By default, this is a prograde transfer but can be optionally 
        /// over-ridden to force a retrograde transfer.
        /// </summary>
        /// <param name="mu">gravitational parameter of central body</param>
        /// <param name="r1">position vector at departure</param>
        /// <param name="r2">position vector at arrival</param>
        /// <param name="prograde">if false, force retrograde transfer</param>
        /// <returns>tuple of minimum time of flight, minimum parabolic time of flight, and minimum energy time of flight</returns>
        /// The purpose of tmin is not clear to me. Current code ignores this. 
        public static (double tmin, double tminp, double tminenergy) MinTimesPrograde(double mu, double3 r1, double3 r2, double3 v1, bool prograde = true)
        {
            double angle = GravityMath.AngleRadiansPrograde(r1, v1, r2);
            MotionDirection motionDir = MotionDirection.SHORT;
            if (angle > math.PI) {
                motionDir = MotionDirection.LONG;
            }
            if (!prograde) {
                motionDir = (motionDir == MotionDirection.SHORT) ? MotionDirection.LONG : MotionDirection.SHORT;
            }
            return MinTimes(mu, r1, r2, motionDir, nrev: 0);
        }

        /// <summary>
        /// Calculate the minimum time of flight for a Lambert transfer.
        /// </summary>
        /// <param name="mu">gravitational parameter of central body</param>
        /// <param name="r1">position vector at departure</param>
        /// <param name="r2">position vector at arrival</param>
        /// <param name="dm">direction of motion</param>
        /// <param name="nrev">number of revolutions</param>
        /// <returns></returns>
        public static (double tmin, double tminp, double tminenergy) MinTimes
        (
            double mu, double3 r1, double3 r2, MotionDirection dm, int nrev
        )
        {
            double a, s, chord, magr1, magr2, cosdeltanu;
            double alpha, alp, beta, fa, fadot, xi, eta, del, an;
            int i;
            double tmin, tminp, tminenergy;

            magr1 = math.length(r1);
            magr2 = math.length(r2);
            cosdeltanu = math.dot(r1, r2) / (magr1 * magr2);
            // make sure it's not more than 1.0
            if (math.abs(cosdeltanu) > 1.0)
                cosdeltanu = 1.0 * math.sign(cosdeltanu);

            // these are the same
            chord = math.sqrt(magr1 * magr1 + magr2 * magr2 - 2.0 * magr1 * magr2 * cosdeltanu);
            //chord = MathTimeLibr.mag(r2 - r1);

            s = (magr1 + magr2 + chord) * 0.5;

            xi = 0.0;
            eta = 0.0;

            // could do this just for nrev cases, but you can also find these for any nrev if (nrev > 0)
            // ------------- calc tmin parabolic tof to see if the orbit is possible
            if (dm == MotionDirection.SHORT)
                tminp = (1.0 / 3.0) * math.sqrt(2.0 / mu) * (math.pow(s, 1.5) - math.pow(s - chord, 1.5));
            else
                tminp = (1.0 / 3.0) * math.sqrt(2.0 / mu) * (math.pow(s, 1.5) + math.pow(s - chord, 1.5));

            // ------------- this is the min energy ellipse tof
            double amin = 0.5 * s;
            beta = 2.0 * math.asin(math.sqrt((s - chord) / s));
            if (dm == MotionDirection.SHORT)
                tminenergy = math.pow(amin, 1.5) * ((2.0 * nrev + 1.0) * math.PI - beta + math.sin(beta)) / math.sqrt(mu);
            else
                tminenergy = math.pow(amin, 1.5) * ((2.0 * nrev + 1.0) * math.PI + beta - math.sin(beta)) / math.sqrt(mu);

            // -------------- calc min tof ellipse (prussing 1992 aas, 2000 jas)
            an = 1.001 * amin;
            fa = 10.0;
            i = 1;
            while (math.abs(fa) > 0.00001 && i <= 20) {
                a = an;
                alp = 1.0 / a;
                double temp = math.sqrt(0.5 * s * alp);
                if (math.abs(temp) > 1.0)
                    temp = 1.0 * math.sign(temp);
                alpha = 2.0 * math.asin(temp);
                // now account for direct or retrograde
                temp = math.sqrt(0.5 * (s - chord) * alp);
                if (math.abs(temp) > 1.0)
                    temp = 1.0 * math.sign(temp);
                if (dm == MotionDirection.LONG)  // old de l
                    beta = 2.0 * math.asin(temp);
                else
                    beta = -2.0 * math.asin(temp);  // fix quadrant
                xi = alpha - beta;
                eta = math.sin(alpha) - math.sin(beta);
                fa = (6.0 * nrev * math.PI + 3.0 * xi - eta) * (math.sin(xi) + eta) - 8.0 * (1.0 - math.cos(xi));

                fadot = ((6.0 * nrev * math.PI + 3.0 * xi - eta) * (math.cos(xi) + math.cos(alpha)) +
                         (3.0 - math.cos(alpha)) * (math.sin(xi) + eta) - 8.0 * math.sin(xi)) * (-alp * math.tan(0.5 * alpha))
                         + ((6.0 * nrev * math.PI + 3.0 * xi - eta) * (-math.cos(xi) - math.cos(beta)) +
                         (-3.0 + math.cos(beta)) * (math.sin(xi) + eta) + 8.0 * math.sin(xi)) * (-alp * math.tan(0.5 * beta));
                del = fa / fadot;
                an = a - del;
                i = i + 1;
            }
            // could update beta one last time with alpha too????
            if (dm == MotionDirection.SHORT)
                tmin = math.pow(an, 1.5) * (2.0 * math.PI * nrev + xi - eta) / math.sqrt(mu);
            else
                tmin = math.pow(an, 1.5) * (2.0 * math.PI * nrev + xi + eta) / math.sqrt(mu);

            return (tmin, tminp, tminenergy);
        }  // lambertminT



        /*------------------------------------------------------------------------------
        *
        *                           procedure lambertuniv
        *
        *  this procedure solves the lambert problem for orbit determination and returns
        *    the velocity vectors at each of two given position vectors.  the solution
        *    uses universal variables for calculation and a bissection technique for
        *    updating psi.
        *
        *  algorithm     : setting the initial bounds:
        *                  using -8pi and 4pi2 will allow single rev solutions
        *                  using -4pi2 and 8pi2 will allow multi-rev solutions
        *                  the farther apart the initial guess, the more iterations
        *                    because of the iteration
        *                  inner loop is for special cases. must be sure to exit both!
        *
        *  author        : david vallado                 davallado@gmail.com   22 jun 2002
        *
        *  inputs          description                          range / units
        *    r1          - ijk position vector 1                km
        *    r2          - ijk position vector 2                km
        *    v1          - ijk velocity vector 1 if avail       km/s
        *    dm          - direction of motion                  'L', 'S'
        *    de          - orbital energy                       'L', 'H'
        *                  only affects nrev >= 1 upper/lower bounds
        *    dtsec       - time between r1 and r2               sec
        *    dtwait      - time to wait before starting         sec
        *    nrev        - number of revs to complete           0, 1, 2, 3,  
        *    kbi         - psi value for min                     
        *    altpad      - altitude pad for hitearth calc       km
        *    show        - control output don't output for speed      'y', 'n'
        *
        *  outputs       :
        *    v1t         - ijk transfer velocity vector         km/s
        *    v2t         - ijk transfer velocity vector         km/s
        *    hitearth    - flag if hit or not                   'y', 'n'
        *    error       - error flag                           1, 2, 3,   use numbers since c++ is so horrible at strings
        *
        *  locals        :
        *    vara        - variable of the iteration,
        *                  not the semi or axis!
        *    y           - area between position vectors
        *    upper       - upper bound for z
        *    lower       - lower bound for z
        *    cosdeltanu  - cosine of true anomaly change        rad
        *    f           - f expression
        *    g           - g expression
        *    gdot        - g dot expression
        *    xold        - old universal variable x
        *    xoldcubed   - xold cubed
        *    zold        - old value of z
        *    znew        - new value of z
        *    c2new       - c2(z) function
        *    c3new       - c3(z) function
        *    timenew     - new time                             sec
        *    small       - tolerance for roundoff errors
        *    i, j        - index
        *
        *  coupling
        *    mag         - magnitude of a vector
        *    dot         - dot product of two vectors
        *    findc2c3    - find c2 and c3 functions
        *
        *  references    :
        *    vallado       2013, 492, alg 58, ex 7-5
        -----------------------------------------------------------------------------*/

        public enum MotionDirection { SHORT, LONG };
        public enum EnergyType { LOW, HIGH };

        public enum Status { OK, G_NOT_CONVERGED, HIT_PLANET, ERROR_OUT };

        public struct LambertOutput {
            public double3 v1t; // velocity of transfer path at start of transfer
            public double3 v2t; // velocity of transfer path at end of transfer

            // record the to/from state (may not know velocities)
            public double3 r1, v1; // state at start of transfer
            public double3 r2, v2; // state at end of transfer

            public double t0;

            // time of flight
            public double t; // time of flight of the transfer
            public Status status;

            public LambertOutput(double3 v1t, double3 v2t, double t, Status status) : this()
            {
                this.v1t = v1t;
                this.v2t = v2t;
                this.t = t;
                this.status = status;
                // code around the Lambert transfer may use these fields
                r1 = double3.zero;
                r2 = double3.zero;
                v1 = double3.zero;
                v2 = double3.zero;
                t0 = 0.0;
            }

            public List<GEManeuver> Maneuvers(int centerId, GEPhysicsCore.Propagator prop, bool intercept = false)
            {
                List<GEManeuver> maneuvers = new List<GEManeuver>();
                // If non-Kepler rail prop, then the transfer is Kepler
                GEPhysicsCore.Propagator xferProp = prop;
                if (prop == GEPhysicsCore.Propagator.PKEPLER || prop == GEPhysicsCore.Propagator.SGP4_RAILS) {
                    xferProp = GEPhysicsCore.Propagator.KEPLER;
                }
                GEManeuver m1 = new GEManeuver(0.0, v1t, ManeuverType.SET_VELOCITY);
                m1.velocityParam = v1t;
                m1.hasRelativeRV = true;
                m1.r_relative = r1;
                m1.v_relative = v1t;
                m1.centerId = centerId;
                m1.info = ManeuverInfo.LAM_1;
                m1.prop = xferProp;
                m1.dV = v1t - v1;
                maneuvers.Add(m1);
                GEManeuver m2 = null;
                if (intercept) {
                    m2 = new GEManeuver(t, v2, ManeuverType.APPLY_DV);
                    m2.velocityParam = double3.zero;
                    m2.hasRelativeRV = true;
                    m2.r_relative = r2;
                    m2.v_relative = v2t;
                    m2.centerId = centerId;
                    m2.info = ManeuverInfo.LAM_2;
                    m2.prop = prop;
                    m2.t_relative = t;
                    m2.dV = 0;
                } else {
                    m2 = new GEManeuver(t, v2, ManeuverType.SET_VELOCITY);
                    m2.hasRelativeRV = true;
                    m2.r_relative = r2;
                    m2.v_relative = v2;
                    m2.centerId = centerId;
                    m2.info = ManeuverInfo.LAM_2;
                    m2.prop = prop;
                    m2.t_relative = t;
                    m2.dV = v2 - v2t;
                }
                //UnityEngine.Debug.Log($"m2={m2.LogString()} v2t={v2t}");
                maneuvers.Add(m2);
                return maneuvers;
            }
        }

        /// <summary>
        /// Solve the Lambert problem for a prograde transfer and given timeXfer to a target in motion.
        /// The position and velocity vectors of the target are computed at the time of arrival.
        /// </summary>
        /// <param name="mu">gravitational parameter of central body</param>
        /// <param name="r1">position vector of interceptor at departure</param>
        /// <param name="r2">position vector of target at departure</param>
        /// <param name="v1">velocity vector of interceptor at departure</param>
        /// <param name="v2">velocity vector of target at departure</param>
        /// <param name="timeXfer">time of flight</param>
        /// <param name="radius">radius of central body</param>
        /// <param name="prograde">if false, force retrograde transfer</param>
        /// <returns></returns>
        public static LambertOutput TransferProgradeToTarget(double mu, double3 r1, double3 r2, double3 v1, double3 v2, double timeXfer,
                                                    double radius = 0.0, bool prograde = true)
        {
            // Evolve the target to the position it will occupy at the time of arrival
            KeplerPropagator.RVT rvt2 = new KeplerPropagator.RVT(r2, v2, 0.0, mu);
            (int s, double3 r2_final, double3 v2_final) = KeplerPropagator.RVforTime(rvt2, timeXfer);
            double angle = GravityMath.AngleRadiansPrograde(r1, v1, r2_final);
            MotionDirection motionDir = MotionDirection.SHORT;
            if (angle > math.PI) {
                motionDir = MotionDirection.LONG;
            }
            if (!prograde) {
                motionDir = (motionDir == MotionDirection.SHORT) ? MotionDirection.LONG : MotionDirection.SHORT;
            }
            LambertOutput lamOutput = Universal(mu, r1, r2_final, v1, motionDir, EnergyType.HIGH, nrev: 0, timeXfer, 0.0, radius);
            lamOutput.r2 = r2_final;
            lamOutput.v2 = v2_final;
            return lamOutput;
        }


        /// <summary>
        /// Solve the Lambert problem for a prograde transfer and given timeXfer to a specific fixed
        /// destination point.
        /// 
        /// The default is prograde but this can be optionally over-ridden to force a retrograde transfer
        /// direction. 
        /// 
        /// </summary>
        /// <param name="mu">gravitational parameter of central body</param>
        /// <param name="r1">position vector at departure</param>
        /// <param name="r2">position vector at arrival</param>
        /// <param name="v1">velocity vector at departure</param>
        /// <param name="timeXfer">time of flight</param>
        /// <param name="radius">radius of central body</param>
        /// <param name="prograde">if false, force retrograde transfer</param>
        /// <returns></returns>
        public static LambertOutput TransferProgradeToPoint(double mu,
                                                    double3 r1,
                                                    double3 r2,
                                                    double3 v1,
                                                    double timeXfer,
                                                    double radius = 0.0,
                                                    bool prograde = true)
        {
            double angle = GravityMath.AngleRadiansPrograde(r1, v1, r2);
            MotionDirection motionDir = MotionDirection.SHORT;
            if (angle > math.PI) {
                motionDir = MotionDirection.LONG;
            }
            if (!prograde) {
                motionDir = (motionDir == MotionDirection.SHORT) ? MotionDirection.LONG : MotionDirection.SHORT;
            }
            return Universal(mu, r1, r2, v1, motionDir, EnergyType.HIGH, nrev: 0, timeXfer, 0.0, radius);
        }

        public static LambertOutput Universal
               (
                double mu,
               double3 r1,
               double3 r2,
               double3 v1,
               MotionDirection motionDir,
               EnergyType energyType,
               int nrev,
               double t_xfer,
               double kbi = 0,      // only used if nrev != 0
               double radius = 0.0)
        {
            const double small = 0.0000001;
            const int numiter = 40;

            int loops, ynegktr;
            double vara, y, upper, lower, cosdeltanu, f, g, gdot, xold, xoldcubed, magr1, magr2,
                psiold, psinew, psilast, c2new, c3new, dtnew, dtold, c2dot, c3dot, dtdpsi, psiold2;
            string estr;
            Status status = Status.OK;
            //int error;

            // needed since assignments aren't at root level in procedure
            double3 v1t = new double3(0.0, 0.0, 0.0);
            double3 v2t = new double3(0.0, 0.0, 0.0);

            /* --------------------  initialize values   -------------------- */
            estr = "";  // determine various cases
            bool hitearth = false;
            //error = 0;
            psilast = 0.0;
            psinew = 0.0;
            dtold = 0.0;
            loops = 0;
            y = 0.0;

            magr1 = math.length(r1);
            magr2 = math.length(r2);

            cosdeltanu = math.dot(r1, r2) / (magr1 * magr2);
            if (motionDir == MotionDirection.LONG)  //~works de == 'h'   
                vara = -math.sqrt(magr1 * magr2 * (1.0 + cosdeltanu));
            else
                vara = math.sqrt(magr1 * magr2 * (1.0 + cosdeltanu));

            /* -------- set up initial bounds for the bisection ------------ */
            if (nrev == 0) {
                lower = -16.0 * math.PI * math.PI; // could be negative infinity for all cases, allow hyperbolic and parabolic solutions
                upper = 4.0 * math.PI * math.PI;
            } else {
                lower = 4.0 * nrev * nrev * math.PI * math.PI;
                upper = 4.0 * (nrev + 1.0) * (nrev + 1.0) * math.PI * math.PI;
                //                if (((df == 'r') && (dm == 'l')) || ((df == 'd') && (dm == 'l')))
                if (energyType == EnergyType.HIGH)   // high way is always the 1st half
                    upper = kbi;
                else
                    lower = kbi;
            }

            /* ----------------  form initial guesses   --------------------- */
            psinew = 0.0;
            xold = 0.0;
            if (nrev == 0) {
                psiold = (math.log(t_xfer) - 9.61202327) / 0.10918231;
                if (psiold > upper)
                    psiold = upper - math.PI;
            } else
                psiold = lower + (upper - lower) * 0.5;

            FindC2C3(psiold, out c2new, out c3new);

            double oosqrtmu = 1.0 / math.sqrt(mu);

            // find initial dtold from psiold
            if (math.abs(c2new) > small)
                y = magr1 + magr2 - (vara * (1.0 - psiold * c3new) / math.sqrt(c2new));
            else
                y = magr1 + magr2;
            if (math.abs(c2new) > small)
                xold = math.sqrt(y / c2new);
            else
                xold = 0.0;
            xoldcubed = xold * xold * xold;
            dtold = (xoldcubed * c3new + vara * math.sqrt(y)) * oosqrtmu;

            // -----------  determine if the orbit is possible at all ------------ 
            if (math.abs(vara) > 0.2)  // small
            {
                loops = 0;
                ynegktr = 1; // y neg ktr
                dtnew = -10.0;
                while ((math.abs(dtnew - t_xfer) >= small) && (loops < numiter) && (ynegktr <= 10)) {
                    if (math.abs(c2new) > small)
                        y = magr1 + magr2 - (vara * (1.0 - psiold * c3new) / math.sqrt(c2new));
                    else
                        y = magr1 + magr2;

                    // ------- check for negative values of y ------- 
                    if ((vara > 0.0) && (y < 0.0)) {
                        ynegktr = 1;
                        while ((y < 0.0) && (ynegktr < 10)) {
                            psinew = 0.8 * (1.0 / c3new) * (1.0 - (magr1 + magr2) * math.sqrt(c2new) / vara);

                            /* ------ find c2 and c3 functions ------ */
                            FindC2C3(psinew, out c2new, out c3new);
                            psiold = psinew;
                            lower = psiold;
                            if (math.abs(c2new) > small)
                                y = magr1 + magr2 - (vara * (1.0 - psiold * c3new) / math.sqrt(c2new));
                            else
                                y = magr1 + magr2;

                            ynegktr++;
                        }
                    }

                    loops = loops + 1;

                    if (ynegktr < 10) {
                        if (math.abs(c2new) > small)
                            xold = math.sqrt(y / c2new);
                        else
                            xold = 0.0;
                        xoldcubed = xold * xold * xold;
                        dtnew = (xoldcubed * c3new + vara * math.sqrt(y)) * oosqrtmu;

                        // try newton rhapson iteration to update psinew
                        if (math.abs(psiold) > 1e-5) {
                            c2dot = 0.5 / psiold * (1.0 - psiold * c3new - 2.0 * c2new);
                            c3dot = 0.5 / psiold * (c2new - 3.0 * c3new);
                        } else {
                            psiold2 = psiold * psiold;
                            c2dot = -1.0 / factorial[4] + 2.0 * psiold / factorial[6] - 3.0 * psiold2 / factorial[8] +
                                     4.0 * psiold2 * psiold / factorial[10] - 5.0 * psiold2 * psiold2 / factorial[12];
                            c3dot = -1.0 / factorial[5] + 2.0 * psiold / factorial[7] - 3.0 * psiold2 / factorial[9] +
                                     4.0 * psiold2 * psiold / factorial[11] - 5.0 * psiold2 * psiold2 / factorial[13];
                        }
                        dtdpsi = (xoldcubed * (c3dot - 3.0 * c3new * c2dot / (2.0 * c2new)) + vara / 8.0 * (3.0 * c3new * math.sqrt(y) / c2new + vara / xold)) * oosqrtmu;
                        psinew = psiold - (dtnew - t_xfer) / dtdpsi;
                        double psitmp = psinew;

                        // check if newton guess for psi is outside bounds(too steep a slope), then use bisection
                        if (psinew > upper || psinew < lower) {
                            // --------readjust upper and lower bounds-------
                            // special check for 0 rev cases 
                            if (energyType == EnergyType.LOW || (nrev == 0)) {
                                if (dtold < t_xfer)
                                    lower = psiold;
                                else
                                    upper = psiold;
                            } else {
                                if (dtold < t_xfer)
                                    upper = psiold;
                                else
                                    lower = psiold;
                            }

                            psinew = (upper + lower) * 0.5;
                            psilast = psinew;
                            estr = estr + ynegktr.ToString() + " biss ";
                        }

                        // -------------- find c2 and c3 functions ---------- 
                        FindC2C3(psinew, out c2new, out c3new);
                        psilast = psiold;  // keep previous iteration
                        psiold = psinew;
                        dtold = dtnew;

                        // ---- make sure the first guess isn't too close --- 
                        if ((math.abs(dtnew - t_xfer) < small) && (loops == 1))
                            dtnew = t_xfer - 1.0;
                    }  // if ynegktr < 10

                    ynegktr = 1;
                }  // while

                if ((loops >= numiter) || (ynegktr >= 10)) {
                    status = Status.G_NOT_CONVERGED;

                    if (ynegktr >= 10) {
                        status = Status.HIT_PLANET;
                    }
                    // UnityEngine.Debug.LogFormat("G_NOT CONVERGED using Battin");
                    return Battin(mu, r1, r2, v1, motionDir, energyType, nrev, t_xfer, radius);
                } else {
                    // ---- use f and g series to find velocity vectors ----- 
                    f = 1.0 - y / magr1;
                    gdot = 1.0 - y / magr2;
                    g = 1.0 / (vara * math.sqrt(y / mu)); // 1 over g
                    //	fdot = Math.Sqrt(y) * (-magr2 - magr1 + y) / (magr2 * magr1 * vara);
                    for (int i = 0; i < 3; i++) {
                        v1t[i] = ((r2[i] - f * r1[i]) * g);
                        v2t[i] = ((gdot * r2[i] - r1[i]) * g);
                    }
                    hitearth = CheckHitPlanet(mu, radius, r1, v1t, r2, v2t, nrev);
                }
            } else {
                // use battin and hodograph
                return Battin(mu, r1, r2, v1, motionDir, energyType, nrev, t_xfer, radius);
            }

            double dnu;
            if (motionDir == MotionDirection.SHORT)  // find dnu of transfer
                dnu = math.acos(cosdeltanu) * 180.0 / math.PI;
            else {
                dnu = -math.acos(cosdeltanu) * 180.0 / math.PI;
                if (dnu < 0.0)
                    dnu = dnu + 360.0;
            }
            if (hitearth) {
                status = Status.HIT_PLANET;
            }
            LambertOutput output = new LambertOutput(v1t, v2t, t_xfer, status);
            output.r1 = r1;
            output.r2 = r2;
            output.v1 = v1;
            return output;
        }  //  lambertuniv

        // only need value up to 13 in the Lambert universal, so make it a look up
        private static double[] factorial = { 1.0, 1.0, 2.0, 6.0, 24.0, 120.0, 720.0, 5040.0, 40320.0,
            362880.0, 3628800.0, 3.991680E7, 4.790016E8, 6227021E9};


        /* -------------------------------------------------------------------------- 
        *                           function lambhodograph
        *
        * this function accomplishes 180 deg transfer(and 360 deg) for lambert problem.
        *
        *  author        : david vallado           davallado@gmail.com  22 may 2017
        *
        *  inputs          description                            range / units
        *    r1    - ijk position vector 1                            km
        *    r2    - ijk position vector 2                            km
        *    v1    - intiial ijk velocity vector 1                    km/s
        *    p     - semiparamater of transfer orbit                  km
        *    ecc   - eccentricity of transfer orbit                   km
        *    dnu   - true anomaly delta for transfer orbit            rad
        *    dtsec - time between r1 and r2                           s
        *    dnu - true anomaly change                                rad
        *
        *  outputs       :
        *    v1t - ijk transfer velocity vector                       km/s
        *    v2t - ijk transfer velocity vector                       km/s
        * 
        *  references :
        *    Thompson JGCD 2013 v34 n6 1925
        *    Thompson AAS GNC 2018
        ----------------------------------------------------------------------------- */

        private static void LambHodograph
        (
            double mu,
        double3 r1, double3 r2, double3 v1, double p, double ecc, double dnu, double dtsec, out double3 v1t, out double3 v2t
        )
        {
            double eps, magr1, magr2, a, b, x1, x2, y2a, y2b, ptx, oomagr1, oomagr2, oop;
            double3 nvec = new double3(0, 0, 0);
            double3 rcrv = new double3(0, 0, 0);
            double3 rcrr = new double3(0, 0, 0);
            int i;
            // needed since assignments aren't at root level in procedure
            v1t = new double3(0.0, 0.0, 0.0);
            v2t = new double3(0.0, 0.0, 0.0);
            oop = 1.0 / p;

            magr1 = math.length(r1);
            oomagr1 = 1.0 / magr1;
            magr2 = math.length(r2);
            oomagr2 = 1.0 / magr2;
            eps = 0.001 / magr2;  // 1e-14

            a = mu * (1.0 * oomagr1 - 1.0 * oop);  // not the semi - major axis
            double mue = mu * ecc * oop;
            b = mue * mue - a * a;
            if (b <= 0.0)
                x1 = 0.0;
            else
                x1 = -math.sqrt(b);

            // 180 deg, and multiple 180 deg transfers
            if (math.abs(math.sin(dnu)) < eps) {
                double3 rcvr = math.cross(r1, v1);
                for (i = 0; i < 3; i++)
                    nvec[i] = rcrv[i] / math.length(rcrv);
                if (ecc < 1.0) {
                    ptx = (2.0 * math.PI) * math.sqrt(p * p * p / (mu * math.pow(1.0 - ecc * ecc, 3)));
                    if (dtsec % ptx > ptx * 0.5)  // mod
                        x1 = -x1;
                }
            } else {
                // this appears to be the more common option
                y2a = mu * oop - x1 * math.sin(dnu) + a * math.cos(dnu);
                y2b = mu * oop + x1 * math.sin(dnu) + a * math.cos(dnu);
                if (math.abs(mu * oomagr2 - y2b) < math.abs(mu * oomagr2 - y2a))
                    x1 = -x1;

                // depending on the cross product, this will be normal or in plane,
                // or could even be a fan
                rcrr = math.cross(r1, r2);
                for (i = 0; i < 3; i++)
                    nvec[i] = rcrr[i] / math.length(rcrr); // if this is r1, v1, the transfer is coplanar!
                if ((dnu % (2.0 * math.PI)) > math.PI) {
                    for (i = 0; i < 3; i++)
                        nvec[i] = -nvec[i];
                }
            }

            rcrv = math.cross(nvec, r1);
            rcrr = math.cross(nvec, r2);
            x2 = x1 * math.cos(dnu) + a * math.sin(dnu);
            for (i = 0; i < 3; i++) {
                v1t[i] = (math.sqrt(mu * p) * oomagr1) * ((x1 / mu) * r1[i] + rcrv[i] * oomagr1);
                v2t[i] = (math.sqrt(mu * p) * oomagr2) * ((x2 / mu) * r2[i] + rcrr[i] * oomagr2);
            }
        }  // lambhodograph



        private static double KBattin
        (
        double v
        )
        {
            double sum1, delold, termold, del, term;
            int i;
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

            // ----- process forwards ---- 
            delold = 1.0;
            termold = d[0];
            sum1 = termold;
            i = 1;
            while ((i <= 20) && (math.abs(termold) > 0.000000001)) {
                del = 1.0 / (1.0 + d[i] * v * delold);
                term = termold * (del - 1.0);
                sum1 = sum1 + term;

                i = i + 1;
                delold = del;
                termold = term;
            }
            return (sum1);

            //int ktr = 20;
            //double sum2 = 0.0;
            //double term2 = 1.0 + d[ktr] * v;
            //for (i = 1; i <= ktr - 1; i++)
            //{
            //    sum2 = d[ktr - i] * v / term2;
            //    term2 = 1.0 + sum2;
            //}
            //return (d[0] / term2);
        }  // double kbattin


        /* -------------------------------------------------------------------------- */

        private static double SeeBattin(double v)
        {
            double eta, sqrtopv;
            double delold, termold, sum1, term, del;
            int i;
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

            sqrtopv = math.sqrt(1.0 + v);
            eta = v / math.pow(1.0 + sqrtopv, 2);

            // ---- process forwards ---- 
            delold = 1.0;
            termold = c[0];
            sum1 = termold;
            i = 1;
            while ((i <= 20) && (math.abs(termold) > 0.000000001)) {
                del = 1.0 / (1.0 + c[i] * eta * delold);
                term = termold * (del - 1.0);
                sum1 = sum1 + term;

                i = i + 1;
                delold = del;
                termold = term;
            }
            return (1.0 / ((1.0 / (8.0 * (1.0 + sqrtopv))) * (3.0 + sum1 / (1.0 + eta * sum1))));

            // first term is diff, indices are offset too
            //double[] c1 = new double[20]
            //   {
            //    9.0 / 7.0, 16.0 / 63.0,
            //    25.0 / 99.0, 36.0 / 143.0,
            //    49.0 / 195.0, 64.0 / 255.0,
            //    81.0 / 323.0, 100.0 / 399.0,
            //    121.0 / 483.0, 144.0 / 575.0,
            //    169.0 / 675.0, 196.0 / 783.0,
            //    225.0 / 899.0, 256.0 / 1023.0,
            //    289.0 / 1155.0, 324.0 / 1295.0,
            //    361.0 / 1443.0, 400.0 / 1599.0,
            //    441.0 / 1763.0, 484.0 / 1935.0
            //   };

            // int ktr = 19;
            // double sum2  = 0.0;   
            // double term2 = 1.0 + c1[ktr] * eta;
            // for (i = 0; i <= ktr - 1; i++)
            // {
            //     sum2 = c1[ktr - i] * eta / term2;
            //     term2 = 1.0 + sum2;
            // }
            // return (8.0*(1.0 + sqrtopv) / 
            //          (3.0 + 
            //            (1.0 / 
            //              (5.0 + eta + ((9.0/7.0)*eta/term2 ) ) ) ) );

        }  // double seebattin


        /*------------------------------------------------------------------------------
        *
        *                           procedure lamberbattin
        *
        *  this procedure solves lambert's problem using battins method. the method is
        *    developed in battin (1987).
        *
        *  author        : david vallado           davallado@gmail.com   22 jun 2002
        *
        *  inputs          description                            range / units
        *    r1          - ijk position vector 1                      km
        *    r2          - ijk position vector 2                      km
        *    v1          - ijk velocity vector 1 if avail             km/s
        *    dm          - direction of motion                       'L', 'S'
        *    de          - orbital energy                            'L', 'H'
        *                  only affects nrev >= 1 solutions
        *    dtsec       - time between r1 and r2                     sec
        *    dtwait      - time to wait before starting               sec
        *    nrev        - number of revs to complete                 0, 1, 2, 3,  
        *    altpad      - altitude pad for hitearth calc             km
        *    show        - control output don't output for speed      'y', 'n'
        *
        *  outputs       :
        *    v1t         - ijk transfer velocity vector               km/s
        *    v2t         - ijk transfer velocity vector               km/s
        *    hitearth    - flag if hti or not                         'y', 'n'
        *    errorsum    - error flag                                 'ok',  
        *    errorout    - text for iterations / last loop
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
        *    mag         - magnitude of a vector
        *
        *  references    :
        *    vallado       2013, 494, Alg 59, ex 7-5
        *    thompson      AAS GNC 2018
        -----------------------------------------------------------------------------*/

        public static LambertOutput Battin
               (
                double mu,
               double3 r1, double3 r2, double3 v1, MotionDirection dm, EnergyType de, int nrev, double t_xfer,
               double radius
               )
        {
            const double small = 0.0000000001;
            double3 rcrossr = new double3(0, 0, 0);
            int loops;
            double u, b, x, xn, y, L, m, cosdeltanu, sindeltanu, dnu, a,
                ror, h1, h2, tempx, eps, denom, chord, k2, s, f;
            double magr1, magr2, lam, temp, temp1, temp2, A, p, ecc, y1, rp;
            double3 v1dvl = new double3(0, 0, 0);
            double3 v2dvl = new double3(0, 0, 0);
            double3 v2 = new double3(0, 0, 0);
            bool hitearth = false;

            Status status = Status.OK;

            // needed since assignments aren't at root level in procedure
            double3 v1t = new double3(0, 0, 0);
            double3 v2t = new double3(0, 0, 0);

            y = 0.0;

            magr1 = math.length(r1);
            magr2 = math.length(r2);

            cosdeltanu = math.dot(r1, r2) / (magr1 * magr2);
            // make sure it's not more than 1.0
            if (math.abs(cosdeltanu) > 1.0)
                cosdeltanu = 1.0 * math.sign(cosdeltanu);

            rcrossr = math.cross(r1, r2);
            if (dm == MotionDirection.SHORT)
                sindeltanu = math.length(rcrossr) / (magr1 * magr2);
            else
                sindeltanu = -math.length(rcrossr) / (magr1 * magr2);
            dnu = math.atan2(sindeltanu, cosdeltanu);
            // the angle needs to be positive to work for the long way
            if (dnu < 0.0)
                dnu = 2.0 * math.PI + dnu;

            // these are the same
            chord = math.sqrt(magr1 * magr1 + magr2 * magr2 - 2.0 * magr1 * magr2 * cosdeltanu);
            //chord = math.length(r2 - r1);

            s = (magr1 + magr2 + chord) * 0.5;
            ror = magr2 / magr1;
            eps = ror - 1.0;

            lam = 1.0 / s * math.sqrt(magr1 * magr2) * math.cos(dnu * 0.5);
            L = math.pow((1.0 - lam) / (1.0 + lam), 2);
            m = 8.0 * mu * t_xfer * t_xfer / (s * s * s * math.pow(1.0 + lam, 6));

            // initial guess
            if (nrev > 0)
                xn = 1.0 + 4.0 * L;
            else
                xn = L;   //l    // 0.0 for par and hyp, l for ell

            // alt approach for high energy(long way, multi-nrev) case
            if ((de.Equals('H')) && (nrev > 0)) {
                h1 = 0.0;
                h2 = 0.0;
                b = 0.0;
                f = 0.0;
                xn = 1e-20;  // be sure to reset this here!!
                x = 10.0;  // starting value
                loops = 1;
                while ((math.abs(xn - x) >= small) && (loops <= 20)) {
                    x = xn;
                    temp = 1.0 / (2.0 * (L - x * x));
                    temp1 = math.sqrt(x);
                    temp2 = (nrev * math.PI * 0.5 + math.atan(temp1)) / temp1;
                    h1 = temp * (L + x) * (1.0 + 2.0 * x + L);
                    h2 = temp * m * temp1 * ((L - x * x) * temp2 - (L + x));

                    b = 0.25 * 27.0 * h2 / (math.pow(temp1 * (1.0 + h1), 3));
                    if (b < 0.0) // reset the initial condition
                        f = 2.0 * math.cos(1.0 / 3.0 * math.acos(math.sqrt(b + 1.0)));
                    else {
                        A = math.pow(math.sqrt(b) + math.sqrt(b + 1.0), (1.0 / 3.0));
                        f = A + 1.0 / A;
                    }

                    y = 2.0 / 3.0 * temp1 * (1.0 + h1) * (math.sqrt(b + 1.0) / f + 1.0);
                    xn = 0.5 * ((m / (y * y) - (1.0 + L)) - math.sqrt(math.pow(m / (y * y) - (1.0 + L), 2) - 4.0 * L));
                    // set if NANs occur   
                    if (double.IsNaN(y)) {
                        y = 75.0;
                        xn = 1.0;
                    }

                    loops = loops + 1;
                }  // while

                x = xn;
                a = s * math.pow(1.0 + lam, 2) * (1.0 + x) * (L + x) / (8.0 * x);
                p = (2.0 * magr1 * magr2 * (1.0 + x) * math.pow(math.sin(dnu * 0.5), 2)) / (s * math.pow(1.0 + lam, 2) * (L + x));  // thompson
                ecc = math.sqrt(1.0 - p / a);
                rp = a * (1.0 - ecc);
                LambHodograph(mu, r1, r2, v1, p, ecc, dnu, t_xfer, out v1t, out v2t);

            } else {
                // standard processing, low energy
                k2 = 0.0;
                b = 0.0;
                u = 0.0;
                // note that the r nrev = 0 case is not assumed here
                loops = 1;
                y1 = 0.0;
                x = 10.0;  // starting value
                while ((math.abs(xn - x) >= small) && (loops <= 30)) {
                    if (nrev > 0) {
                        x = xn;
                        temp = 1.0 / ((1.0 + 2.0 * x + L) * (4.0 * x * x));
                        temp1 = (nrev * math.PI * 0.5 + math.atan(math.sqrt(x))) / math.sqrt(x);
                        h1 = temp * math.pow(L + x, 2) * (3.0 * math.pow(1.0 + x, 2) * temp1 - (3.0 + 5.0 * x));
                        h2 = temp * m * ((x * x - x * (1.0 + L) - 3.0 * L) * temp1 + (3.0 * L + x));
                    } else {
                        x = xn;
                        tempx = SeeBattin(x);
                        denom = 1.0 / ((1.0 + 2.0 * x + L) * (4.0 * x + tempx * (3.0 + x)));
                        h1 = math.pow(L + x, 2) * (1.0 + 3.0 * x + tempx) * denom;
                        h2 = m * (x - L + tempx) * denom;
                    }

                    // ----------------------- evaluate cubic------------------
                    b = 0.25 * 27.0 * h2 / (math.pow(1.0 + h1, 3));

                    u = 0.5 * b / (1.0 + math.sqrt(1.0 + b));
                    k2 = KBattin(u);
                    y = ((1.0 + h1) / 3.0) * (2.0 + math.sqrt(1.0 + b) / (1.0 + 2.0 * u * k2 * k2));
                    xn = math.sqrt(math.pow((1.0 - L) * 0.5, 2) + m / (y * y)) - (1.0 + L) * 0.5;

                    y1 = math.sqrt(m / ((L + x) * (1.0 + x)));
                    loops = loops + 1;

                    if (double.IsNaN(y)) {
                        y = 75.0;
                        xn = 1.0;
                    }

                }  // while

                if (loops < 30) {
                    p = (2.0 * magr1 * magr2 * y * y * math.pow(1.0 + x, 2) * math.pow(math.sin(dnu * 0.5), 2)) / (m * s * math.pow(1.0 + lam, 2));  // thompson
                    ecc = math.sqrt((eps * eps + 4.0 * magr2 / magr1 * math.pow(math.sin(dnu * 0.5), 2) * math.pow((L - x) / (L + x), 2)) / (eps * eps + 4.0 * magr2 / magr1 * math.pow(math.sin(dnu * 0.5), 2)));
                    a = 1.0;   // fix
                    rp = a * (1.0 - ecc);
                    LambHodograph(mu, r1, r2, v1, p, ecc, dnu, t_xfer, out v1t, out v2t);

                }  // if loops converged < 30
            }  // if nrev and r
            hitearth = CheckHitPlanet(mu, radius, r1, v1t, r2, v2t, nrev);
            if (hitearth) {
                status = Status.HIT_PLANET;
            }
            LambertOutput output = new LambertOutput(v1t, v2t, t_xfer, status);
            output.r1 = r1;
            output.r2 = r2;
            output.v1 = v1;
            return output;
        }  //  lambertbattin


        /* -----------------------------------------------------------------------------
        *
        *                           function findc2c3
        *
        *  this function calculates the c2 and c3 functions for use in the universal
        *    variable calculation of z.
        *
        *  author        : david vallado           davallado@gmail.com   27 may 2002
        *
        *  revisions
        *                -
        *
        *  inputs          description                    range / units
        *    znew        - z variable                     rad2
        *
        *  outputs       :
        *    c2new       - c2 function value
        *    c3new       - c3 function value
        *
        *  locals        :
        *    sqrtz       - square root of znew
        *
        *  coupling      :
        *    Math.Sinh        - hyperbolic Math.Sine
        *    Math.Cosh        - hyperbolic Math.Cosine
        *
        *  references    :
        *    vallado       2013, 63, alg 1
        * --------------------------------------------------------------------------- */

        private static void FindC2C3
            (
            double znew,
            out double c2new, out double c3new
            )
        {
            double small, sqrtz;
            small = 0.00000001;

            // -------------------------  implementation   -----------------
            if (znew > small) {
                sqrtz = math.sqrt(znew);
                c2new = (1.0 - math.cos(sqrtz)) / znew;
                c3new = (sqrtz - math.sin(sqrtz)) / (sqrtz * sqrtz * sqrtz);
            } else {
                if (znew < -small) {
                    sqrtz = math.sqrt(-znew);
                    c2new = (1.0 - math.cosh(sqrtz)) / znew;
                    c3new = (math.sinh(sqrtz) - sqrtz) / (sqrtz * sqrtz * sqrtz);
                } else {
                    c2new = 0.5;
                    c3new = 1.0 / 6.0;
                }
            }
        }  //  findc2c3

        /* ------------------------------------------------------------------------------
        //
        //                           function checkhitearth
        //
        //  this function checks to see if the trajectory hits the earth during the
        //    transfer. 
        //
        //  author        : david vallado           davallado@gmail.com   14 aug 2017
        //
        //  inputs          description                    range / units
        //    altPad      - pad for alt above surface       km  
        //    r1          - initial position vector of int  km   
        //    v1t         - initial velocity vector of trns km/s
        //    r2          - final position vector of int    km
        //    v2t         - final velocity vector of trns   km/s
        //    nrev        - number of revolutions           0, 1, 2,  
        //
        //  outputs       :
        //    hitearth    - is earth was impacted           'y' 'n'
        //    hitearthstr - is earth was impacted           "y - radii" "no"
        //
        //  locals        :
        //    sme         - specific mechanical energy
        //    rp          - radius of perigee               km
        //    a           - semimajor axis of transfer      km
        //    ecc         - eccentricity of transfer
        //    p           - semi-paramater of transfer      km
        //    hbar        - angular momentum vector of
        //                  transfer orbit
        //    radiuspad   - radius including user pad       km
        //
        //  coupling      :
        //    dot         - dot product of vectors
        //    mag         - magnitude of a vector
        //    MathTimeLibr.cross       - MathTimeLibr.cross product of vectors
        //
        //  references    :
        //    vallado       2013, 503, alg 60
        // ------------------------------------------------------------------------------*/

        public static bool CheckHitPlanet
            (
                double mu,
                double radius,
                double3 r1,
                double3 v1t,
                double3 r2,
                double3 v2t,
                int nrev
            )
        {
            double magh, magv1, v12, ainv, ecc, ecosea1, esinea1, ecosea2;
            double3 hbar;
            double rp = 0.0;
            double a = 0.0;

            double magr1 = math.length(r1);
            double magr2 = math.length(r2);

            bool hitearth = false;

            // check whether Lambert transfer trajectory hits the Earth
            if (magr1 < radius || magr2 < radius) {
                // hitting earth already at start or stop point
                hitearth = true;
            } else {
                double rdotv1 = math.dot(r1, v1t);
                double rdotv2 = math.dot(r2, v2t);

                // Solve for a 
                magv1 = math.length(v1t);
                v12 = magv1 * magv1;
                ainv = 2.0 / magr1 - v12 / mu;

                // Find ecos(E) 
                ecosea1 = 1.0 - magr1 * ainv;
                ecosea2 = 1.0 - magr2 * ainv;

                // Determine radius of perigee
                // 4 distinct cases pass thru perigee 
                // nrev > 0 you have to check
                if (nrev > 0) {
                    a = 1.0 / ainv;
                    // elliptical orbit
                    if (a > 0.0) {
                        esinea1 = rdotv1 / math.sqrt(mu * a);
                        ecc = math.sqrt(ecosea1 * ecosea1 + esinea1 * esinea1);
                    }
                    // hyperbolic orbit
                    else {
                        esinea1 = rdotv1 / math.sqrt(mu * math.abs(-a));
                        ecc = math.sqrt(ecosea1 * ecosea1 - esinea1 * esinea1);
                    }
                    rp = a * (1.0 - ecc);
                    if (rp < radius) {
                        hitearth = true;
                    }
                }
                // nrev = 0, 3 cases:
                // heading to perigee and ending after perigee
                // both headed away from perigee, but end is closer to perigee
                // both headed toward perigee, but start is closer to perigee
                else {
                    // this rp calc section is debug only!!!
                    a = 1.0 / ainv;
                    hbar = math.cross(r1, v1t);
                    magh = math.length(hbar);
                    double3 nbar = new double3();
                    double3 ebar = new double3();
                    nbar[0] = -hbar[1];
                    nbar[1] = hbar[0];
                    nbar[2] = 0.0;
                    double magn = math.length(nbar);
                    double c1 = magv1 * magv1 - mu / magr1;
                    double rdotv = math.dot(r1, v1t);
                    for (int i = 0; i <= 2; i++)
                        ebar[i] = (c1 * r1[i] - rdotv * v1t[i]) / mu;
                    ecc = math.length(ebar);
                    rp = a * (1.0 - ecc);


                    // check only cases that may be a problem
                    // if ((rdotv1 < 0.0 && rdotv2 > 0.0) || (rdotv1 > 0.0 && rdotv2 > 0.0 && ecosea1 < ecosea2) ||
                    // (rdotv1 < 0.0 && rdotv2 < 0.0 && ecosea1 > ecosea2)) {
                    if ((rdotv1 < 0.0 && rdotv2 > 0.0) || nrev > 0 || (rdotv1 > 0.0 && rdotv2 > 0.0 && ecosea1 < ecosea2) || (rdotv1 < 0.0 && rdotv2 < 0.0 && ecosea1 > ecosea2)) {

                        // parabolic orbit
                        if (math.abs(ainv) <= 1.0e-10) {
                            hbar = math.cross(r1, v1t);
                            magh = math.length(hbar); // find h magnitude
                            rp = magh * magh * 0.5 / mu;
                            if (rp < radius) {
                                hitearth = true;
                            }
                        } else {
                            a = 1.0 / ainv;
                            // elliptical orbit
                            if (a > 0.0) {
                                esinea1 = rdotv1 / math.sqrt(mu * a);
                                ecc = math.sqrt(ecosea1 * ecosea1 + esinea1 * esinea1);
                            }
                            // hyperbolic orbit
                            else {
                                esinea1 = rdotv1 / math.sqrt(mu * math.abs(-a));
                                ecc = math.sqrt(ecosea1 * ecosea1 - esinea1 * esinea1);
                            }
                            if (ecc < 1.0) {
                                rp = a * (1.0 - ecc);
                            } else {
                                // hyperbolic heading towards the earth
                                if (rdotv1 < 0.0 && rdotv2 > 0.0) {
                                    rp = a * (1.0 - ecc);
                                } else if (rdotv1 < 0.0 && rdotv2 < 0.0) {
                                    // NBP - both heading inwards
                                    rp = magr2;
                                } else if (rdotv1 > 0.0 && rdotv2 > 0.0) {
                                    // NBP - both heading outwards
                                    rp = magr1;
                                }
                            }
                            if (rp < radius) {
                                hitearth = true;
                            }

                        } // ell and hyp checks
                    } // end nrev = 0 cases
                }  // nrev = 0 cases
            }  // check if starting positions ok
            return hitearth;
        } // checkhitearth


        /// <summary>
        /// Determine the velocity change to transfer from r1 to r2 with a given flight path angle.
        /// 
        /// This algorithm is very efficient (no inner iteration to root find) BUT it does not
        /// compute the transfer time. This can be done explicitly with ComputeXferTime.  
        /// 
        /// If a radius is specified then a hit check will require that the orbit velocity at r2 is
        /// computed and this will result in additional computation.
        /// </summary>
        /// <param name="mu">GM of the central body</param>
        /// <param name="r1">Initial position</param>
        /// <param name="r2">Final position</param>
        /// <param name="v1">Initial velocity</param>
        /// <param name="fpa">Flight path angle</param>
        /// <param name="radius">Radius of the central body</param>
        /// <returns></returns>
        public static LambertOutput TransferProgradeToPointWithFPA(double mu, double3 r1, double3 r2, double3 v1, double fpa, double radius = 0)
        {
            double angle = GravityMath.AngleRadiansPrograde(r1, v1, r2);
            double fpa_adjusted = fpa;
            if (angle > math.PI) {
                fpa_adjusted = math.PI - fpa;
            }

            double r1_mag = math.length(r1);
            double r2_mag = math.length(r2);

            // eqn (3.35) DiPrinzio (p83)
            double cosfpa = math.cos(fpa);
            double denom = cosfpa * cosfpa * (r1_mag * r1_mag - r1_mag * r2_mag * (math.cos(angle) - math.sin(angle) * math.tan(fpa)));
            double v1sq = mu * r2_mag * (1.0 - math.cos(angle)) / denom;
            double v1t_mag = math.sqrt(v1sq);

            // get direction based on FPA. 
            double3 v1t = v1t_mag * Orbital.VDirFromFPAandRR(fpa_adjusted, r1, r2);
            double3 v2t = double3.zero;
            Status status = Status.OK;
            LambertOutput output = new LambertOutput(v1t, v2t, 0.0, status);
            if (radius > 0.0) {
                // need v2t to check for hit planet. This seems heavy handed...
                Orbital.COE coe = Orbital.RVtoCOE(r1, v1t, mu);
                double tof = Orbital.TimeOfFlight(r1, r2, coe);
                (r2, v2t) = Orbital.COEtoRVatTime(coe, tof);
                bool hitplanet = CheckHitPlanet(mu, radius, r1, v1t, r2, v2t, nrev: 0);
                if (hitplanet) {
                    status = Status.HIT_PLANET;
                }
                output.t = tof;
                output.v2t = v2t;
                output.status = status;
            }
            return output;
        }

        public static double ComputeTransferTime(double mu, double3 r1, double3 r2, ref LambertOutput lambertOutput)
        {
            Orbital.COE coe = Orbital.RVtoCOE(r1, lambertOutput.v1t, mu);
            double tof = Orbital.TimeOfFlight(r1, r2, coe);
            lambertOutput.t = tof;
            return tof;
        }


    }
}
