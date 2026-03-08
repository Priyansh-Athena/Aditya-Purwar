using System; // Math
using Unity.Mathematics;
using Unity.Collections;

namespace GravityEngine2 {
    /// <summary>
    /// Code to propagate Kepler orbits (ellipse, parabola, hyperbola). 
    /// 
    /// This code will be called from Job in the job system so it cannot make reference to any
    /// classes, hence the implementation as a data struct and static methods.
    /// 
    /// The propagator is based on the universal variable Kepler method from Vallado. This is
    /// chosen since it has smooth hand off between ellipse, parabola and hyperbola.
    ///
    /// RVT has two constructors, one that preserves the initial COE (which can then be retreived by e.g.
    /// orbit display code that does not care about the exact phase of the orbit (since it will be
    /// drawing the entire shape). 
    /// </summary>
    public class KeplerPropagator {
        private const double small = 1E-6;
        private const double halfpi = math.PI * 0.5;

        public struct RVT {
            // r0, v0 are relative to the center body
            public double3 r0;
            public double3 v0;
            public double t0;
            public double mu;
            public Orbital.COEStruct coe0;     // initial COE
            public bool initByCOE;

            // values required for the Kepler algorithm based on initial r, v
            public double mag_r0;

            public bool radialInfall;
            public double sme;
            public double alpha;
            public double rdotv;
            public double a;


            public RVT(double3 r0, double3 v0, double t0, double mu)
            {
                this.r0 = r0;
                this.v0 = v0;
                this.t0 = t0;
                this.mu = mu;
                radialInfall = math.length(math.cross(math.normalize(r0), math.normalize(v0))) < small;
                Orbital.COE coe = Orbital.RVtoCOE(r0, v0, mu);
                coe0 = new Orbital.COEStruct(coe);
                initByCOE = false;

                // need C&P here since struct
                mag_r0 = math.length(r0);
                double mag_v0 = math.length(v0);
                sme = sme = ((mag_v0 * mag_v0) * 0.5) - (mu / mag_r0);
                alpha = -sme * 2.0 / mu;
                rdotv = math.dot(r0, v0);
                if (Math.Abs(sme) > 1E-12)
                    a = -mu / (2.0 * sme);
                else
                    a = double.NaN;
                Init();
            }

            public RVT(Orbital.COE coe, double t0, double mu)
            {
                coe.mu = mu;
                (r0, v0) = Orbital.COEtoRVatTime(coe, 0.0);
                radialInfall = math.length(math.cross(math.normalize(r0), math.normalize(v0))) < small;
                this.t0 = t0;
                this.mu = mu;
                this.coe0 = new Orbital.COEStruct(coe);
                initByCOE = true;

                // need C&P here since struct
                mag_r0 = math.length(r0);
                double mag_v0 = math.length(v0);
                sme = sme = ((mag_v0 * mag_v0) * 0.5) - (mu / mag_r0);
                alpha = -sme * 2.0 / mu;
                rdotv = math.dot(r0, v0);
                if (Math.Abs(sme) > 1E-12)
                    a = -mu / (2.0 * sme);
                else
                    a = double.NaN;
                Init();
            }

            public void Init()
            {
                mag_r0 = math.length(r0);
                double mag_v0 = math.length(v0);
                sme = sme = ((mag_v0 * mag_v0) * 0.5) - (mu / mag_r0);
                alpha = -sme * 2.0 / mu;
                rdotv = math.dot(r0, v0);
                if (Math.Abs(sme) > 1E-12)
                    a = -mu / (2.0 * sme);
                else
                    a = double.NaN;
            }
            override
            public string ToString()
            {
                return string.Format(" t0={0} r0={1} v0={2} mu={3} a={4}", t0, r0, v0, mu, a);
            }

            public string ToWorldString(GBUnits.GEScaler scaler)
            {
                double lenScale = scaler.ScaleLenGEToWorld(1.0);
                double velScale = scaler.ScaleVelocityGEToWorld(1.0);
                double tScale = scaler.ScaleTimeGEToWorld(1.0);
                double massScale = scaler.ScaleMassGEToWorld(1.0);
                return string.Format(" t0={0} r0={1} v0={2} mass={3} a={4}", t0 * tScale, r0 * lenScale, v0 * velScale, mu * massScale, a * lenScale);
            }
        }

        public const int MAX_KEPLER_DEPTH = 3;

        public struct PropInfo {
            public RVT rvt;
            public int centerId;

            public int keplerDepth;

            public PropInfo(double3 r, double3 v, double t, double mu, int centerId, int keplerDepth)
            {
                rvt = new RVT(r, v, t, mu);
                this.centerId = centerId;
                this.keplerDepth = keplerDepth;
            }

            public PropInfo(double3 r, double3 v, double t, PropInfo copyFrom)
            {
                rvt = new RVT(r, v, t, copyFrom.rvt.mu);
                centerId = copyFrom.centerId;
                keplerDepth = copyFrom.keplerDepth;
            }

            public PropInfo(PropInfo copyFrom)
            {
                rvt = copyFrom.rvt; // copies
                centerId = copyFrom.centerId;
                keplerDepth = copyFrom.keplerDepth;
            }
        }

        /// <summary>
        /// Evolves all the massive Kepler bodies to their new absolute position. 
        /// 
        /// This is done heirarchically, so MASSIVE1, 2, 3.
        /// </summary>
        /// <param name="bodies"></param>
        /// <param name="rvtEntries"></param>
        /// <param name="parms"></param>
        public static void EvolveKeplerMassive(double t_to, ref GEPhysicsCore.GEBodies bodies,
                                        ref NativeArray<PropInfo> kpi,
                                        ref NativeArray<int> indices,
                                        int lenIndices)
        {
            double3 r, v;
            int status;
            int p = -1;
            // find the max kepler depth in the indices. A bit greedy but very flexible.
            int maxDepth = 0;
            for (int j = 0; j < lenIndices; j++) {
                int index = indices[j];
                if (kpi[bodies.propIndex[index]].keplerDepth > maxDepth) {
                    maxDepth = kpi[bodies.propIndex[index]].keplerDepth;
                }
            }
            // need to update in heirarchy order
            for (int i = 0; i <= maxDepth; i++) {
                for (int j = 0; j < lenIndices; j++) {
                    int index = indices[j];
                    if (bodies.self_integrating[index]) {
                        continue;
                    }
                    p = bodies.propIndex[index];
                    if (kpi[p].keplerDepth == i) {
                        (status, r, v) = RVforTime(kpi[p].rvt, t_to);
                        if (status == T_TOO_EARLY) {
                            bodies.r[index] = r;   // will be NaN and controller will handle from there
                        } else {
                            bodies.r[index] = r + bodies.r[kpi[p].centerId];
                            bodies.v[index] = v + bodies.v[kpi[p].centerId];
                        }
                    }
                }

            }
        }

        public static bool EvolveMasslessProgagators(double t_to, ref GEPhysicsCore.GEBodies bodies,
                                ref NativeArray<PropInfo> kpi,
                                ref NativeArray<GEPhysicsCore.PatchInfo> patchInfo,
                                ref NativeArray<int> indices,
                                int lenIndices,
                                ref NativeList<GEPhysicsCore.PhysEvent> eventList,
                                bool coRoFrame)
        {
            double3 r, v;
            int status;
            int p;
            bool physEvent = false;
            for (int j = 0; j < lenIndices; j++) {
                int i = indices[j];
                if (bodies.self_integrating[i]) {
                    continue;
                }
                if (bodies.patchIndex[i] >= 0) {
                    p = patchInfo[bodies.patchIndex[i]].propIndex;
                    if (patchInfo[bodies.patchIndex[i]].prop != GEPhysicsCore.Propagator.KEPLER) {
                        throw new System.NotImplementedException("patch prop does not match KEPLER");
                    }
                } else {
                    p = bodies.propIndex[i];
                }
                if (bodies.propType[i] == GEPhysicsCore.Propagator.KEPLER) {
                    (status, r, v) = RVforTime(kpi[p].rvt, t_to);
                    if (status == T_TOO_EARLY) {
                        GEPhysicsCore.PhysEvent kError = new GEPhysicsCore.PhysEvent {
                            type = GEPhysicsCore.EventType.KEPLER_ERROR,
                            bodyId = i,
                            bodyId_secondary = kpi[p].centerId
                        };
                        eventList.Add(kError);
                        physEvent = true;
                    }
                    // If CoRo need to convert center position to inertial BUG If heoirarchical will be in inertial frame...
                    if (coRoFrame) {
                        (double3 cr, double3 cv) =
                            CR3BP.FrameRotatingToInertial(bodies.r[kpi[p].centerId], bodies.v[kpi[p].centerId], t_to);
                        bodies.r[i] = r + cr;
                        bodies.v[i] = v + cv;
                    } else {
                        bodies.r[i] = r + bodies.r[kpi[p].centerId];
                        bodies.v[i] = v + bodies.v[kpi[p].centerId];
                    }
                } else {
                    throw new NotImplementedException("unsupported propagator");
                }
            }
            return physEvent;
        }

        /// <summary>
		/// Version to allow direct use of a propagator in GE
		/// </summary>
		/// <param name="t"></param>
		/// <param name="propId"></param>
		/// <param name="kpi"></param>
		/// <param name="state"></param>
        public static void EvolveRelative(double t, int propId, ref NativeArray<PropInfo> kpi, ref GEBodyState state)
        {
            int status;
            (status, state.r, state.v) = RVforTime(kpi[propId].rvt, t);
            if (status == 0)
                state.t = t;
            else
                state.t = -1.0;
        }

        /// <summary>
        /// Update the KeplerPropagator for the body at bodyIndex with the new position, 
        /// velocity and time based on applying a maneuver in the Run() loop.
        /// </summary>
        /// <param name="bodyIndex"></param>
        /// <param name="time"></param>
        /// <param name="bodies"></param>
        /// <param name="kpi"></param>
        public static void ManeuverPropagator(int bodyIndex,
                        double time,
                        ref GEPhysicsCore.GEBodies bodies,
                        ref NativeArray<PropInfo> kpi)
        {
            int p = bodies.propIndex[bodyIndex];
            int centerId = kpi[p].centerId;
            RVT rvt = kpi[p].rvt;
            RVT rvtNew = new RVT(bodies.r[bodyIndex] - bodies.r[centerId],
                                 bodies.v[bodyIndex] - bodies.v[centerId],
                                 time,
                                 rvt.mu);
            PropInfo kpiNew = new PropInfo {
                rvt = rvtNew,
                centerId = kpi[p].centerId
            };
            kpi[p] = kpiNew;
        }


        // Orignal comments from Vallado
        /* -----------------------------------------------------------------------------
        *
        *                           function kepler
        *
        *  this function solves keplers problem for orbit determination and returns a
        *    future geocentric equatorial (ijk) position and velocity vector.  the
        *    solution uses universal variables.
        *
        *  author        : david vallado                  719-573-2600   22 jun 2002
        *
        *  revisions
        *    vallado     - fix some mistakes                             13 apr 2004
        *
        *  inputs          description                    range / units
        *    ro          - ijk position vector - initial  km
        *    vo          - ijk velocity vector - initial  km / s
        *    dtsec       - length of time to propagate    s
        *
        *  outputs       :
        *    r           - ijk position vector            km
        *    v           - ijk velocity vector            km / s
        *    error       - error flag                     'ok', ...
        *
        *  locals        :
        *    f           - f expression
        *    g           - g expression
        *    fdot        - f dot expression
        *    gdot        - g dot expression
        *    xold        - old universal variable x
        *    xoldsqrd    - xold squared
        *    xnew        - new universal variable x
        *    xnewsqrd    - xnew squared
        *    znew        - new value of z
        *    c2new       - c2(psi) function
        *    c3new       - c3(psi) function
        *    dtsec       - change in time                 s
        *    timenew     - new time                       s
        *    rdotv       - result of ro dot vo
        *    a           - semi or axis                   km
        *    alpha       - reciprocol  1/a
        *    sme         - specific mech energy           km2 / s2
        *    period      - time period for satellite      s
        *    s           - variable for parabolic case
        *    w           - variable for parabolic case
        *    h           - angular momentum vector
        *    temp        - temporary real*8 value
        *    i           - index
        *
        *  coupling      :
        *    mag         - magnitude of a vector
        *    findc2c3    - find c2 and c3 functions
        *
        *  references    :
        *    vallado       2013, 93, alg 8, ex 2-4
        ---------------------------------------------------------------------------- */
        /// <summary>
        /// Evolve the orbit to the time indicated. The algorithm used requires some internal
        /// iteration but in general converges very quiclky. (All solutions to Kepler's equation
        /// use some iteration, since the equation is not closed form).
        /// 
        /// Universal Kepler evolution using KEPLER (algorithm 8) from Vallado, p93
        /// Code taken from book companion site and adapted to C#/Unity.
        /// </summary>
        /// <param name="physicsTime"></param>
        /// <param name="r_new">Position at the specified time (returned by ref)</param>
        /// 


        public const int PROP_OK = 0;
        public const int T_TOO_EARLY = 1;
        public const int KUNIV_NOT_CONVERGED = 2;

        public const int INFALL_NOT_CONVERGED = 3;

        public static string[] errorText = new string[] { "OK", "Time < time0", "Did not converge" };

        public static (int status, double3, double3) RVforTime(RVT rvt, double physicsTime)
        {

            if (rvt.radialInfall) {
                return EvolveRecilinearUnboundCM(physicsTime, rvt);
            }
            // If XZ orbits then r0, v0 are in the XZ plane and this will work without any special code. 
            int ktr, numiter;
            double f, g, fdot, gdot, rval, xold, xoldsqrd,
                xnewsqrd, znew, dtnew, a, dtsec,
                alpha, sme, s, w, temp,
                magro, magr;
            double c2new = 0.0;
            double c3new = 0.0;
            double xnew = 0.0;

            double time0 = rvt.t0;

            double3 r_new = new(double.NaN, double.NaN, double.NaN);
            double3 v_new = new(double.NaN, double.NaN, double.NaN);

            if (Math.Abs(physicsTime - time0) < 1E-5)
                return (PROP_OK, rvt.r0, rvt.v0);

            // can have a weird precision issue when same time used in Init and first evolve.
            if ((physicsTime - time0) < 0) {
                return (T_TOO_EARLY, r_new, v_new);
            }
            // evolution time is relative to time0
            double dtseco = physicsTime - time0;

            // Very large times can cause precision issues, normalize if possible
            //if ((eccentricity < 1) && (dtseco > 1E6))
            //{
            //    dtseco = dtseco % orbit_period;
            //}

            dtsec = dtseco;

            // -------------------------  implementation   -----------------
            // set constants and intermediate printouts
            // Did find some test cases where convergence was very slow. Bump the limit. 
            // This is typically only for the first few calls when the time is close to the initial time. 
            numiter = 1000;

            // --------------------  initialize values   -------------------
            znew = 0.0;

            if (Math.Abs(dtseco) > small) {
                // <TODO>: (performance) put this where r0, v0 are changed and re-use it. 
                magro = rvt.mag_r0;
                // </TODO>

                // -------------  find sme, alpha, and a  ------------------
                sme = rvt.sme;
                alpha = rvt.alpha;

                a = rvt.a;

                // ------------   setup initial guess for x  ---------------
                // -----------------  circle and ellipse -------------------
                // if (alpha >= small) {
                if (alpha >= 0) {
                    if (Math.Abs(alpha - 1.0) > small)
                        xold = Math.Sqrt(rvt.mu) * dtsec * alpha;
                    else
                        // - first guess can't be too close. ie a circle, r=a
                        xold = Math.Sqrt(rvt.mu) * dtsec * alpha * 0.97;
                } else {
                    // --------------------  parabola  ---------------------
                    if (Math.Abs(alpha) < small) {
                        double3 h = math.cross(rvt.r0, rvt.v0);
                        double p = math.lengthsq(h) / rvt.mu;
                        s = 0.5 * (halfpi - Math.Atan(3.0 * Math.Sqrt(rvt.mu / (p * p * p)) * dtsec));
                        w = Math.Atan(Math.Pow(Math.Tan(s), 1.0 / 3.0));
                        xold = Math.Sqrt(p) * (2.0 * GravityMath.Cot(2.0 * w));
                        alpha = 0.0;
                    } else {
                        // ------------------  hyperbola  ------------------
                        temp = -2.0 * rvt.mu * dtsec /
                            (a * (rvt.rdotv + Math.Sign(dtsec) * Math.Sqrt(-rvt.mu * a) * (1.0 - magro * alpha)));
                        xold = Math.Sign(dtsec) * Math.Sqrt(-a) * Math.Log(temp);
                    }
                } // if alpha

                ktr = 1;
                dtnew = -10.0;
                // conv for dtsec to x units
                double tmp = 1.0 / Math.Sqrt(rvt.mu);

                while ((Math.Abs(dtnew * tmp - dtsec) >= small) && (ktr < numiter)) {
                    xoldsqrd = xold * xold;
                    znew = xoldsqrd * alpha;

                    // ------------- find c2 and c3 functions --------------
                    FindC2C3(znew, out c2new, out c3new);

                    // ------- use a newton iteration for new values -------
                    rval = xoldsqrd * c2new + rvt.rdotv * tmp * xold * (1.0 - znew * c3new) +
                        magro * (1.0 - znew * c2new);
                    dtnew = xoldsqrd * xold * c3new + rvt.rdotv * tmp * xoldsqrd * c2new +
                        magro * xold * (1.0 - znew * c3new);

                    // ------------- calculate new value for x -------------
                    xnew = xold + (dtsec * Math.Sqrt(rvt.mu) - dtnew) / rval;

                    //UnityEngine.Debug.LogFormat("znew: {0}, c2new: {1}, c3new: {2}, rval: {3}, dtnew: {4}, xnew: {5}", znew, c2new, c3new, rval, dtnew, xnew);

                    // ----- check if the univ param goes negative. if so, use bissection
                    if (xnew < 0.0)
                        xnew = xold * 0.5;

                    ktr = ktr + 1;
                    xold = xnew;
                }  // while

                if (ktr >= numiter) {   // Has not converged - issue warning
                    return (KUNIV_NOT_CONVERGED, r_new, v_new); // NaN
                } else {
                    //UnityEngine.Debug.LogFormat("Converged in {0} iterations t={1}", ktr, physicsTime);
                    // --- find position and velocity vectors at new time --
                    xnewsqrd = xnew * xnew;
                    f = 1.0 - (xnewsqrd * c2new / magro);
                    g = dtsec - xnewsqrd * xnew * c3new / Math.Sqrt(rvt.mu);
                    r_new = new(f * rvt.r0.x + g * rvt.v0.x,
                                f * rvt.r0.y + g * rvt.v0.y,
                                f * rvt.r0.z + g * rvt.v0.z);
                    magr = Math.Sqrt(r_new[0] * r_new[0] + r_new[1] * r_new[1] + r_new[2] * r_new[2]);
                    gdot = 1.0 - (xnewsqrd * c2new / magr);
                    fdot = (Math.Sqrt(rvt.mu) * xnew / (magro * magr)) * (znew * c3new - 1.0);
                    temp = f * gdot - fdot * g;
                    //if (Math.Abs(temp - 1.0) > 0.00001)
                    //    Debug.LogWarning(string.Format("consistency check failed {0}", (temp - 1.0)));
                    v_new = new(fdot * rvt.r0.x + gdot * rvt.v0.x,
                                fdot * rvt.r0.y + gdot * rvt.v0.y,
                                fdot * rvt.r0.z + gdot * rvt.v0.z);
                }
            } // if fabs
            else {
                // ----------- set vectors to incoming since 0 time --------
                r_new = new(rvt.r0.x, rvt.r0.y, rvt.r0.z);
            }
            return (PROP_OK, r_new, v_new);
        }   // kepler


        /// <summary>
        /// Evolve the rectilinear motion based on the algorithm in "About the rectilinear Kepler motion"
        /// by Condurache and Martinusi. Bul. Inst. Polit. Iasi. 
        /// 
        /// For a radial infall this will "bounce".
        /// <param name="t"></param>
        /// <param name="rvt"></param>
        /// <returns></returns>
        private static (int status, double3 r, double3 v) EvolveRecilinearUnboundCM(double t, RVT rvt)
        {
            int status = PROP_OK;
            double rdotv = math.dot(rvt.r0, rvt.v0);
            double3 r_unit = math.normalize(rvt.r0);
            double tt0 = t - rvt.t0;
            double r0f = 2 * math.length(rvt.r0) / rdotv;
            double radicand = 3 * tt0 / rvt.mu + r0f * r0f * r0f;
            double r_mag = rvt.mu * math.sqrt(radicand);
            double v_mag = 2 / math.pow(radicand, 1.0 / 3.0);
            return (status, r_unit * r_mag, r_unit * v_mag);
        }



        public static void FindC2C3(double znew, out double c2new, out double c3new)
        {
            double small, sqrtz;
            small = 0.00000001;

            // -------------------------  implementation   -----------------
            if (znew > small) {
                sqrtz = System.Math.Sqrt(znew);
                c2new = (1.0 - System.Math.Cos(sqrtz)) / znew;
                c3new = (sqrtz - System.Math.Sin(sqrtz)) / (sqrtz * sqrtz * sqrtz);
            } else {
                if (znew < -small) {
                    sqrtz = System.Math.Sqrt(-znew);
                    c2new = (1.0 - System.Math.Cosh(sqrtz)) / znew;
                    c3new = (System.Math.Sinh(sqrtz) - sqrtz) / (sqrtz * sqrtz * sqrtz);
                } else {
                    c2new = 0.5;
                    c3new = 1.0 / 6.0;
                }
            }
        }  // findc2c3
    }
}
