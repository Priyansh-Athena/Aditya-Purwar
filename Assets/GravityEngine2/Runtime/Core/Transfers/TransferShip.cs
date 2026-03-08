using System.Collections.Generic;
using System.Diagnostics;
using Unity.Mathematics;

namespace GravityEngine2 {
    /// <summary>
    /// Class to determine the maneuvers required to get a ship at an existing position in orbit around
    /// a center body to a specified orbit. 
    /// 
    /// There are two general classes of ship transfers:
    /// SHIP_TO_ORBIT: Transfer the ship to a designated target orbit, but do not care which 
    /// exact point the target orbit is entered. 
    /// 
    /// SHIP_TO_TARGET: Transfer the ship to the position of a target object or a target point in 
    /// physics space. This can be either a rendezvous (in which a second maneuver will match the 
    /// target's velocity) or intercept in which there is no second maneuver.
    /// 
    /// SHIP_TO_POINT: Determine a transfer that gets the ship to the point specified. The point must
    ///   be set with SetTransferPoint()
    /// 
    /// The specific transfer mode can be one of:
    /// BEST: If the orbits are circular, do Hohmann. Otherwise do a LAMBERT.
    /// CIRCULARIZE:
    /// HOHMANN: Explicitly request a Hohmann transfer. May fail if orbits are not circular.
    /// LAMBERT: Perform a LAMBERT transfer using the timeFactor * t_minimumEnergy for the indicated
    /// start and end points. 
    /// 
    /// Q: What is end point for a Lambert to a target orbit??
    /// 
    /// In all cases (except to a target point) the constructor is given the state of the ship and the
    /// COE of the target. 
    /// 
    /// These algorithms all assume the ship and target are in orbit around the same central body. This is
    /// not explicitly verified.
    /// 
    /// This code is not a Monobehaviour. To use in an object in a scene see GSTransferShip which is the
    /// "in scene" wrapper.
    /// 
    /// The maneuvers are computed via a call to Compute() and once done maneuvers can be retrieved
    /// via Maneuvers(). The caller then needs to add these to GE (using the correct bodyId and time offset)
    /// 
    /// </summary>
    public class TransferShip {
        public enum TargetMode {
            SHIP_TO_ORBIT,
            TARGET_ORBIT,
            TARGET_RDVS,
            TARGET_INTERCEPT,
            SHIP_TO_POINT,
            CIRCULARIZE
        };

        public bool lambertAlways = false;

        public enum Status { OK, FAILED, DEST_NOT_CIRCULAR, SHIP_NOT_CIRCULAR, HIT_PLANET, TO_ORBIT_NO_MU };

        public enum LambertTimeType { RELATIVE_TO_NOMINAL, WORLD_TIME };

        private LambertTimeType timeMode;

        private GEBodyState shipState;
        private Orbital.COE toOrbit;

        private double3 toPoint;

        private double timeFactor = 1.0;

        private double timeTransfer = 10.0;

        private double mu;

        private double circleThreshold;

        private TargetMode lastTargetMode;
        private List<GEManeuver> maneuvers;
        private double dVtotal;

        private Orbital.COE shipCOE;

        private bool checkHit;
        private double planetRadius;

        // Lambert transfer will be in retrograde direction (generally very high dV compared to prograde!)
        private bool retrogradeTransfer;

        private bool legacyRetrograde;

        /// <summary>
        /// Constructor for case with a target orbit
        /// </summary>
        /// <param name="shipState">Ship state relative to the central body</param>
        /// <param name="toOrbit"></param>
        /// <param name="mu">mu for the central body (from ge.GetMu() or G*M)</param>
        public TransferShip(GEBodyState shipState,
                            Orbital.COE toOrbit,
                            double mu,
                            double circleThreshold = Orbital.SMALL_E,
                            bool checkHit = false,
                            double planetRadius = 0.0,
                            bool retrograde = false,
                            bool legacyRetrograde = false
                            )
        {
            this.shipState = shipState;
            this.toOrbit = toOrbit;
            this.mu = mu;
            this.circleThreshold = circleThreshold;
            this.checkHit = checkHit;
            this.planetRadius = planetRadius;
            this.retrogradeTransfer = retrograde;
            this.legacyRetrograde = legacyRetrograde;
            shipCOE = Orbital.RVtoCOE(shipState.r, shipState.v, mu);
            maneuvers = new List<GEManeuver>();
            dVtotal = 0;
        }

        /// <summary>
        /// Set the transfer point in GE internal units. 
        /// </summary>
        /// <param name="p"></param>
        public void SetTargetPoint(double3 p)
        {
            toPoint = p;
        }

        public double GetDvTotal()
        {
            return dVtotal;
        }

        /// <summary>
        /// The relative time from start of maneuvers when the sequence will be complete. This will always be the time
        /// of the last maneuver
        /// </summary>
        /// <returns></returns>
        public double TimeComplete()
        {
            if (maneuvers.Count == 0) {
                UnityEngine.Debug.LogWarning("Compute maneuvers first.");
                return -1;
            }
            return maneuvers[maneuvers.Count - 1].t_relative;
        }

        /// <summary>
        /// Return the list of maneuvers. These will be maneuvers relative to some centerId 
        /// and will be set to begin at t=0.
        /// 
        /// In cases other than PKEPELR and SGP4 propagation the default values for the remaining
        /// parameters are appropriate.
        /// 
        /// In cases where SGP4 or PKEPLER is use it is necessary to change the propagator
        /// for the intermediate legs of the transfer to be KEPLER because this is what te tranfer algorithm
        /// uses when computing the transfer. Since a body cannot switch a propagator in the middle
        /// of physics evolution such bodies must be configured as patched so each leg of the maneuver is a 
        /// patch segment. These can then have different propagators.
        /// </summary>
        /// <param name="centerId">the optional centerId to assign to the maneuvers</param>
        /// <param name="propagator">the propagator for the ship after transfer</param>
        /// <param name="keplerTranfer">if true, then the intermediate maneuvers will be patched Kepler segments</param>
        /// <returns></returns>
        public List<GEManeuver> Maneuvers(int centerId = -1,
                                        GEPhysicsCore.Propagator propagator = GEPhysicsCore.Propagator.GRAVITY)
        {
            if (centerId >= 0) {
                foreach (GEManeuver m in maneuvers)
                    m.centerId = centerId;
            }
            GEPhysicsCore.Propagator shipProp = propagator;
            // if the ship propagator is PKEPLER or SGP$ then the xfer segments must be KEPLER since this
            // is what it was computed with. Then use shipProp for the last segment.
            if (shipProp == GEPhysicsCore.Propagator.PKEPLER || shipProp == GEPhysicsCore.Propagator.SGP4_RAILS) {
                shipProp = GEPhysicsCore.Propagator.KEPLER;
            }

            foreach (GEManeuver m in maneuvers)
                m.prop = shipProp;
            // set the last maneuver to have the requested propagator
            maneuvers[maneuvers.Count - 1].prop = propagator;

            return maneuvers;
        }

        public Status Compute(TargetMode targetMode)
        {
            lastTargetMode = targetMode;
            Status status = Status.OK;
            if (targetMode == TargetMode.CIRCULARIZE) {
                Circularize();
            } else if (targetMode != TargetMode.SHIP_TO_POINT) {
                // try a Hohmann, will fail if to/from not circular
                if (!lambertAlways) {
                    status = Hohmann(targetMode);
                }
                if (lambertAlways || status != Status.OK) {
                    if (targetMode == TargetMode.SHIP_TO_ORBIT) {
                        LambertToOrbit();
                    } else {
                        if (legacyRetrograde) {
                            status = LambertGE(targetMode);
                        } else {
                            status = LambertGE2(targetMode);
                        }
                    }
                }
            } else {
                // ship to point
                status = LambertToPoint();
            }
            return status;
        }

        public void SetTimeFactor(double tf)
        {
            timeFactor = tf;
        }

        public void SetTimeMode(LambertTimeType timeMode)
        {
            this.timeMode = timeMode;
        }

        public void SetTimeTransfer(double t)
        {
            timeTransfer = t;
        }

        private void Circularize()
        {
            maneuvers.Clear();
            GEManeuver m1 = new GEManeuver();
            m1.t_relative = 0;
            m1.type = ManeuverType.SET_VELOCITY;
            m1.velocityParam = Orbital.VelocityForCircularOrbitRelative(shipState, mu);
            dVtotal = math.length(shipState.r - m1.velocityParam);
            maneuvers.Add(m1);
        }

        private Status Hohmann(TargetMode targetMode)
        {
            Status status = Status.OK;
            // check if both orbits are circular
            if (toOrbit.e > circleThreshold) {
                return Status.DEST_NOT_CIRCULAR;
            }
            if (shipCOE.e > circleThreshold) {
                return Status.SHIP_NOT_CIRCULAR;
            }
            bool rdvs = targetMode == TargetMode.TARGET_RDVS;
            rdvs |= targetMode == TargetMode.TARGET_INTERCEPT;
            HohmannGeneral hg = new HohmannGeneral(shipCOE, toOrbit, rendezvous: rdvs);
            maneuvers = hg.Maneuvers();
            // if this is an intercept, then set the velChange in final maneuver to a do nothing
            if (targetMode == TargetMode.TARGET_INTERCEPT) {
                GEManeuver lastManeuver = maneuvers[maneuvers.Count - 1];
                lastManeuver.type = ManeuverType.APPLY_DV;
                lastManeuver.velocityParam = double3.zero;
                lastManeuver.info = ManeuverInfo.INTERCEPT;
            }
            return status;
        }

        private Status LambertGE(TargetMode targetMode)
        {
            Status status = Status.OK;
            LambertMode lamMode = LambertMode.TO_R2;  // default is for toPoint
            if (targetMode == TargetMode.TARGET_RDVS) {
                lamMode = LambertMode.RENDEZVOUS;
            } else if (targetMode == TargetMode.TARGET_INTERCEPT) {
                lamMode = LambertMode.INTERCEPT;
            }
            // Do a minimum time transfer
            LambertUniversal lambertU;
            if (lamMode == LambertMode.TO_R2) {
                lambertU = new LambertUniversal(shipCOE, toPoint, retrograde: retrogradeTransfer);
            } else {
                lambertU = new LambertUniversal(shipCOE, toOrbit, retrograde: retrogradeTransfer);
            }
            if (checkHit) {
                lambertU.HitPlanetRadius(planetRadius);
            }
            double tXfer = timeTransfer;
            if (timeMode == LambertTimeType.RELATIVE_TO_NOMINAL) {
                tXfer = lambertU.GetMinTime() * timeFactor;
            }
            LambertUniversal.ComputeError luStatus = lambertU.ComputeXfer(tXfer, lamMode, retrograde: retrogradeTransfer);
            // ?? does this match dest orbit ??
            maneuvers = lambertU.Maneuvers();
            if (luStatus == LambertUniversal.ComputeError.IMPACT) {
                status = Status.HIT_PLANET;
            } else if (luStatus == LambertUniversal.ComputeError.XFER_180) {
                // did LambertU have issues with 180 degrees? If so, use LambertBattin
                LambertBattin lambertB = new LambertBattin(shipCOE, toOrbit);
                int lbStatus = lambertB.ComputeXfer(tXfer, lamMode, requirePrograde: !retrogradeTransfer);
                if (lbStatus == LambertBattin.IMPACT) {
                    status = Status.HIT_PLANET;
                }
                maneuvers = lambertB.Maneuvers();
            } else if (luStatus != LambertUniversal.ComputeError.OK) {
                // in extreme G not converged cases, Battin does better
                LambertBattin lambertB = new LambertBattin(shipCOE, toOrbit);
                int lbStatus = lambertB.ComputeXfer(tXfer, lamMode, requirePrograde: !retrogradeTransfer);
                if (lbStatus == LambertBattin.IMPACT) {
                    status = Status.HIT_PLANET;
                }
                maneuvers = lambertB.Maneuvers();
            }

            return status;
        }

        private Status LambertGE2(TargetMode targetMode)
        {
            Status status = Status.OK;
            bool intercept = false;
            if (targetMode == TargetMode.TARGET_INTERCEPT) {
                intercept = true;
            }
            if (toOrbit.mu == 0) {
                status = Status.TO_ORBIT_NO_MU;
                return status;
            }
            (double3 target_r, double3 target_v) = Orbital.COEtoRVRelative(toOrbit);

            double tXfer = timeTransfer;
            if (timeMode == LambertTimeType.RELATIVE_TO_NOMINAL) {
                (double tmin, double tminp, double tminenergy) = Lambert.MinTimesPrograde(mu, shipState.r, target_r, shipState.v, prograde: !retrogradeTransfer);
                tXfer = tminenergy * timeFactor;
            }
            Lambert.LambertOutput lamOutput = Lambert.TransferProgradeToTarget(mu, shipState.r, target_r, shipState.v, target_v, tXfer, prograde: !retrogradeTransfer);
            // When asked for maneuvers will fix up the centerId and propagator
            maneuvers = lamOutput.Maneuvers(centerId: -1, prop: GEPhysicsCore.Propagator.GRAVITY, intercept: intercept);
            if (lamOutput.status == Lambert.Status.HIT_PLANET) {
                status = Status.HIT_PLANET;
            }

            return status;
        }

        private Status LambertToPoint()
        {
            Status status = Status.OK;
            double tXfer = timeTransfer;
            LambertUniversal lambertU = new LambertUniversal(shipCOE, toPoint, retrograde: retrogradeTransfer);
            if (timeMode == LambertTimeType.RELATIVE_TO_NOMINAL) {
                tXfer = lambertU.GetMinTime() * timeFactor;
            }

            if (checkHit) {
                lambertU.HitPlanetRadius(planetRadius);
            }
            LambertUniversal.ComputeError luStatus =
                    lambertU.ComputeXfer(tXfer, LambertMode.TO_R2, retrograde: retrogradeTransfer);

            // ?? does this match dest orbit ??
            maneuvers = lambertU.Maneuvers();
            if (luStatus == LambertUniversal.ComputeError.IMPACT) {
                status = Status.HIT_PLANET;
            }
            // did LambertU have issues with 180 degrees? If so, use LambertBattin
            if (luStatus == LambertUniversal.ComputeError.XFER_180) {
                LambertBattin lambertB = new LambertBattin(shipCOE, toPoint);
                int lbStatus = lambertB.ComputeXfer(tXfer, LambertMode.TO_R2, requirePrograde: !retrogradeTransfer);
                if (lbStatus == LambertBattin.IMPACT) {
                    status = Status.HIT_PLANET;
                }
                maneuvers = lambertB.Maneuvers();
            }

            return status;
        }

        private Status LambertToOrbit()
        {
            // since all we want to do is get to the target orbit do a heuristic scan of the dV for
            // various phases in the target orbit. Don't get too fancy
            double dVmin = double.MaxValue;
            double toPhase = 0.0;
            double steps = 100;
            for (double s = 0.0; s < steps; s++) {
                toOrbit.nu = 2.0 * System.Math.PI * s / steps;
                LambertUniversal lambertU = new LambertUniversal(shipCOE, toOrbit, retrograde: retrogradeTransfer);
                if (checkHit) {
                    lambertU.HitPlanetRadius(planetRadius);
                }
                double timeMin = lambertU.GetMinTime() * timeFactor;
                LambertUniversal.ComputeError luStatus = lambertU.ComputeXfer(timeMin, LambertMode.TO_R2);
                if (luStatus == LambertUniversal.ComputeError.OK) {
                    double dV = lambertU.DeltaV();
                    if (dV < dVmin) {
                        dVmin = dV;
                        toPhase = toOrbit.nu;
                    }
                }
                if (luStatus == LambertUniversal.ComputeError.IMPACT) {
                    return Status.HIT_PLANET;
                }

            }
            // recompute
            toOrbit.nu = toPhase;
            LambertUniversal lambertU2 = new LambertUniversal(shipCOE, toOrbit, retrograde: retrogradeTransfer);
            if (checkHit) {
                lambertU2.HitPlanetRadius(planetRadius);
            }
            double timeMin2 = lambertU2.GetMinTime() * timeFactor;
            LambertUniversal.ComputeError lu2Status = lambertU2.ComputeXfer(timeMin2, LambertMode.TO_R2);
            if (lu2Status == LambertUniversal.ComputeError.IMPACT) {
                return Status.HIT_PLANET;
            }
            maneuvers = lambertU2.Maneuvers();
            return Status.OK;
        }
    }
}
