using System;
using UnityEngine;
using Unity.Mathematics;
using System.Collections.Generic;

namespace GravityEngine2 {
    /// <summary>
    /// Hohmann transfer or rendezvous between any two circular orbits. Handles
    /// diffences in radius, inclination and RAAN (Omega).
    ///
    /// This routine assumes the to & from orbits have been checked and are
    /// cicular within the tolerance desired by the user. e.g. TransferShip does this. 
    /// 
    /// Rendezvous in general requires an intermediate transfer orbit to adjust the
    /// phasing of the chasing ship in order that it arrives at the common node with
    /// the correct phase relationship for rendezvous. This intermediate
    /// orbit is "free" in dV (since the velocity will help it get to the target). 
    /// 
    /// The general case follows Vallado Algorithm 46 (p368) with additional code
    /// to handle the case of outer orbit to inner orbit. 
    /// 
    /// 
    /// </summary>
    public class HohmannGeneral {
        private List<GEManeuver> maneuvers;

        private double INCL_LIMIT = 1E-3;

        public HohmannGeneral(Orbital.COE fromOrbit,
                                Orbital.COE toOrbit,
                                bool rendezvous)
        {
            maneuvers = new List<GEManeuver>();

            bool inclinationChange = Math.Abs(fromOrbit.i - toOrbit.i) > INCL_LIMIT;
            bool sameRadius = Math.Abs(fromOrbit.p - toOrbit.p) / fromOrbit.p < Orbital.SMALL_E;

            if (sameRadius) {
                if (!inclinationChange) {
                    if (rendezvous) {
                        SameOrbitPhasing(fromOrbit, toOrbit);
                    } else {
                        Debug.LogWarning("Already in the required orbit. Nothing to do. ");
                    }
                } else {
                    CircularInclinationAndAN(fromOrbit, toOrbit, rendezvous);
                }
            } else {
                if (!inclinationChange) {
                    CoplanarHohmannVallado(fromOrbit, toOrbit, rendezvous);
                } else {
                    bool innerToOuter = (fromOrbit.a < toOrbit.a);
                    if (innerToOuter)
                        InnerToOuter(fromOrbit, toOrbit, rendezvous);
                    else
                        OuterToInner(fromOrbit, toOrbit, rendezvous);
                }
            }
        }

        public List<GEManeuver> Maneuvers(int centerId = -1)
        {
            if (centerId >= 0) {
                foreach (GEManeuver m in maneuvers)
                    m.centerId = centerId;
            }
            return maneuvers;
        }

        // Easier to follow without all the if(innerToOuter) stuff, so implement two versions for non-coplanar with rdvs.
        // Differ in when the plane change is done (outer orbit since less dV cost)
        // and some differences in phasing calculations. 

        private void InnerToOuter(Orbital.COE fromOrbit,
                                    Orbital.COE toOrbit, bool rendezvous)
        {
            double mu = fromOrbit.mu;
            double w_target = toOrbit.GetAngularVelocity();
            double w_int = fromOrbit.GetAngularVelocity();
            double a_transfer = 0.5f * (fromOrbit.a + toOrbit.a);
            double t_transfer = Math.PI * Math.Sqrt(a_transfer * a_transfer * a_transfer / mu);

            // Debug.LogFormat("int: a={0} w={1}  target: a={2} w={3}", fromOrbit.a, w_int, toOrbit.a, w_target);

            // lead angle required by target
            double alpha_L = w_target * t_transfer;

            double u_initial = 0;
            double u_final = 0;
            double phase_to_node = 0;
            FindClosestNode(fromOrbit, toOrbit, ref u_initial, ref u_final, ref phase_to_node);

            double deltat_node = phase_to_node / w_int;

            // Debug.LogFormat("Node at: {0} (deg), distance to node (deg)={1}", u_initial * GravityMath.RAD2DEG, delta_theta_int * GravityMath.RAD2DEG);

            // Algorithm uses lambda_true. See the definition in Vallado (2-92). Defined for circular equitorial
            // orbits as angle from I-axis (x-axis) to the satellite.
            // This is not the same as u = argument of latitude, which is measured from the ascending node. 

            // Target moves to lambda_tgt1 as interceptor moves to node
            double lambda_tgt1 = toOrbit.nu + w_target * deltat_node;
            // phase lag from interceptor to target (target is 1/2 revolution from u_initial)
            // Vallado uses the fact that node is at omega, which assumes destination orbit is equitorial?
            double lambda_int = u_final + Math.PI; // Math.PI + fromOrbit.omega_uc;
            double theta_new = lambda_int - lambda_tgt1;
            if (theta_new < 0)
                theta_new += 2.0 * Math.PI;
            // This is not working. Why??
            // double alpha_new = Math.PI + theta_new;
            // Keep in 0..2 Pi (my addition)
            double alpha_new = theta_new % (2.0 * Math.PI);

            //Debug.LogFormat("lambda_tgt1={0} theta_new={1} toOrbit.GetCircPhase()={2} t_node={3} w_target={4}",
            //    lambda_tgt1 * GravityMath.RAD2DEG, theta_new * GravityMath.RAD2DEG, toOrbit.GetCircPhase(), deltat_node, w_target);

            // k_target: number of revolutions in transfer orbit. Provided as input
            // k_int: number of revs in phasing orbit. Want to ensure a_phase < a_target to not
            //        waste deltaV.
            double k_target = 0.0;
            double two_pi_k_target = k_target * 2.0 * Math.PI;
            double P_phase = (alpha_new - alpha_L + two_pi_k_target) / w_target;
            while (P_phase < 0) {
                // Debug.Log("Pphase < 0. Bumping k_target");
                k_target += 1.0;
                two_pi_k_target = k_target * 2.0 * Math.PI;
                P_phase = (alpha_new - alpha_L + two_pi_k_target) / w_target;
            }
            double k_int = 1.0;
            double two_pi_k_int = k_int * 2.0 * Math.PI;
            double a_phase = Math.Pow(mu * (P_phase * P_phase / (two_pi_k_int * two_pi_k_int)), 1.0 / 3.0);
            //Debug.LogFormat("alpha_new={0} alpha_L={1} Pphase={2}", alpha_new * GravityMath.RAD2DEG, alpha_L * GravityMath.RAD2DEG, P_phase);

            int loopCnt = 0;
            while (rendezvous && ((a_phase < fromOrbit.a) || (a_phase > toOrbit.a))) {
                if (a_phase < fromOrbit.a) {
                    // Debug.Log("Adjust: a_phase < toOrbit - add a target lap. a_phase=" + a_phase);
                    k_target += 1.0;
                    two_pi_k_target = k_target * 2.0 * Math.PI;
                } else if (a_phase > toOrbit.a) {
                    // Debug.Log("Adjust: a_phase > fromOrbit - add a phase lap. a_phase=" + a_phase);
                    k_int += 1.0;
                    two_pi_k_int = k_int * 2.0 * Math.PI;
                }
                P_phase = (alpha_new - alpha_L + two_pi_k_target) / w_target;
                a_phase = Math.Pow(mu * (P_phase * P_phase / (two_pi_k_int * two_pi_k_int)), 1.0 / 3.0);
                //Debug.LogFormat("alpha_new={0} alpha_L={1} Pphase={2} a_phase={3} k_int={4} k_tgt={5}",
                //    alpha_new * GravityMath.RAD2DEG, alpha_L * GravityMath.RAD2DEG, P_phase, a_phase, k_int, k_target);
                if (loopCnt++ > 10) {
                    Debug.LogWarning("Failed to find transfer. Rendezvous phasing issue. ");
                    return;
                }
            }

            double t_phase = k_int * 2.0 * Math.PI * Math.Sqrt(a_phase * a_phase * a_phase / mu);

            double deltaV_phase = 0;
            double deltaV_trans1, deltaV_trans2;
            double v_trans1_rel = Math.Sqrt(2.0 * mu / fromOrbit.a - mu / a_transfer);


            double delta_incl = (toOrbit.i - fromOrbit.i) * GravityMath.DEG2RAD;
            if (rendezvous) {
                if (fromOrbit.a > 2.0 * a_phase) {
                    Debug.LogError("Phasing orbit is not an ellipse. Cannot proceed. a_phase=" + a_phase);
                    return;
                }
                deltaV_phase = Math.Sqrt(2.0 * mu / fromOrbit.a - mu / a_phase)
                                                    - Math.Sqrt(mu / fromOrbit.a);
                deltaV_trans1 = Math.Sqrt(2.0 * mu / fromOrbit.a - mu / a_transfer)
                                                    - Math.Sqrt(2.0 * mu / fromOrbit.a - mu / a_phase);
            } else {
                deltaV_trans1 = Math.Sqrt(2.0 * mu / fromOrbit.a - mu / a_transfer)
                                                    - Math.Sqrt(mu / fromOrbit.a);
            }
            // Note: deltaV_trans2 value is just for the accounting of dV. Manuever does a setv to 
            // ensure correct orientation. 
            deltaV_trans2 = Math.Sqrt(2.0 * mu / toOrbit.a - mu / a_transfer
                                                + mu / toOrbit.a
                                                - 2.0 * Math.Sqrt(2.0 * mu / toOrbit.a - mu / a_transfer)
                                                    * Math.Sqrt(mu / toOrbit.a) * Math.Cos(delta_incl));

            //Debug.LogFormat("T1: a_int={0} a_phase={1} a_tgt={2} dt={3} dV_phase={4} dv1={5} dv2={6}",
            //    fromOrbit.a, a_phase, toOrbit.a,
            //    deltat_node, deltaV_phase, deltaV_trans1, deltaV_trans2);

            double3 xferStart = Orbital.PositionForCOE(fromOrbit, u_initial);
            double3 h_unit = Orbital.OrbitAxis(fromOrbit);
            if (rendezvous) {
                // phasing burn: in same plane as the from orbit at the node
                double t = deltat_node;
                double3 v1 = new double3(deltaV_phase, 0, 0);
                GEManeuver m_phase = new GEManeuver(t, v1, ManeuverType.SET_VELOCITY) {
                    info = ManeuverInfo.HG_IO_PHASE,
                    // Patch fields
                    r_relative = xferStart
                };
                m_phase.v_relative = math.normalize(math.cross(h_unit, m_phase.r_relative));
                m_phase.dV = m_phase.v_relative * deltaV_phase;
                m_phase.v_relative *= Math.Sqrt(mu / fromOrbit.a) + deltaV_phase;
                m_phase.hasRelativeRV = true;
                m_phase.velocityParam = m_phase.v_relative;
                maneuvers.Add(m_phase);

            } else {
                t_phase = 0;
            }

            // transfer burn - stay in initial orbit plane
            double3 v2 = new double3(deltaV_trans1, 0, 0);
            double t1 = deltat_node + t_phase;
            GEManeuver m_trans1 = new GEManeuver(t1, v2, ManeuverType.SET_VELOCITY) {
                info = ManeuverInfo.HG_IO_X1,
                r_relative = xferStart
            };
            m_trans1.v_relative = math.normalize(math.cross(h_unit, m_trans1.r_relative));
            m_trans1.dV = m_trans1.v_relative * deltaV_trans1;
            m_trans1.v_relative *= v_trans1_rel;
            m_trans1.hasRelativeRV = true;
            m_trans1.velocityParam = m_trans1.v_relative;
            maneuvers.Add(m_trans1);

            // Arrival burn - do plane change here (just assign the correct velocity)
            // (Need to reverse this when from outer to inner...do plane change at start)
            double finalPhase = u_final + Math.PI;
            double3 finalV = Orbital.VelocityForCOE(toOrbit, finalPhase);
            // double3 finalPos = Orbital.PositionForCOE(toOrbit, finalPhase);
            t1 = deltat_node + t_phase + t_transfer;
            GEManeuver m_trans2 = new GEManeuver(t1, finalV, ManeuverType.SET_VELOCITY) {
                info = ManeuverInfo.HG_IO_X2,
            };

            h_unit = Orbital.OrbitAxis(toOrbit);
            m_trans2.r_relative = Orbital.PositionForCOE(toOrbit, finalPhase);
            m_trans2.v_relative = math.normalize(math.cross(h_unit, m_trans2.r_relative));
            m_trans2.v_relative *= Math.Sqrt(mu / toOrbit.a);
            m_trans2.hasRelativeRV = true;

            // Determine xfer arrival velocity to determine dV, dVvector
            KeplerPropagator.RVT kRVT = new KeplerPropagator.RVT(m_trans1.r_relative, m_trans1.v_relative, t0: 0, toOrbit.mu);
            (int status, double3 r_xfer, double3 v3d_xfer) = KeplerPropagator.RVforTime(kRVT, t_transfer);
            m_trans2.dV = m_trans2.v_relative - v3d_xfer;
            maneuvers.Add(m_trans2);
        }

        private void OuterToInner(Orbital.COE fromOrbit,
                                    Orbital.COE toOrbit,
                                    bool rendezvous)
        {
            double mu = fromOrbit.mu;
            double w_target = toOrbit.GetAngularVelocity();
            double w_int = fromOrbit.GetAngularVelocity();
            double a_transfer = 0.5f * (fromOrbit.a + toOrbit.a);
            double t_transfer = Math.PI * Math.Sqrt(a_transfer * a_transfer * a_transfer / mu);

            //Debug.LogFormat("int: a={0} w={1}  target: a={2} w={3} a_transfer={4}",
            //    fromOrbit.a, w_int, toOrbit.a, w_target, a_transfer);

            // lead angle required by target
            double alpha_L = w_target * t_transfer;

            double u_initial = 0;
            double u_final = 0;
            double phase_to_node = 0;
            // u_initial is the phase in the from orbit of the common point (u_initial = nu + omega_lc, but for circle omega_lc=0)
            // u_final is the phase in the to orbit of the common point. 
            FindClosestNode(fromOrbit, toOrbit, ref u_initial, ref u_final, ref phase_to_node);
            double deltat_node = phase_to_node / w_int;

            double t_phase = 0;
            double a_phase = 0;
            double deltaV_trans1 = 0;
            double3 xferStart = Orbital.PositionForCOE(fromOrbit, u_initial);

            if (rendezvous) {

                //Debug.LogFormat("Node at: {0} rad/{1} (deg), distance to node (deg)={2}", u_initial, u_initial * GravityMath.RAD2DEG, delta_theta_int * GravityMath.RAD2DEG);

                // Algorithm uses lambda_true. See the definition in Vallado (2-92). Defined for circular equitorial
                // orbits as angle from I-axis (x-axis) to the satellite.
                // This is not the same as u = argument of latitude, which is measured from the ascending node. 

                // Target moves to lambda_tgt1 as interceptor moves to node
                double lambda_tgt1 = toOrbit.nu + w_target * deltat_node;
                // phase lag from interceptor to target (target is 1/2 revolution from u_initial)
                // Vallado uses the fact that node is at omega, which assumes destination orbit is equitorial?
                double lambda_int = u_final + Math.PI; // Math.PI + fromOrbit.omega_uc;
                double theta_new = lambda_int - lambda_tgt1;
                if (theta_new < 0)
                    theta_new += 2.0 * Math.PI;
                // This is not working. Why??
                // double alpha_new = Math.PI + theta_new;
                double alpha_new = theta_new;

                //Debug.LogFormat("lambda_tgt1={0} theta_new={1} toOrbit.GetCircPhase()={2} t_node={3} w_target={4}",
                //    lambda_tgt1 * GravityMath.RAD2DEG, theta_new * GravityMath.RAD2DEG, toOrbit.GetCircPhase(), deltat_node, w_target);

                // k_target: number of revolutions in transfer orbit. 
                // k_int: number of revs in phasing orbit. Want to ensure a_phase > a_target to not
                //        waste deltaV (outer to inner).
                double k_target = 0.0;
                double two_pi_k_target = k_target * 2.0 * Math.PI;
                double P_phase = (alpha_new - alpha_L + two_pi_k_target) / w_target;
                while (P_phase < 0) {
                    //Debug.Log("Pphase < 0. Bumping k_target");
                    k_target += 1.0;
                    two_pi_k_target = k_target * 2.0 * Math.PI;
                    P_phase = (alpha_new - alpha_L + two_pi_k_target) / w_target;
                }
                double k_int = 1.0;
                double two_pi_k_int = k_int * 2.0 * Math.PI;
                a_phase = Math.Pow(mu * (P_phase * P_phase / (two_pi_k_int * two_pi_k_int)), 1.0 / 3.0);
                //Debug.LogFormat("alpha_new={0} alpha_L={1} Pphase={2} a_phase={3}",
                //    alpha_new * GravityMath.RAD2DEG, alpha_L * GravityMath.RAD2DEG, P_phase, a_phase);


                // For outer to inner modify both target and phase orbits
                int loopCnt = 0;
                while (rendezvous && ((a_phase < toOrbit.a) || (a_phase > fromOrbit.a))) {
                    if (a_phase < toOrbit.a) {
                        //Debug.Log("Adjust: a_phase < toOrbit - add a target lap. a_phase=" + a_phase);
                        k_target += 1.0;
                        two_pi_k_target = k_target * 2.0 * Math.PI;
                    } else if (a_phase > fromOrbit.a) {
                        //Debug.Log("Adjust: a_phase > fromOrbit - add a phase lap. a_phase=" + a_phase);
                        k_int += 1.0;
                        two_pi_k_int = k_int * 2.0 * Math.PI;
                    }
                    P_phase = (alpha_new - alpha_L + two_pi_k_target) / w_target;
                    a_phase = Math.Pow(mu * (P_phase * P_phase / (two_pi_k_int * two_pi_k_int)), 1.0 / 3.0);
                    //Debug.LogFormat("alpha_new={0} alpha_L={1} Pphase={2} a_phase={3} k_int={4} k_tgt={5}",
                    //    alpha_new * GravityMath.RAD2DEG, alpha_L * GravityMath.RAD2DEG, P_phase, a_phase, k_int, k_target);
                    if (loopCnt++ > 10) {
                        Debug.LogWarning("Failed to find transfer. Rendezvous phasing issue. ");
                        return;
                    }
                }

                t_phase = k_int * 2.0 * Math.PI * Math.Sqrt(a_phase * a_phase * a_phase / mu);

                if (fromOrbit.a > 2.0 * a_phase) {
                    Debug.LogError("Phasing orbit is not an ellipse. Cannot proceed. a_phase=" + a_phase);
                    return;
                }
                deltaV_trans1 = Math.Sqrt(2.0 * mu / fromOrbit.a - mu / a_transfer)
                                                    - Math.Sqrt(2.0 * mu / fromOrbit.a - mu / a_phase);
            }

            GEManeuver m_phase = new GEManeuver {
                info = ManeuverInfo.HG_OI_PHASE,
                hasRelativeRV = true,
            };
            GEManeuver m_trans1 = new GEManeuver {
                info = ManeuverInfo.HG_OI_X1,
                hasRelativeRV = true,
            };

            if (rendezvous) {
                // Debug.LogFormat("ecc={0} a={1}", xferData.ecc, xferData.a);
                // phasing burn: In outer to inner do the plane change here, less dV
                m_phase.type = ManeuverType.SET_VELOCITY;
                // uFinal is node location since xferData is a copy of toOrbit BUT want the velocity of apoapsis
                // (this phase is peri)
                double vxfer_mag = Math.Sqrt(2.0 * mu / fromOrbit.a - mu / a_phase);
                m_phase.velocityParam = vxfer_mag * math.normalize(Orbital.VelocityForCOE(toOrbit, u_final));
                // m_phase.dV = (m_phase.velChange - fromOrbit.GetPhysicsVelocityForEllipse(u_finalDeg)).magnitude;
                m_phase.dV = m_phase.velocityParam - Orbital.VelocityForCOE(fromOrbit, u_initial);
                m_phase.t_relative = deltat_node;
                //m_phase.physPosition = xferStart;
                //// KeplerSeq fields
                //m_phase.physPosition = xferStart;
                m_phase.r_relative = xferStart; // - ge.GetPositionDoubleV3(fromOrbit.centralMass);
                m_phase.v_relative = m_phase.velocityParam;
                //m_phase.dVvector = new Vector3d(m_phase.velChange - fromOrbit.GetPhysicsVelocityForEllipse(u_initialDeg));
                maneuvers.Add(m_phase);

                // transfer burn - phasing orbit did plane change
                m_trans1.type = ManeuverType.SET_VELOCITY;
                // m_trans1.dV = deltaV_trans1;
                m_trans1.t_relative = deltat_node + t_phase;
                //// KeplerSeq fields
                m_trans1.r_relative = m_phase.r_relative;
                m_trans1.v_relative = m_phase.velocityParam + deltaV_trans1 * math.normalize(m_phase.velocityParam);
                m_trans1.velocityParam = m_trans1.v_relative;
                m_trans1.hasRelativeRV = true;

                //// Determine xfer arrival velocity to determine dV, dVvector
                KeplerPropagator.RVT kRVT2 = new KeplerPropagator.RVT(m_phase.r_relative, m_phase.v_relative, t0: 0, toOrbit.mu);
                (int status2, double3 r1_xfer, double3 v1_xfer) = KeplerPropagator.RVforTime(kRVT2, t_phase);
                m_trans1.dV = m_trans1.v_relative - v1_xfer;
                maneuvers.Add(m_trans1);
            } else {
                // no rendezvous - direct xfer
                t_phase = 0;
                // transfer burn - do inclination change here
                m_trans1.type = ManeuverType.SET_VELOCITY;
                // want velocity at apoapsis (since we are the farthest point from center)
                double v1_xfer = Math.Sqrt(2.0 * mu / fromOrbit.a - mu / a_transfer);
                m_trans1.velocityParam = v1_xfer * math.normalize(Orbital.VelocityForCOE(toOrbit, u_final));
                // m_trans1.dV = (m_trans1.velChange - fromOrbit.GetPhysicsVelocityForEllipse(u_finalDeg)).magnitude;
                m_trans1.dV = m_trans1.velocityParam - Orbital.VelocityForCOE(fromOrbit, u_initial);
                m_trans1.t_relative = deltat_node;
                //// KeplerSeq fields
                m_trans1.r_relative = xferStart; // - ge.GetPositionDoubleV3(fromOrbit.centralMass);
                m_trans1.v_relative = m_trans1.velocityParam;
                m_trans1.hasRelativeRV = true;
                maneuvers.Add(m_trans1);
            }

            // Arrival burn
            double finalPhase = (u_final + Math.PI);
            double3 finalV = Orbital.VelocityForCOE(toOrbit, finalPhase);
            // Vector3 finalPos = toOrbit.GetPhysicsPositionforEllipse(finalPhase);
            GEManeuver m_trans2 = new GEManeuver {
                info = ManeuverInfo.HG_OI_X2,
                type = ManeuverType.SET_VELOCITY,
                velocityParam = finalV,
                t_relative = deltat_node + t_phase + t_transfer
            };

            // Determine xfer arrival velocity to determine dV, dVvector
            //OrbitPropagator orbitProp = new OrbitPropagator(m_trans1.relativePos, m_trans1.relativeVel, 0, toOrbit.mu);
            KeplerPropagator.RVT kRVT = new KeplerPropagator.RVT(m_trans1.r_relative, m_trans1.v_relative, t0: 0, toOrbit.mu);
            //(Vector3d r_xfer, Vector3d v3d_xfer) = orbitProp.PropagateToTime(t_transfer);
            (int status, double3 r_xfer, double3 v_xfer) = KeplerPropagator.RVforTime(kRVT, t_transfer);
            m_trans2.dV = finalV - v_xfer;

            //// maneuver positions and info for KeplerSeq conversion and velocity directions
            // use true phase when getting pos/vel.
            double3 h_unit = Orbital.OrbitAxis(toOrbit);
            m_trans2.r_relative = Orbital.PositionForCOE(toOrbit, finalPhase);
            m_trans2.v_relative = math.normalize(math.cross(h_unit, m_trans2.r_relative));
            m_trans2.v_relative *= Math.Sqrt(mu / toOrbit.a);
            m_trans2.hasRelativeRV = true;
            maneuvers.Add(m_trans2);
        }

        private void CoplanarHohmannVallado(Orbital.COE fromOrbit,
                                            Orbital.COE toOrbit,
                                            bool rendezvous)
        {
            double mu = fromOrbit.mu;
            double a_target = toOrbit.a;
            double a_int = fromOrbit.a;
            bool innerToOuter = (a_target > a_int);
            double a_transfer = 0.5 * (a_int + a_target);
            double t_transfer = Math.PI * Math.Sqrt((a_transfer * a_transfer * a_transfer) / mu);
            double t_wait = 0;
            if (rendezvous) {
                // determine the phasing required
                double w_target = Math.Sqrt(mu / (a_target * a_target * a_target));
                double w_int = Math.Sqrt(mu / (a_int * a_int * a_int));
                double alpha_L = w_target * t_transfer;
                // theta is required seperation (target - interceptor)
                double theta = alpha_L - Math.PI;
                double w_delta = w_int - w_target;
                double phaseDelta = GravityMath.AngleDeltaRadianCW(toOrbit.GetCircPhase(), fromOrbit.GetCircPhase());
                // Debug.LogFormat("phase delta={0} to={1} from={2}", phaseDelta, toOrbit.GetCircPhase(), fromOrbit.GetCircPhase());
                if (!innerToOuter) {
                    // target is moving away during xfer
                    theta = Math.PI - alpha_L;
                    w_delta *= -1.0;
                    phaseDelta = GravityMath.AngleDeltaRadianCW(fromOrbit.GetCircPhase(), toOrbit.GetCircPhase());
                }
                double KLIMIT = 10.0;
                double k = 0.0;
                while ((t_wait <= 0) && (k < KLIMIT)) {
                    t_wait = (theta - phaseDelta + 2.0 * Math.PI * k) / w_delta;
                    k += 1.0;
                }
                if (k >= KLIMIT) {
                    Debug.LogError("Transfer required too many orbits");
                    return;
                }
            }

            double dV1 = Math.Sqrt(2.0 * mu / a_int - mu / a_transfer) - Math.Sqrt(mu / a_int);

            // maneuver positions and info for KeplerSeq conversion and velocity directions. Uses fact that 
            // Hohmann velocity will be h x r
            double3 h_unit = Orbital.OrbitAxis(fromOrbit);
            // use true phase when getting pos/vel.
            double phaseAtXfer = fromOrbit.nu + 2.0 * Math.PI / fromOrbit.GetPeriod() * t_wait;
            // Create two maneuvers for xfer
            GEManeuver m1 = new GEManeuver {
                type = ManeuverType.SET_VELOCITY,
                velocityParam = new double3(dV1, 0, 0),
                t_relative = t_wait,
                info = ManeuverInfo.HG_CHV_1,
                hasRelativeRV = true,
                r_relative = Orbital.PositionForCOE(fromOrbit, phaseAtXfer),
            };
            m1.v_relative = math.normalize(math.cross(h_unit, m1.r_relative));
            m1.v_relative *= Math.Sqrt(mu / fromOrbit.a) + dV1;
            m1.velocityParam = m1.v_relative;
            maneuvers.Add(m1);

            double dV2 = Math.Sqrt(mu / a_target) - Math.Sqrt(2.0 * mu / a_target - mu / a_transfer);

            GEManeuver m2 = new GEManeuver {
                type = ManeuverType.SET_VELOCITY,
                // dV = dV2,
                velocityParam = new double3(dV2, 0, 0),
                t_relative = t_wait + t_transfer,
                info = ManeuverInfo.HG_CHV_2
            };
            //// adjust for omegas (ideally should be zero if circular, but if near circular will get some)
            phaseAtXfer = (phaseAtXfer + math.PI - toOrbit.omegaU - toOrbit.omegaL) % (2.0 * Math.PI);
            m2.r_relative = Orbital.PositionForCOE(toOrbit, phaseAtXfer);
            m2.v_relative = math.normalize(math.cross(h_unit, m2.r_relative));
            m2.v_relative *= Math.Sqrt(mu / toOrbit.a);
            m2.velocityParam = m2.v_relative;
            m2.hasRelativeRV = true;
            maneuvers.Add(m2);

            //// Determine the relative velocities
            m1.dV = math.normalize(m1.v_relative) * dV1;

            m2.dV = math.normalize(m2.v_relative) * dV2;
            //// Determine xfer arrival velocity to determine dV, dVvector
            //OrbitPropagator orbitProp = new OrbitPropagator(m1.relativePos, m1.relativeVel, 0, toOrbit.mu);
            //(Vector3d r_xfer, Vector3d v3d_xfer) = orbitProp.PropagateToTime(t_transfer);
            //m2.dVvector = m2.relativeVel - v3d_xfer;

        }

        /// <summary>
        /// Rendezvous two ships in the same orbit. 
        /// 
        /// IO.X2
        /// </summary>
        /// <param name="fromOrbit"></param>
        /// <param name="toOrbit"></param>
        /// <param name="rendezvous"></param>
        private void SameOrbitPhasing(Orbital.COE fromOrbit, Orbital.COE toOrbit)
        {
            double mu = fromOrbit.mu;
            double a = fromOrbit.a;
            double w_target = Math.Sqrt(mu / (a * a * a));

            double theta = GravityMath.AngleDeltaRadianCW(fromOrbit.GetCircPhase(), toOrbit.GetCircPhase());
            // Debug.LogFormat("Theta={0} \nfrom={1} \nto={2}", theta * GravityMath.RAD2DEG, fromOrbit.LogString(), toOrbit.LogString() );
            double t_phase = 0;
            double a_phase = 0;
            double k_target = 1.0;
            double k_int = 1.0;
            int loopCnt = 0;
            const int LOOP_LIMIT = 20;
            // target leads, want to do one (or more) orbits minus phase lead
            t_phase = (2.0 * Math.PI * k_target - theta) / w_target;
            if (theta > Math.PI) {
                theta = 2.0 * Math.PI - theta;
                t_phase = (2.0 * Math.PI * k_target + theta) / w_target;
            }

            while (((a_phase > 2.0 * a) || (a_phase < 0.5 * a)) && loopCnt < LOOP_LIMIT) {
                double two_pi_k_int = k_int * 2.0 * Math.PI;
                a_phase = Math.Pow(mu * (t_phase * t_phase) / (two_pi_k_int * two_pi_k_int), 1.0 / 3.0);
                if (a_phase > 3.0 * a) {
                    k_int += 1.0;
                } else if (a_phase < 0.5 * a) {
                    k_target += 1.0;
                }
                loopCnt++;
            }
            if (loopCnt >= LOOP_LIMIT) {
                Debug.LogError("Loop count exceeded");
            }
            double t_transfer = 2.0 * Math.PI * Math.Sqrt(a_phase * a_phase * a_phase / mu);


            double dV1 = Math.Sqrt(2.0 * mu / a - mu / a_phase) - Math.Sqrt(mu / a);

            // Debug.LogFormat("a_phase={0} t_phase={1} k_int={2} k_tgt={3} dv1={4}", a_phase, t_transfer, k_int, k_target, dV1);

            // Create two maneuvers for xfer
            GEManeuver m1 = new GEManeuver {
                info = ManeuverInfo.HG_SOP_1,
                type = ManeuverType.SET_VELOCITY,
                // dV = dV1,
                velocityParam = new double3(dV1, 0, 0),
                t_relative = 0
            };
            //m1.nbody = fromOrbit.nbody;
            // maneuver positions and info for KeplerSeq conversion and velocity directions
            double3 h_unit = Orbital.OrbitAxis(fromOrbit);
            //// do NOT use GetCircPhase(), since the PhysicsPosition will apply the omega
            //m1.physPosition = new Vector3d(fromOrbit.GetPhysicsPositionforEllipse(phaseNow));
            m1.r_relative = Orbital.PositionForCOE(fromOrbit, fromOrbit.nu);
            m1.v_relative = math.normalize(math.cross(h_unit, m1.r_relative));
            m1.v_relative *= Math.Sqrt(2.0 * mu / a - mu / a_phase);
            m1.velocityParam = m1.v_relative;
            m1.hasRelativeRV = true;
            maneuvers.Add(m1);

            double dV2 = Math.Sqrt(mu / a) - Math.Sqrt(2.0 * mu / a - mu / a_phase);

            GEManeuver m2 = new GEManeuver {
                info = ManeuverInfo.HG_SOP_2,
                type = ManeuverType.SET_VELOCITY,
                // dV = dV2,
                velocityParam = new double3(dV2, 0, 0),
                t_relative = t_transfer
            };
            //m2.nbody = fromOrbit.nbody;
            double phaseAtXfer = toOrbit.nu + (2.0 * Math.PI / toOrbit.GetPeriod()) * t_transfer;
            //m2.physPosition = new Vector3d(toOrbit.GetPhysicsPositionforEllipse(phaseAtXfer));
            m2.r_relative = Orbital.PositionForCOE(toOrbit, phaseAtXfer);
            m2.v_relative = math.normalize(math.cross(h_unit, m2.r_relative));
            m2.v_relative *= Math.Sqrt(mu / toOrbit.a);
            m2.hasRelativeRV = true;
            m2.velocityParam = m2.v_relative;
            maneuvers.Add(m2);

            //m1.relativeTo = fromOrbit.centralMass;
            m1.dV = math.normalize(m1.v_relative) * dV1;
            //m2.relativeTo = fromOrbit.centralMass;
            m2.dV = math.normalize(m2.v_relative) * dV2;
        }



        /// <summary>
        /// Determine the closest node of the fromOrbit with respect to the to orbit i.e. the point at which the planes of the orbits cross. 
        /// The result is the phase from the current position in the from orbit to the nearest node. 
        /// </summary>
        /// <param name="fromData">Orbit data of fromOrbit, including phase</param>
        /// <param name="toData">Orbit data of to orbit (including phase)</param>
        /// <param name="u_initial">absolute phase angle of nearest node in fromOrbit (radians)</param>
        /// <param name="u_final">absolute phase angle to nearest node in toOrbit (radians)</param>
        /// <param name="phase_to_node">Phase delta from current position in from orbit to node (radians)</param>
        public static void FindClosestNode(Orbital.COE fromData,
                                            Orbital.COE toData,
                                            ref double u_initial,
                                            ref double u_final,
                                            ref double phase_to_node)
        {
            double3 z_unit = new double3(0, 0, 1);
            // 1) Find the line of intersection of the orbit planes. 
            // - normal is just the rotation of the Z_unit vector
            double3 n_initial = Orbital.RotateToOrbitFrame(fromData, z_unit);
            double3 n_final = Orbital.RotateToOrbitFrame(toData, z_unit);
            double3 lofn = math.normalize(math.cross(n_initial, n_final));

            // 2) Project the line of nodes onto each orbital plane and determine it's angle to the local X_unit direction
            u_initial = Orbital.PhaseAngleRadiansForDirection(lofn, fromData);
            u_final = Orbital.PhaseAngleRadiansForDirection(lofn, toData);

            // Need to see if they are in the same hemisphere...
            double3 initialXY = new(Math.Cos(u_initial), Math.Sin(u_initial), 0);
            double3 finalXY = new(Math.Cos(u_final), Math.Sin(u_final), 0);

            double3 initialNode = Orbital.RotateToOrbitFrame(fromData, initialXY);
            double3 finalNode = Orbital.RotateToOrbitFrame(toData, finalXY);
            if (math.dot(initialNode, finalNode) < 0) {
                u_final = (u_final + Math.PI) % (2.0 * Math.PI);
            }

            // how far is fromOrbit from a node? 
            double fromPhase = fromData.GetCircPhase() - fromData.omegaU;
            phase_to_node = GravityMath.AngleDeltaRadianCW(fromPhase, u_initial);
            // if node is more than a half-rev away, use the opposing node
            if (phase_to_node > Math.PI) {
                phase_to_node -= Math.PI;
                u_initial = (u_initial + Math.PI) % (2.0 * Math.PI);
                u_final = (u_final + Math.PI) % (2.0 * Math.PI);
            }
            // Debug.LogFormat("   FromOmega:{0} ToOmega:{1} LofN: {2} (deg) u_initial={3} (deg) u_final={4} (deg) dPhase={5} (deg) ",
            //    fromData.omegaU, toData.omegaU,
            //    lofn, u_initial * GravityMath.RAD2DEG, 
            //    u_final * GravityMath.RAD2DEG, 
            //    phase_to_node * GravityMath.RAD2DEG);
            // Debug.LogFormat("   iNode={0} fNode={1} iDotf={2}", initialNode, finalNode, math.dot(initialNode, finalNode));
        }

        public void CircularInclinationAndAN(Orbital.COE fromOrbit, Orbital.COE toOrbit, bool rendezvous)
        {
            double mu = fromOrbit.mu;
            // check the orbits are circular and have the same radius
            if (fromOrbit.e > Orbital.SMALL_E) {
                Debug.LogWarning("fromOrbit is not circular. ecc=" + fromOrbit.e);
                return;
            }
            if (toOrbit.e > Orbital.SMALL_E) {
                Debug.LogWarning("toOrbit is not circular. ecc=" + toOrbit.e);
                return;
            }
            if (Math.Abs(fromOrbit.a - toOrbit.a) > Orbital.SMALL_E) {
                Debug.LogWarning("Orbits do not have the same radius delta=" + Math.Abs(fromOrbit.a - toOrbit.a));
                return;
            }
            double u_initial = 0;
            double u_final = 0;
            // not used in same radius case.
            double phase_to_node = 0;
            FindClosestNode(fromOrbit, toOrbit, ref u_initial, ref u_final, ref phase_to_node);

            // Code uses OmegaU already, do not use GetCircPhase()
            double time_to_crossing = fromOrbit.GetPeriod() * GravityMath.AngleDeltaRadianCW(fromOrbit.nu, u_initial) / (2.0 * Math.PI);

            // Determine velocity change required
            double3 dV = Orbital.VelocityForCOE(toOrbit, u_final) -
                            Orbital.VelocityForCOE(fromOrbit, u_initial);

            // Create a maneuver object
            GEManeuver m = new GEManeuver();
            m.info = ManeuverInfo.HG_CIAN_1;
            //m.physPosition = new Vector3d(fromOrbit.GetPhysicsPositionforEllipse((float)(u_initialDeg)));
            m.type = ManeuverType.SET_VELOCITY;
            m.dV = dV;
            m.velocityParam = Orbital.VelocityForCOE(toOrbit, u_final);
            m.t_relative = time_to_crossing;
            //m.nbody = fromOrbit.nbody;
            m.r_relative = Orbital.PositionForCOE(fromOrbit, u_initial);
            m.v_relative = m.velocityParam;
            m.hasRelativeRV = true;
            //m.dVvector = new Vector3d(dV);
            maneuvers.Add(m);

            if (rendezvous) {
                // Can do the co-planar phasing at any time. Apply a small time_offset amount to ensure the plane change
                // and sync maneuver are well separated
                double a = toOrbit.a;
                double w_to = Math.Sqrt(mu / (a * a * a));
                // make copies toOrbit and update phases as required
                Orbital.COE fromCopy = new Orbital.COE(toOrbit);
                Orbital.COE toCopy = new Orbital.COE(toOrbit);
                // advance phases 
                fromCopy.nu = u_final;
                toCopy.nu += (w_to * time_to_crossing);
                // Same orbit phasing will add two maneuvers
                SameOrbitPhasing(fromCopy, toCopy);
                //
                // Need to offset the SOP to when the AN is reached
                // HACK: Time offset is to keep correct sequencing of maneuvers. It will cause a small
                // position velocity error
                const double time_offset = 1E-3;
                // struct, so we are copying here...
                GEManeuver geManeuver = maneuvers[1];
                geManeuver.t_relative += time_to_crossing + time_offset;
                maneuvers[1] = geManeuver;
                geManeuver = maneuvers[2];
                geManeuver.t_relative += time_to_crossing + time_offset;
                maneuvers[2] = geManeuver;
                // TODO: Need to adjust physPos/physVel at maneuver time
            }
        }


    }
}
