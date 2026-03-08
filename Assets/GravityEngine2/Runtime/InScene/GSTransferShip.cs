using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace GravityEngine2 {
    /// <summary>
    /// Scene wrapper for the @see TransferShip class. This handles the configuration and invocation 
    /// of a transfer using inspector entries managed by an Editor script. 
    /// 
    /// </summary>
    public class GSTransferShip : MonoBehaviour {
        // Has custom editor script
        public GSController gsController;
        public GSBody ship;
        public GSBody centerBody;

        public TransferShip.TargetMode targetMode = TransferShip.TargetMode.SHIP_TO_ORBIT;
        public bool lambertAlways = false;

        // params (usage depends on targetMode)
        public GSDisplayOrbit targetOrbit;
        public GSBody targetBody;
        public double3 targetPoint;

        /// <summary>
        /// For Lambert transfers the time of the transfer needs to be specified. The most
        /// "scale friendly" way to do this is to use a time factor which is a multiplier on the
        /// nominal minimal energy Lambert transfer. 
        /// 
        /// If the explicit world time fo the transfer is to be specified this is also supported. 
        /// 
        /// The result is a modal choice for Lambert transfers.
        /// </summary>


        public TransferShip.LambertTimeType lambertTimeMode = TransferShip.LambertTimeType.RELATIVE_TO_NOMINAL;
        public double timeFactor = 1.0;

        public double timeTransfer = 10.0;

        public double circleThreshold = Orbital.SMALL_E;

        // Hit detect: check versus physical radius of center body
        public bool checkHit = false;

        public bool retrograde = false;

        public bool legacyRetrograde = false;

        // UI
        public bool keysEnabled = false;
        public bool transferAtStart = false;

        private List<GEManeuver> maneuvers;

        private void Start()
        {
            if (transferAtStart) {
                gsController.ControllerStartedCallbackAdd(ControllerStart);
            }
        }

        private void ControllerStart(GSController gsc)
        {
            GECore ge = gsc.GECore();
            ge.PhyLoopCompleteCallbackAdd(XferShip);
        }

        // arg unused (but required by callback API)
        private void ComputeTransferManeuvers(GECore ge, object args)
        {
            double mu = ge.MuWorld(targetOrbit.centerDisplayBody.gsBody.Id());
            GEBodyState shipState = new GEBodyState();
            int shipId = ship.Id();
            bool ok = ge.StateByIdRelative(shipId, centerBody.Id(), ref shipState);
            Orbital.COE targetCOE;
            if ((targetMode == TransferShip.TargetMode.TARGET_RDVS) ||
                (targetMode == TransferShip.TargetMode.TARGET_INTERCEPT) ||
                (targetMode == TransferShip.TargetMode.TARGET_ORBIT)) {
                // from GE so already scaled
                targetCOE = ge.COE(targetBody.Id(), centerBody.Id());
            } else if (targetOrbit != null) {
                targetCOE = targetOrbit.LastCOE();
                targetCOE.mu = mu;
            } else {
                Debug.LogWarning("No target COE defined");
                return;
            }
            if (ok) {
                double planetRadius = centerBody.radius *
                        GBUnits.DistanceConversion(centerBody.bodyInitData.units, gsController.defaultUnits);
                TransferShip transferShip = new TransferShip(shipState, targetCOE, mu,
                                                circleThreshold,
                                                checkHit: checkHit,
                                                planetRadius: planetRadius,
                                                retrograde: retrograde,
                                                legacyRetrograde: legacyRetrograde);
                transferShip.SetTimeFactor(timeFactor);
                transferShip.SetTargetPoint(targetPoint);
                transferShip.SetTimeMode(lambertTimeMode);
                transferShip.SetTimeTransfer(timeTransfer);
                transferShip.lambertAlways = lambertAlways;
                TransferShip.Status xferStatus = transferShip.Compute(targetMode);
                if (xferStatus == TransferShip.Status.HIT_PLANET) {
                    Debug.LogWarning("Hit planet. Transfer not recommended");
                } else if (xferStatus != TransferShip.Status.OK) {
                    Debug.LogError("Transfer status = " + xferStatus);
                }

                maneuvers = transferShip.Maneuvers(centerBody.Id(), ship.propagator);
                // Debug.Log(ge.DumpAll("Transfers Added"));
            } else {
                Debug.LogError("Unable to find info for ship in GE ship=" + ship.name);
            }
        }

        /// <summary>
        /// Example callback function. In this case just log the actual dV
        /// applied compared to the dV anticipated by the maneuver.
        /// </summary>
        /// <param name="m"></param>
        /// <param name="pEvent"></param>
        private void ManeuverDone(GEManeuver m, GEPhysicsCore.PhysEvent pEvent)
        {
            // pEvent holds v at time of maneuver as v and v after as v_secondary
            Debug.LogFormat("Maneuver {0} complete estimated dV ={1} actual={2}", m.info, m.dV, pEvent.v_secondary - pEvent.v);
        }

        /// <summary>
        /// Apply the previously computed maneuvers via ComputeTransfer() to the ship.
        /// if the ship is using PKepler or SGP4 then the intermediate maneuvers need to be
        /// added as Kepler segments since this is what TransferShip has assumed as propagation.
        ///
        /// </summary>
        /// <param name="ge"></param>
        private void DoTransfer(GECore ge)
        {
            int shipId = ship.Id();
            if (maneuvers == null || maneuvers.Count == 0) {
                Debug.LogError("No maneuvers. Either did not compute or Lambert did not find a solution");
                return;
            }
            GEPhysicsCore.Propagator propType = ge.PropagatorTypeForId(shipId);
            bool nonKeplerRails = propType == GEPhysicsCore.Propagator.PKEPLER || propType == GEPhysicsCore.Propagator.SGP4_RAILS;
            for (int i = 0; i < maneuvers.Count; i++) {
                GEManeuver m = maneuvers[i];
                // HACK: to allow patched evolution, set the center ID in the maneuvers (put in TS?)
                m.centerId = centerBody.Id();
                // if (nonKeplerRails) {
                //     if (i == maneuvers.Count - 1) {
                //         m.prop = ge.PropagatorTypeForId(shipId);
                //     } else {
                //         m.prop = GEPhysicsCore.Propagator.KEPLER;
                //     }
                // }
                m.doneCallback = ManeuverDone;
                // will automatically take time relative to current time
                ge.ManeuverAdd(shipId, m);
                Debug.LogFormat("Xfer add: " + m.LogString());
            }
        }

        /// <summary>
        /// Compute and perform a transfer. 
        /// 
        /// Maneuvers will be added to the indicated GEcore
        /// </summary>
        /// <param name="ge"></param>
        /// <param name="args"></param>
        private void XferShip(GECore ge, object args)
        {
            ComputeTransferManeuvers(ge, args);
            DoTransfer(ge);
        }

        /// <summary>
        /// Get the maneuvers for the last transfer ComputeTransfer()
        /// </summary>
        /// <returns></returns>
        public List<GEManeuver> Maneuvers()
        {
            return maneuvers;
        }

        // Update is called once per frame
        void Update()
        {
            if (keysEnabled && Input.GetKeyDown(KeyCode.T)) {
                GECore ge = gsController.GECore();
                ge.PhyLoopCompleteCallbackAdd(XferShip);
            }
        }

        /// <summary>
        /// Compute and perform the transfer. 
        /// 
        /// This will be added as a physics complete callback, since in job mode we cannot change
        /// state while jobs are running. 
        /// </summary>
        public void Transfer()
        {
            maneuvers = new List<GEManeuver>(); // want an empty list if things go wrong
            gsController.GECore().PhyLoopCompleteCallbackAdd(XferShip);
        }

        /// <summary>
        /// Compute a transfer BUT do not preform it. 
        /// 
        /// The resulting maneuvers can be retrieved via Maneuvers()
        /// 
        /// This will be added as a physics complete callback, since in job mode we cannot change
        /// state while jobs are running. 
        /// </summary>
        public void ComputeTransfer()
        {
            maneuvers = new List<GEManeuver>(); // want an empty list if things go wrong
            gsController.GECore().PhyLoopCompleteCallbackAdd(ComputeTransferManeuvers);
        }
    }
}
