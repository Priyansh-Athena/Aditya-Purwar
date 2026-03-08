using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;


namespace GravityEngine2 {

    /// <summary>
    /// Sample controller to allow a mouse click in display space (assumed in the XY display space)
    /// to designate a destination for the ship to transfer to. 
    /// A Lambert transfer to the point is computed and if the path intersects the planet this is
    /// detected. This impact is logged and the resulting path is colored RED. 
    /// 
    /// The planet radius is taken from the physical radius of the centerBody GSBody data. 
    ///
    /// For simplicity, the controller assumes:
    /// - the planet is at the center of the display
    /// </summary>
    public class LambertToPointController : MonoBehaviour {

        [Header("Click for point, T to execute")]
        public Camera sceneCamera;
        public GSController gsController;

        public GSBody ship;
        public GSBody centerBody;

        [Header("Transfer time in fraction of to body orbital period")]
        public double xferTimeFactor = 0.5;

        public GameObject pointMarker;

        public GSDisplayOrbit transferOrbitPreview;

        public GSDisplayOrbitSegment transferOrbitPreviewSegment;

        private double3 targetPoint;

        private Lambert.LambertOutput lambertOutput;

        private bool prograde = true;

        // arg unused (but required by callback API)
        private void ComputeTransfer(GECore ge, object args)
        {
            double mu = ge.MuWorld(centerBody.Id());
            GEBodyState shipState = new GEBodyState();
            int shipId = ship.Id();
            bool ok = ge.StateById(shipId, ref shipState);
            Orbital.COE coe = ge.COE(shipId, centerBody.Id(), false);
            double xferTime = xferTimeFactor * coe.GetPeriod();
            double planetRadius = centerBody.radius *
                    GBUnits.DistanceConversion(centerBody.bodyInitData.units, gsController.defaultUnits);
            if (ok) {
                lambertOutput = Lambert.TransferProgradeToPoint(mu, shipState.r, targetPoint, shipState.v, xferTime, prograde: prograde);
                if (lambertOutput.status == Lambert.Status.OK) {
                    // Get info from the maneuver and use it to show the transfer orbit
                    if (transferOrbitPreviewSegment != null) {
                        transferOrbitPreviewSegment.RVRelativeSet(shipState.r, lambertOutput.v1t);
                        transferOrbitPreviewSegment.toRvec = targetPoint;
                        transferOrbitPreviewSegment.DisplayEnabledSet(true);
                    }
                    // Get info from the maneuver and use it to show the transfer orbit
                    if (transferOrbitPreview != null) {
                        transferOrbitPreview.RVRelativeSet(shipState.r, lambertOutput.v1t);
                        transferOrbitPreview.DisplayEnabledSet(true);
                    }
                } else {
                    Debug.LogWarningFormat("Lambert transfer failed {0}", lambertOutput.status);
                }
            } else {
                Debug.LogError("Unable to find info for ship in GE ship=" + ship.name);
            }
        }

        private void DoTransfer(GECore ge, object args)
        {
            if (lambertOutput.status != Lambert.Status.OK) {
                Debug.LogWarning("Transfer status not OK");
                return;
            }

            int shipId = ship.Id();
            GEPhysicsCore.Propagator prop = ge.PropagatorTypeForId(shipId);
            List<GEManeuver> maneuvers = lambertOutput.Maneuvers(centerBody.Id(), prop, intercept: true);
            ge.ManeuverAdd(shipId, maneuvers[0]);
            ge.ManeuverAdd(shipId, maneuvers[1]);

            transferOrbitPreview?.DisplayEnabledSet(false);
            transferOrbitPreviewSegment?.DisplayEnabledSet(false);
        }


        void Update()
        {
            // mouse button press designates a transfer point in the XZ (display plane)
            if (Input.GetMouseButtonDown(0)) {
                // assume XZ orbit plane
                Vector3 mousePos = Input.mousePosition;
                Vector3 mouseOnZ0 = new Vector3(mousePos.x, mousePos.y, sceneCamera.transform.position.magnitude);
                Vector3 targetNew = sceneCamera.ScreenToWorldPoint(mouseOnZ0);
                pointMarker.transform.position = new Vector3(targetNew.x, 0, targetNew.z);
                targetPoint = new double3(targetNew.x, targetNew.z, 0); // XY orbit plane in physics coords
                targetPoint /= transferOrbitPreview.GSDisplay().DisplayScale();
                Debug.LogFormat("Click at mouse={0} world={1}", mousePos, targetPoint);

                gsController.PausedSet(true);
                gsController.GECore().PhyLoopCompleteCallbackAdd(ComputeTransfer);
            }
            if (Input.GetKeyDown(KeyCode.T)) {
                gsController.PausedSet(false);
                // do the transfer
                gsController.GECore().PhyLoopCompleteCallbackAdd(DoTransfer);
            }
            if (Input.GetKeyDown(KeyCode.R)) {
                // assume we have a target point (i.e. mouse clicked first)
                prograde = !prograde;
                gsController.GECore().PhyLoopCompleteCallbackAdd(ComputeTransfer);
            }

        }
    }

}
