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
    public class ShipToPointController : MonoBehaviour {

        [Header("Click for point, T to execute")]
        public Camera sceneCamera;
        public GSController gsController;

        public GSBody ship;
        public GSBody centerBody;

        public double timeFactor = 1.0;

        public GameObject pointMarker;

        public bool retrogradeTransfer = false;

        public GSDisplayOrbit transferOrbitPreview;

        public GSDisplayOrbitSegment transferOrbitPreviewSegment;

        private TransferShip transferShip;

        private double3 targetPoint;

        // arg unused (but required by callback API)
        private void ComputeTransfer(GECore ge, object args)
        {
            double mu = ge.MassWorld(centerBody.Id());
            GEBodyState shipState = new GEBodyState();
            int shipId = ship.Id();
            bool ok = ge.StateById(shipId, ref shipState);
            double planetRadius = centerBody.radius *
                    GBUnits.DistanceConversion(centerBody.bodyInitData.units, gsController.defaultUnits);
            if (ok) {
                transferShip = new TransferShip(shipState,
                                                toOrbit: null,
                                                mu,
                                                checkHit: true,
                                                planetRadius: planetRadius,
                                                retrograde: retrogradeTransfer);
                transferShip.SetTimeFactor(timeFactor);
                transferShip.SetTargetPoint(targetPoint);
                transferShip.lambertAlways = true;
                TransferShip.Status xferStatus = transferShip.Compute(TransferShip.TargetMode.SHIP_TO_POINT);
                if (xferStatus == TransferShip.Status.HIT_PLANET) {
                    Debug.LogWarning("Hit planet. Transfer not allowed");
                    // show transfer path so user can see
                }
                Debug.Log(ge.DumpAll("Transfer Computed"));
                GEManeuver m = transferShip.Maneuvers()[0];
                // Get info from the maneuver and use it to show the transfer orbit
                if (transferOrbitPreview != null) {
                    transferOrbitPreview.RVRelativeSet(shipState.r, m.v_relative);
                    transferOrbitPreview.DisplayEnabledSet(true);
                }
                // Get info from the maneuver and use it to show the transfer orbit
                if (transferOrbitPreviewSegment != null) {
                    transferOrbitPreviewSegment.RVRelativeSet(shipState.r, m.v_relative);
                    transferOrbitPreviewSegment.toRvec = targetPoint;
                    transferOrbitPreviewSegment.DisplayEnabledSet(true);
                }
            } else {
                Debug.LogError("Unable to find info for ship in GE ship=" + ship.name);
            }
        }

        private void DoTransfer(GECore ge, object args)
        {
            if (transferShip == null) {
                Debug.LogWarning("No transfer computed");
                return;
            }

            int shipId = ship.Id();
            foreach (GEManeuver m in transferShip.Maneuvers(centerBody.Id(), ship.propagator)) {
                Debug.LogFormat("Xfer add: " + m.LogString());
                // will automatically take time relative to current time
                ge.ManeuverAdd(shipId, m);
            }
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
                Debug.LogFormat("Click at mouse={0} world={1}", mousePos, targetNew);
                pointMarker.transform.position = new Vector3(targetNew.x, 0, targetNew.z);
                targetPoint = new double3(targetNew.x, targetNew.z, 0); // XY orbit plane in physics coords
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
                retrogradeTransfer = !retrogradeTransfer;
                gsController.GECore().PhyLoopCompleteCallbackAdd(ComputeTransfer);
            }

        }
    }

}
