using UnityEngine;
using Unity.Mathematics;

namespace GravityEngine2 {
    /// <summary>
    /// Tutorial ShipManual controller code. 
    /// 
    /// Attach to GSDisplayBody.
    /// 
    /// Move ship in a planar orbit. Arrow keys change the orientation of the ship in the scene. 
    /// 
    /// Pressing the F keys applies a small amount of thrust in the direction of the ship in a discrete burst
    /// (one burst per keypress). 
    /// 
    /// The "J" key shows how to modify an orbit parameter interactively. 
    /// 
    /// This controller demonstrates how to change the behavior of a ship by interacting with the GravityEngine instance that
    /// controls it. 
    /// 
    /// </summary>
    public class ShipInOrbitController : MonoBehaviour {
        [Header("P: Pause G:Go")]
        [Header("A/D to direct thrust")]
        [Header("W/S to increase/decrease thrust")]
        public GSController gsController;

        [Header("DisplayOrbit to show preview when paused")]
        public GSDisplayOrbit orbitPreview;

        //! Required component
        private GSDisplayBody displayBody;

        //! id for the GSBody of the ship
        private int bodyId;

        private GECore ge;

        private GEBodyState pausedState;

        private double3 dV;

        //! angle change per A/D keypress
        private double angleDeltaRad = 5.0 * GravityMath.DEG2RAD;
        private const double thrustInitial = 0.05;
        private const double thrustStep = 0.025;

        // Start is called before the first frame update
        void Start()
        {
            displayBody = GetComponent<GSDisplayBody>();
            if (displayBody == null) {
                Debug.LogError("Script must be attached to a GSDisplayBody: " + gameObject.name);
            }
            gsController.ControllerStartedCallbackAdd(GEStart);
        }

        public void GEStart(GSController gsc)
        {
            ge = gsc.GECore();

            // turn off debug keys in GSC (conflict possible)
            if (gsController.debugKeys) {
                gsController.debugKeys = false;
                Debug.LogWarning("Turned off GSController debug keys to avoid conflict");
            }
        }

        // Update is called once per frame
        void Update()
        {
            if (Input.GetKeyDown(KeyCode.P)) {
                // pause and get ship state
                gsController.PausedSet(true);
                bodyId = displayBody.gsBody.Id();
                ge.StateById(bodyId, ref pausedState);
                // make initial thrust a fixed percent of current velocity
                // give orbit a meaningful value before enabling
                orbitPreview.gameObject.SetActive(true);
                dV = thrustInitial * pausedState.v;
                System.Collections.Generic.List<GSDisplay> displays = gsController.Displays();
                foreach (GSDisplay disp in displays) {
                    disp.DisplayObjectAdd(orbitPreview);
                }
                orbitPreview.RVRelativeSet(pausedState.r, pausedState.v + dV);

            } else if (Input.GetKeyDown(KeyCode.G)) {
                // go: commit velocity change and unpause
                gsController.PausedSet(false);
                orbitPreview.gameObject.SetActive(false);
                orbitPreview.enabled = false;
                int bodyId = displayBody.gsBody.Id();
                pausedState.v += dV;
                ge.StateSetById(bodyId, pausedState);
            } else if (Input.GetKeyDown(KeyCode.A)) {
                // Rotating about Z in physics RH system!
                dV = GravityMath.Rot1Z(dV, angleDeltaRad);
            } else if (Input.GetKeyDown(KeyCode.D)) {
                dV = GravityMath.Rot1Z(dV, -angleDeltaRad);
            } else if (Input.GetKeyDown(KeyCode.W)) {
                dV += thrustStep * dV;
            } else if (Input.GetKeyDown(KeyCode.S)) {
                dV -= thrustStep * dV;
            } else if (Input.GetKeyDown(KeyCode.J)) {
                // this is a bit awkward, but don't want to add something to inspector
                int centerId = orbitPreview.centerDisplayBody.gsBody.Id();
                // increase semi-major axis of the orbit
                Orbital.COE coe = Orbital.RVtoCOE(pausedState.r, pausedState.v, ge.MuWorld(centerId));
                coe.SetA(coe.a * 1.1); // increase by 10%
                (double3 r, double3 v) = Orbital.COEtoRVRelative(coe);
                pausedState.r = r;
                pausedState.v = v;
                // move the ship to the new R, V since it is no longer on the orbit
                ge.StateSetById(bodyId, pausedState);
            }

            if (orbitPreview.gameObject.activeInHierarchy)
                orbitPreview.RVRelativeSet(pausedState.r, pausedState.v + dV);
        }


    }
}
