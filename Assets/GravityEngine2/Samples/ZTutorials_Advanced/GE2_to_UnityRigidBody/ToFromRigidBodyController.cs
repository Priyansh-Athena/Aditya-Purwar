
using UnityEngine;

namespace GravityEngine2 {
    public class ToFromRigidBodyController : MonoBehaviour {
        public RigidBodyOrbit[] rigidBodyOrbits;

        public GSController gsController;

        private bool inRBmode = false;

        [Header("Delta in display space to return to GE2 mode")]
        public float collisionDelta = 5.0f;

        void Start()
        {
            gsController.ControllerStartedCallbackAdd(RBSetup);
        }

        private void RBSetup(GSController gsc)
        {
            gsc.GECore().PhysicsEventListenerAdd(CollisionListener);
        }

        private void CollisionListener(GECore ge, GEPhysicsCore.PhysEvent physEvent)
        {
            // trigger collisions may report multiple times due to timesteps
            if (physEvent.type == GEPhysicsCore.EventType.COLLISION) {
                if (!inRBmode) {
                    Debug.Log("Collision reported");
                    // for simplicity controller assumes it affects the bodies listed here
                    ToggleRBMode();
                }
            }
        }

        private void ToggleRBMode()
        {
            inRBmode = !inRBmode;
            foreach (RigidBodyOrbit rbo in rigidBodyOrbits)
                rbo.RigidBodyMode(inRBmode);
        }

        // Update is called once per frame
        void Update()
        {
            if (Input.GetKeyDown(KeyCode.S)) {
                ToggleRBMode();
            }
            if (inRBmode) {
                // when they get far enough apart, return to GE2
                // assume two bodies for simplicity
                if (Vector3.Distance(rigidBodyOrbits[0].transform.position,
                                    rigidBodyOrbits[1].transform.position) > collisionDelta) {
                    ToggleRBMode();
                }
            }
        }
    }

}
