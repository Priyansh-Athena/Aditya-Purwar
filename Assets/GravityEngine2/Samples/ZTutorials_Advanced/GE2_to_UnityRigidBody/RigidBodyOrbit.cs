using UnityEngine;
using Unity.Mathematics;

namespace GravityEngine2 {
    /// <summary>
    /// Demo script to allow a GE2 ship to flip from evolution under GE2 control to evolution 
    /// under control of Unity physics by applying a force to the rigidbody component in display space. The script
    /// assumes the ship is orbiting a planet and there is no other significant gravitational force. 
    /// 
    /// This allows the Unity collision physics to model e.g. collisions and docking bounces when objects are
    /// close enough to interact. 
    /// 
    /// The script assumes the center body is at the origin and does not move. 
    /// 
    /// Future: Could create oversight code that looks for potential close encounters in GE2 and then passes the bodies over
    /// to Unity until they move away from each other again. 
    /// 
    /// Future: Could handle multiple gravitational forces by having GE compute the accel and add a way to retrieve it.
    /// 
    /// </summary>
    public class RigidBodyOrbit : MonoBehaviour {
        public GSController gsController;
        public GSDisplay gsDisplay;

        private GSDisplayBody shipDisplayBody;
        private GSDisplayObject shipDisplayOrbit;
        public GSBody centerBody;

        public float rigidBodyMass = 1.0f;

        public bool RBatStart = false;

        public bool enableKeys = true;

        // for measurment purposes switch at 4 and 7 seconds
        public bool autoSwitch = true;

        private GECore geCore;
        private Rigidbody rigidBody;

        // flag to pass command to switch from Update() to FixedUpdate()
        private bool rbActive;

        // scale conversion GSController world units to display scale
        private float rWorldToRB;
        private float vWorldToRB;
        private double rRBtoWorld;
        private double vRBtoWorld;

        // instance of structs to avoid new() each time around
        private Vector3 vec3;
        private double3 r3, v3;

        double worldTimePerUnityTime;

        // mass of two bodies and the scaled gravitational constant
        private float Gm1m2;

        double lastFUTime;

        private int lastZoom;

        // Start is called before the first frame update
        void Start()
        {
            shipDisplayBody = GetComponent<GSDisplayBody>();
            shipDisplayOrbit = GetComponent<GSDisplayOrbit>();
            rigidBody = GetComponent<Rigidbody>();
            rigidBody.isKinematic = false;
            rigidBody.constraints = RigidbodyConstraints.FreezePosition;
            rigidBody.mass = rigidBodyMass;
            rigidBody.useGravity = false;
            rigidBody.linearDamping = 0;
            rigidBody.angularDamping = 0;
            rigidBody.interpolation = RigidbodyInterpolation.Extrapolate;

            // preconmpute the scaling required to map from GE. Do this as a callback to avoid start races
            gsController.ControllerStartedCallbackAdd(SetupScaling);
        }

        private void SetupScaling(GSController gsc)
        {
            float zoomFactor;

            geCore = gsController.GECore();
            rWorldToRB = gsDisplay.scale;
            vWorldToRB = gsDisplay.scale;
            rRBtoWorld = 1.0 / rWorldToRB;
            double displayG = GBUnits.GForUnits(gsController.defaultUnits);
            // need to adjust G based on display scale. G goes like kg m^3/sec^2. Only changing length
            displayG *= rWorldToRB * rWorldToRB * rWorldToRB;
            // now need to adjust based on time (Unity is not evolving at world time!)
            (lastZoom, zoomFactor) = gsController.TimeZoomAndFactor();
            worldTimePerUnityTime = zoomFactor * gsController.WorldTimePerGameSec();
            vWorldToRB *= (float)worldTimePerUnityTime;
            vRBtoWorld = 1.0 / vWorldToRB;
            // 1/s^2  s -> u so multiply by s/u twice
            displayG *= worldTimePerUnityTime * worldTimePerUnityTime;

            Gm1m2 = (float)(displayG * centerBody.mass * rigidBodyMass);
            // null check so know if callback or called from below
            if (RBatStart && gsc != null)
                GE2toRB();
        }

        private void GE2toRB()
        {
            Debug.Log(gameObject.name + " GE2->RB");
            // Remove from GE2 -> Rigid Body motion
            int shipId = shipDisplayBody.gsBody.Id();
            // 1. Get state info (from last Update cycle & snap back to FixedUpdate time
            GEBodyState shipState = new GEBodyState();
            geCore.StateById(shipId, ref shipState);
            // GE integrator *may* have over-stepped requested world time by some fraction of numerical integrator dt
            double tDelta = gsController.TimeWorldDeltaSinceLastPhyLoop();
            shipState.r += tDelta * shipState.v;
            // 2. Scale r, v into units that make sense in the display scene and set RB values
            GravityMath.Double3IntoVector3(shipState.r, ref vec3);
            if (gsDisplay.xzOrbitPlane)
                GravityMath.Vector3ExchangeYZ(ref vec3);
            Vector3 r_rb = rWorldToRB * vec3;
            GravityMath.Double3IntoVector3(shipState.v, ref vec3);
            if (gsDisplay.xzOrbitPlane)
                GravityMath.Vector3ExchangeYZ(ref vec3);
            Vector3 v_rb = vWorldToRB * vec3;
            // The physics system runs on FixedUpdate, we want to set the R, V for the position and velocity
            // at the time FU last run (so that when it next runs it starts with those values)
            float tSinceFU = (float)(Time.timeSinceLevelLoadAsDouble - lastFUTime);
            Debug.LogFormat("tSinceFU={0} tDelta={1}", tSinceFU, tDelta);
            r_rb = r_rb - tSinceFU * v_rb;
            rigidBody.constraints = RigidbodyConstraints.None;  // order matters, do this before setting v
            rigidBody.position = r_rb;
            rigidBody.linearVelocity = v_rb;
            // rigidBody.AddForce(v_rb, ForceMode.VelocityChange);
            // 3. Disable display but let body evolve in GE2 without display
            shipDisplayBody?.DisplayEnabledSet(false);
            shipDisplayOrbit?.DisplayEnabledSet(false);
            rbActive = true;
        }

        private void RBtoGE2()
        {
            Debug.Log("RB->GE2");
            // Remove from Rigid Body motion -> GE2
            // 1. Get r, v from RB and scale into GE world units
            //GravityMath.Vector3IntoDouble3(rigidBody.position, ref r3);
            // Physics.SyncTransforms();
            GravityMath.Vector3IntoDouble3(transform.position, ref r3);
            r3 *= rRBtoWorld;
            GravityMath.Vector3IntoDouble3(rigidBody.linearVelocity, ref v3);
            v3 *= vRBtoWorld;
            if (gsDisplay.xzOrbitPlane) {
                GravityMath.Double3ExchangeYZ(ref r3);
                GravityMath.Double3ExchangeYZ(ref v3);
            }
            // 2. Add the GEBody using R, V relative to the center body
            GSBody gsbody = shipDisplayBody.gsBody;
            // does better empiracally without this extrapolation...
            double tDelta = 0.0; //gsController.TimeUnityToWorld(gsController.TimeWorldDeltaSinceLastPhyLoop());
                                 // last phy loop will be earlier than extrapolated position
            gsbody.bodyInitData.r = r3 - tDelta * v3;
            // Determine accel at new world position to adjust v in world units
            double r_mag = math.length(gsbody.bodyInitData.r);
            double3 r_dir = math.normalize(gsbody.bodyInitData.r);
            double3 a = GBUnits.GForUnits(gsController.defaultUnits) * centerBody.mass * r_dir / (r_mag * r_mag);
            // center body already set
            //gsController.BodyAdd(shipDisplayBody.gsBody);
            GEBodyState shipState = new GEBodyState();
            shipState.r = r3 - tDelta * v3;
            shipState.v = v3 - tDelta * a;
            gsController.GECore().StateSetById(gsbody.Id(), shipState);
            Debug.LogFormat("RB->GE2 rb pos={0} xform={1} r={2} v={3}", rigidBody.position, transform.position,
                            gsbody.bodyInitData.r, gsbody.bodyInitData.v);

            // 3. Update display objects with new body id and re-enable display
            shipDisplayBody?.DisplayEnabledSet(true);
            shipDisplayOrbit?.DisplayEnabledSet(true);
            rigidBody.constraints = RigidbodyConstraints.FreezePosition;
            rbActive = false;
        }

        void FixedUpdate()
        {
            if (flipToRB) {
                Debug.LogFormat("Flip to RB lastV={0}", lastVelocity);
                rigidBody.constraints = RigidbodyConstraints.None;
                // rigidBody.AddForce(lastVelocity, ForceMode.VelocityChange);
                rigidBody.position = transform.position;
                rigidBody.linearVelocity = lastVelocity;
                // 3. Remove the body from GE2 evolution and disable display objects
                shipDisplayBody?.DisplayEnabledSet(false);
                shipDisplayOrbit?.DisplayEnabledSet(false);
                gsController.BodyRemove(shipDisplayBody.gsBody);
                rbActive = true;
                flipToRB = false;
            }
            // do force calc 
            if (rbActive) {
                // due to timeZoom
                (int zoom, float zoomFactor) = gsController.TimeZoomAndFactor();
                if (zoom != lastZoom) {
                    double oldRBtoWorld = vRBtoWorld;
                    SetupScaling(gsc: null);
                    // we also need to rescale the velocity 
                    rigidBody.linearVelocity *= (float)(oldRBtoWorld * vWorldToRB);
                }
                // apply gravitational force (in display scale units) to the body
                vec3 = rigidBody.position;
                float forceG = Gm1m2 / (vec3.magnitude * vec3.magnitude);
                rigidBody.AddForce(-forceG * vec3.normalized);
            } else {
                // running in GE2
                lastVelocity = (rigidBody.position - lastPosition) / Time.fixedDeltaTime;
                lastPosition = rigidBody.position;
            }
            lastFUTime = Time.timeSinceLevelLoadAsDouble;
        }

        public void RigidBodyMode(bool enabled)
        {
            if (enabled && !rbActive) {
                GE2toRB();
            } else if (!enabled && rbActive) {
                RBtoGE2();
            }
        }

        int frameCount = 0;
        int FRAMES_PER_SWITCH = 240;

        bool flipToRB;

        // Update is called once per frame
        void Update()
        {
            if (enableKeys && Input.GetKeyDown(KeyCode.S)) {
                if (rbActive) {
                    RBtoGE2();
                } else {
                    GE2toRB();
                    //flipToRB = true;
                }
            }
            if (autoSwitch) {
                if (frameCount++ > FRAMES_PER_SWITCH) {
                    frameCount = 0;
                    if (rbActive) {
                        RBtoGE2();
                    } else {
                        GE2toRB();
                    }
                }
            }
        }

        private Vector3 lastVelocity;
        private Vector3 lastPosition;
        // assumes GScontroller in immediate mode (so GScontroller will have set transform position)
        void LateUpdate()
        {
            if (!rbActive) {
                // track r, v for switch over
                rigidBody.position = transform.position;
            }
        }

    }
}
