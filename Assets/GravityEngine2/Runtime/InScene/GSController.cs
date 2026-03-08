using System;
using Unity.Mathematics;
using System.Collections.Generic;
using UnityEngine;
using Unity.Jobs;

namespace GravityEngine2 {
    /// <summary>
    /// The GSController (GSC) manages a set of gravitational bodies in an orbital system and their
    /// associated scene display components. There may be zero, one or many GSControllers that
    /// make reference to the gravitational bodies managed by this controller.
    ///
    /// In-scene gravitational bodies MUST contain a GSBody component to be added by GSC.
    /// GSC can find the bodies to be added in one of three ways:
    ///   CHILDREN: scan the children of the GSC game object for GSBody objects
    ///   LIST: provide a list of references in the inspector
    ///   SCENE: Scan the entire scene for GSBody objects. (In this case there will likely only be 1 GSC in the
    ///          scene)
    ///
    /// There can be any number of independent sets of gravitational bodies managed by separate instances of
    /// this component in a scene i.e. it is not a singleton. 
    ///
    /// The goal of the scene controller is to handle the creation of a system of gravitational bodies via
    /// components attached to game objects in the scene and to evolve these bodies forward in time. 
    ///
    /// Gravitational evolution/simulation is performed in the Update() function via gravitySystem.
    ///
    /// DISPLAYING:
    /// The updating of game objects based on gravitational interaction is delegated to GSDisplay
    /// components. This allows multiple views of objects in the scene and allow the scene display components
    /// to select which objects should be displayed based on game state, distance from camera etc.
    /// 
    /// </summary>
    public class GSController : MonoBehaviour, GEListenerIF {
        public bool DEBUG = true;  // enable debugging logging to the console

        /// <summary>
        /// Mode the controller will use to find GSBody objects to add
        /// CHILDREN: Scan all children of this controller for GSBody
        /// LIST: Add objects explicitly listed
        /// SCENE: Find all objects in the scene with GSBody components
        /// </summary>
        public enum AddMode { CHILDREN, LIST, SCENE };

        public AddMode addMode = AddMode.CHILDREN;

        public enum EvolveMode { IMMEDIATE, IJOB_LATEUPDATE };

        public EvolveMode evolveMode;

        public GSBody[] addBodies;

        public GBUnits.Units defaultUnits;

        //! Typical orbit scale in default units.
        public double orbitScale = 100.0;
        //! Mass scale scale in default units for the most common center object
        public double orbitMass = 1000.0;

        //! Time for Typical circular orbit
        public enum TimeScale { PER_ORBIT, WORLD };
        public TimeScale timeScale;
        public double gameSecPerOrbit = 60.0;
        public double worldSecPerGameSec = 60.0;

        public double stepsPerOrbit = 500.0; // for more eeccentric orbits may need more. Used to set dt in integrators.
        public static double MIN_STEPS_ORBIT = 100.0;

        private double worldTimePerGameSec = 1.0;

        // Reference frame details (CR3BP)
        public bool editorReferenceFrameFoldout = false;

        public GECore.ReferenceFrame referenceFrame = GravityEngine2.GECore.ReferenceFrame.INERTIAL;
        public GSBody primaryBody;
        public GSBody secondaryBody;
        public CR3BP.TB_System cr3bpSystem = CR3BP.TB_System.EARTH_MOON;

        public CR3BP.CR3BPSystemData cr3bpSysData;

        public bool editorScalingDetailsFoldout;

        // World time at start (for non-DL scenes)
        public bool editorWorldTimeFoldout;
        public int year = 2023;
        public int month = 1;
        public int day = 1;
        public double utc = 0.0;

        public bool debugKeys = false;
        public bool dumpInWorldUnits = true;

        public bool debugLogEvents = false;

        private bool paused = false;

        private double lastUnityEvolveTime = 0.0;

        //! Elapsed time in world space units (does not advance when GSController is paused, advances faster when time is zoomed up)
        private double elapsedWorldTime = 0.0;
        private double startTimeJD;

        private int timeZoom = 0;
        private float[] timeZoomFactor = new float[] { 1f, 2f, 3f, 4f, 5f, 10f, 20f, 50f, 100f };

        public Integrators.Type integrator;

        private List<GSDisplay> sceneDisplays;

        private GECore ge;
        // flag to indicate need to setup traj before executing or scheduling.
        // This allows pending setups to be done once and not per Add.
        private bool setupTrajectoryInfo;

        private List<JobHandle> particleJobHandles;

        private bool setup = false;

        // In heirarchical systems, only one controller can drive the worldTime forward
        // and the others just take it from there. 
        private GSController primaryController;

        // TODO: Maintain this state
        // private bool haveParticles = true;

        /// <summary>
        /// Trajectory prediction. Bodies can ask for trajectory prediction.
		/// Typically done with a bool in the GSDisplayBody (since this is
		/// typically a per-display choice).
        ///
        /// This requires the instantiation of a second GE which tracks
		/// the first but runs ahead of the current one. Data recording into
		/// recordedOutput is done. Initially (and when something changes in
		/// the GE via Add/Maneuuver etc.) the traj GE needs to do re-do 
		/// the work to compute up to the timeAhead requested. After that it just does
        /// the incremental advance as time goes forward. 
        /// 
        /// </summary>
        public bool editorTrajectoryFoldout;
        public bool trajEnabled;
        public double trajLookAhead = 10.0;
        public int trajNumSteps = 200;

        // Advanced integration control

        private List<int> trajBodies;

        public delegate void ControllerStartedCallback(GSController gsc);

        private List<ControllerStartedCallback> startedCallbacks;

        private Dictionary<int, GSBody> gsBodiesById;

        void Awake()
        {
            particleJobHandles = new List<JobHandle>();
            startedCallbacks = new List<ControllerStartedCallback>();
            gsBodiesById = new Dictionary<int, GSBody>();
        }

        // Start is called before the first frame update
        void Start()
        {
            trajBodies = new List<int>();
            List<GSBody> bodies = BodiesForInit();
            GravitySetup(bodies);
            foreach (ControllerStartedCallback scb in startedCallbacks)
                scb(this);
        }

        private void GEInit()
        {
            if (stepsPerOrbit < MIN_STEPS_ORBIT) {
                stepsPerOrbit = MIN_STEPS_ORBIT;
                Debug.LogWarning("Bumped steps per orbit to " + MIN_STEPS_ORBIT);
            }

            startTimeJD = TimeUtils.JulianDate(year, month, day, utc);
            switch (referenceFrame) {
                case GravityEngine2.GECore.ReferenceFrame.INERTIAL:
                    ge = new GECore(integrator,
                                            defaultUnits,
                                            orbitScale,
                                            orbitMass,
                                            stepsPerOrbit,
                                            startTimeJD);
                    break;

                case GravityEngine2.GECore.ReferenceFrame.COROTATING_CR3BP:
                    cr3bpSysData = CR3BP.SystemData(cr3bpSystem);
                    ge = new GECore(integrator,
                                    defaultUnits,
                                    cr3bpSysData,
                                    stepsPerOrbit,
                                    startTimeJD);
                    break;
            }
            // with Nbody units in GE one orbit period is 2 Pi in GE time
            if (timeScale == TimeScale.PER_ORBIT) {
                worldTimePerGameSec = ge.OrbitPeriodWorldTime() / gameSecPerOrbit;
            } else {
                worldTimePerGameSec = worldSecPerGameSec;
            }

            double dt = ge.Dt();
            // Want to ensure that for some reasonable frame rate (say 120fps), we're doing about one dt per frame
            // otherwise will get jerky motion.
            double checkDtPerFrame = 1.0 / 120.0 * worldTimePerGameSec * ge.GEScaler().ScaleTimeWorldToGE(1.0); ;
            if (dt > checkDtPerFrame) {
                Debug.Log("Tune dt =" + checkDtPerFrame);
                ge.DtSet(checkDtPerFrame);
            }

            ge.ListenerAdd(this);
            ge.PhysicsEventListenerAdd(PhysEventCallback);
        }

        private void PhysEventCallback(GECore ge, GEPhysicsCore.PhysEvent pEvent)
        {
            if (debugLogEvents) {
                Debug.Log(pEvent.LogString());
            }
        }

        /// <summary>
        /// Add a callback that will be invoked once the GScontroller has completed setup. 
        ///
        /// This is a useful mechanism for Monobehaviours in the scene to do setupin a way that 
        /// does not rely on Start() operations occuring in a specific time order.
        /// </summary>
        /// <param name="startCB"></param>
        public void ControllerStartedCallbackAdd(ControllerStartedCallback startCB)
        {
            if (setup) {
                startCB(this);
            } else {
                startedCallbacks.Add(startCB);
            }
        }

        /// <summary>
        /// Register a GSDisplay class with this controller. Typically used by GSDisplay as part of
        /// startup and not as part of user code.
        /// </summary>
        /// <param name="gsd"></param>
        public void DisplayRegister(GSDisplay gsd)
        {
            if (sceneDisplays == null)
                sceneDisplays = new List<GSDisplay>();

            sceneDisplays.Add(gsd);
        }

        /// <summary>
        /// Return a list of the active displays managed by this  GSController. 
        /// </summary>
        /// <returns></returns>
        public List<GSDisplay> Displays()
        {
            return sceneDisplays;
        }

        /// <summary>
        /// Used by GSControllerEditor script. 
        /// </summary>
        /// <returns></returns>
        public List<GSBody> BodiesForInit()
        {
            GSBody[] toAdd = null;
            switch (addMode) {
                case AddMode.CHILDREN:
                    toAdd = gameObject.GetComponentsInChildren<GSBody>();
                    break;

                case AddMode.LIST:
                    toAdd = addBodies;
                    break;

                case AddMode.SCENE:
                    toAdd = FindObjectsByType<GSBody>(FindObjectsSortMode.None);
                    break;
            }
            List<GSBody> bodies = new List<GSBody>();
            // put array into list
            bodies.AddRange(toAdd);
            // If CR3BP do not add primary and secondary. They will be treated as special.
            if (referenceFrame == GravityEngine2.GECore.ReferenceFrame.COROTATING_CR3BP) {
                bodies.Remove(primaryBody);
                bodies.Remove(secondaryBody);
                // by construction in GECore, these were assigned
                primaryBody.IdSet(0);
                secondaryBody.IdSet(1);
            }
            return bodies;
        }

        /// <summary>
        /// Create a gravity system to hold physics info on the bodies
        /// and to evolve them.
        ///
        /// </summary>
        private void GravitySetup(List<GSBody> bodies)
        {
            // In special cases (meta controller like SolarSystemController)
            // a direct call to GEInit() may have been made before Start()
            if (ge == null)
                GEInit();

            foreach (GSBody b in bodies) {
                if (b.Id() < 0) {
                    BodyAdd(b);
                }
            }

            ge.PreCalcIntegration();

            // GSDisplay/Trajectory
            List<int> trajectories = new List<int>();
            foreach (GSDisplay gsd in sceneDisplays) {
                // Scene display Init
                trajectories.AddRange(gsd.Init()); // <= *** GSDISPLAY INIT ***
            }
            if (trajectories.Count > 0) {
                // Define a unique list of bodies that want trajectory info recorded.
                // May be consecutive entries that are the same, want to only add this once. 
                trajectories.Sort();
                int last = -1;
                for (int i = 0; i < trajectories.Count; i++) {
                    // prune duplicates
                    if (trajectories[i] != last) {
                        trajBodies.Add(trajectories[i]);
                        last = trajectories[i];
                    }
                }
                // can do this now, since GE is not running
                ge.TrajectorySetup(trajLookAhead, trajNumSteps, trajBodies);
            }
            foreach (GSDisplay gsd in sceneDisplays) {
                gsd.ParticleSetup(ge.Dt(), ge.TimeGE(), ge.GEScaler());
            }
            setup = true;
#pragma warning disable 162     // disable unreachable code warning
            if (DEBUG) {
                Debug.Log(ge.DumpAll("scene controller initial state"));
                Debug.Log(ge.DumpAll("scene controller initial state WORLD", worldScale: true));
                foreach (GSDisplay gsd in sceneDisplays) {
                    Debug.Log(gsd.DumpAll());
                }
            }
#pragma warning restore 162        // apply an impulse to the indicated NBody
        }

        private void SetupTrajectoryInfo()
        {
            ge.TrajectorySetup(trajLookAhead, trajNumSteps, trajBodies);
            setupTrajectoryInfo = false;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public GECore GECore()
        {
            return ge;
        }

        /// <summary>
        /// Set the paused state for the GSController. Used to pause and resume
        /// physics evolution of the bodies managed by this controller.
        /// </summary>
        /// <param name="paused"></param>
        public void PausedSet(bool paused)
        {
            this.paused = paused;
        }

        /// <summary>
        /// Add a GSBody to the physics engine managed by this controller. 
        ///
        /// Note that any display routines (GSDisplayBody, GSDisplayOrbit) asociated
        /// with the body must be added by calls to the appropriate GSDisplay. 
        /// </summary>
        /// <param name="gsbody"></param>
        public void BodyAdd(GSBody gsbody)
        {
            if (ge.IsRunning()) {
                Debug.LogErrorFormat("Cannot add {0} when running. Either mode=IMMEDIATE or AddPhysLoopCompleteCallback");
                return;
            }

            // Special case: Binary orbits. These consist of three objects, 2 x GSBody members
            // and a GSBinary for the CM. The CM will be initially added so it's R, V can be
            // determined but may be removed if addCM is false. (Cleanup code at bottom)
            if (gsbody.Id() != GSBody.NO_ID)
                return; // entire binary already added

            GSBinary gsBinary = gsbody.Binary();
            if (gsBinary != null) {
                if (gsBinary.Id() == GSBody.NO_ID) {
                    BodyAdd(gsBinary);
                }
                GEBodyState cmState = new GEBodyState();
                ge.StateById(gsBinary.Id(), ref cmState);
                gsBinary.InitMember(gsbody, cmState);
            }

            // normal case
            BodyInitData bid = gsbody.bodyInitData;
            GBUnits.Units units = bid.units;
            double m = gsbody.mass;

            int id;
            if (gsbody.propagator == GEPhysicsCore.Propagator.EPHEMERIS) {
                units = defaultUnits;
                EphemerisData eData = new EphemerisData {
                    fileName = gsbody.ephemFilename,
                    relative = gsbody.ephemRelative,
                    format = gsbody.ephemFormat,
                    interpolate = gsbody.ephemInterpolate,
                };
                double startTimeJD = TimeUtils.JulianDate(year, month, day, utc);
                if (!EphemerisLoader.LoadFile(eData, defaultUnits, startTimeJD)) {
                    Debug.LogErrorFormat("failed to load ephemeric data {0} file={1}",
                        gsbody.gameObject.name, eData.fileName);
                    return;
                }
                // now need to ask GE to configure an ephem prop.
                // cid = -1; // if -1 then absolute, else relative
                int cid = -1;
                if (gsbody.ephemRelative) {
                    GSBody centerBody = gsbody.CenterBody();
                    if (centerBody == null) {
                        Debug.LogErrorFormat("{0} has no center body. Cannot add.", gsbody.gameObject.name);
                        return;
                    }
                    // May need to add the body we are orbiting around (and could be recurisive)
                    if (centerBody.Id() < 0) {
                        BodyAdd(centerBody);
                    }
                    cid = centerBody.Id();
                }
                id = ge.BodyAddWithEphemeris(eData, cid, m, gsbody.patched);
            } else {
                // does the GSBody want to be inited in orbit around something?
                bool initOrbit = !(bid.initData == BodyInitData.InitDataType.RV_ABSOLUTE);

                if (initOrbit) {
                    GSBody centerBody = gsbody.CenterBody();
                    if (centerBody == null) {
                        Debug.LogErrorFormat("{0} has no center body. Cannot add.", gsbody.gameObject.name);
                        return;
                    }
                    // May need to add the body we are orbiting around (and could be recurisive)
                    if (centerBody.Id() < 0) {
                        BodyAdd(centerBody);
                    }
                    int cid = centerBody.Id();
                    switch (bid.initData) {
                        case BodyInitData.InitDataType.COE:
                        case BodyInitData.InitDataType.COE_ApoPeri:
                        case BodyInitData.InitDataType.COE_HYPERBOLA: {
                                Orbital.COE coe = gsbody.COEFromBID();
                                // convert to world default units (conversion to phys inside GECore)
                                if (bid.units != defaultUnits) {
                                    coe.p *= GBUnits.DistanceConversion(bid.units, defaultUnits, cr3bpSysData);
                                    coe.a *= GBUnits.DistanceConversion(bid.units, defaultUnits, cr3bpSysData);
                                }
                                // determine if there is an epoch time
                                double epochWorldTime = bid.EpochTimeWorld(startTimeJD);
                                id = ge.BodyAddInOrbitWithCOE(coe, cid, gsbody.propagator, m, gsbody.patched,
                                            earlyPropagation: gsbody.earlyPropagation,
                                            epochWorldTime: epochWorldTime);
                                break;
                            }

                        case BodyInitData.InitDataType.RV_RELATIVE:
                            double3 r = bid.r;
                            double3 v = bid.v;
                            if (bid.units != defaultUnits) {
                                r *= GBUnits.DistanceConversion(bid.units, defaultUnits, cr3bpSysData);
                                v *= GBUnits.VelocityConversion(bid.units, defaultUnits, cr3bpSysData);
                            }
                            id = ge.BodyAddInOrbitWithRVRelative(r, v, cid,
                                    gsbody.propagator, m, gsbody.patched,
                                    earlyPropagation: gsbody.earlyPropagation); // patch start??
                            break;

                        case BodyInitData.InitDataType.LATLONG_POS:
                            // convert altitude to center body units
                            double altitude = gsbody.bodyInitData.altitude;
                            if (bid.units != gsbody.centerBody.bodyInitData.units) {
                                altitude *= GBUnits.DistanceConversion(bid.units, gsbody.centerBody.bodyInitData.units);
                            }
                            double3 r1 = GravityMath.LatLongToR(gsbody.bodyInitData.latitude,
                                                gsbody.bodyInitData.longitude,
                                                gsbody.centerBody.radius + altitude);
                            double3 v1 = double3.zero;
                            if (bid.units != defaultUnits) {
                                r1 *= GBUnits.DistanceConversion(bid.units, defaultUnits, cr3bpSysData);
                            }
                            // phi0 is just the offset. Prop will use the longitude
                            double phi0 = gsbody.centerBody.rotationPhi0;
                            if (gsbody.bodyInitData.earthRotation) {
                                phi0 = Orbital.GreenwichSiderealTime(startTimeJD);
                            }
                            if (gsbody.propagator == GEPhysicsCore.Propagator.PLANET_SURFACE) {
                                double3 axis = GravityMath.Vector3ToDouble3(gsbody.centerBody.rotationAxis);
                                id = ge.BodyAddOnPlanetSurface(gsbody.bodyInitData.latitude,
                                    gsbody.bodyInitData.longitude, cid,
                                    gsbody.centerBody.radius + altitude,
                                    axis,
                                    phi0,
                                    gsbody.centerBody.rotationRate,
                                    isPatched: gsbody.patched);
                            } else {
                                id = ge.BodyAddInOrbitWithRVRelative(r1, v1, cid,
                                    gsbody.propagator, m, gsbody.patched);
                            }
                            break;

                        case BodyInitData.InitDataType.TWO_LINE_ELEMENT: {
                                double epochWorldTime = bid.EpochTimeWorld(startTimeJD);
                                id = ge.BodyAddInOrbitWithTLE(gsbody.bodyInitData.tleData, cid, gsbody.propagator, gsbody.patched,
                                        earlyPropagation: gsbody.earlyPropagation,
                                        epochWorldTime: epochWorldTime);
                                break;
                            }

                        default:
                            throw new NotImplementedException("Unknown case: " + bid.initData);
                    }
                } else {
                    // not in orbit. Can only be FIXED or GRAVITY
                    if (GEPhysicsCore.NeedsCenter(gsbody.propagator)) {
                        Debug.LogErrorFormat("Propagator {0} requires an Orbit for {1}. Not added.",
                                gsbody.propagator, gsbody.gameObject.name);
                        return;
                    }
                    double3 r = bid.r;
                    double3 v = bid.v;
                    if (bid.units != defaultUnits) {
                        r *= GBUnits.DistanceConversion(bid.units, defaultUnits, cr3bpSysData);
                        v *= GBUnits.VelocityConversion(bid.units, defaultUnits, cr3bpSysData);
                    }
                    id = ge.BodyAdd(r, v, m, gsbody.propagator == GEPhysicsCore.Propagator.FIXED);
                }
            }
            if (id < 0) {
                Debug.LogErrorFormat("Failed to add {0}", gsbody.gameObject.name);
                return;
            }
            gsbody.IdSet(id);
            gsBodiesById.Add(id, gsbody);

            // External acceleration??
            GSExternalAcceleration gsExtA = gsbody.GetComponent<GSExternalAcceleration>();
            if (gsExtA != null) {
                gsExtA.AddToGE(id, ge, units);
            }
            // Colliders
            GSCollider gsc = gsbody.GetComponent<GSCollider>();
            if (gsc != null) {
                double radius = gsbody.radius * GBUnits.DistanceConversion(bid.units, defaultUnits, cr3bpSysData);
                double mass = gsc.inertialMass;
                if (gsc.useGsbodyMass) {
                    // currently no mass conversions between different world unit systems. 
                    mass = gsbody.mass;
                }
                ge.ColliderAddToBody(id, radius, gsc.bounceFactor, gsc.collisionType, mass);
            }

            // GSBinary cleanup. If CM was not to be retained and we have added both members, then remove
            // the CM
            if (gsBinary != null) {
                if ((gsBinary.body1.Id() >= 0) && (gsBinary.body2.Id() >= 0) && !gsBinary.addCM) {
                    BodyRemove(gsBinary);
                }
            }
        }

        /// <summary>
        /// Remove the specified body from the controller and underlying GE.
        ///
        /// If trajectories are in play, the body will also be removed from the
        /// trajectory GE.
        ///
        /// Remove is done when the ge physics is not running. In IMMEDIATE mode will be
        /// run immediatly, otherwise will be in LateUpdate when job completes.
        /// </summary>
        /// <param name="gsBody"></param>
        public void BodyRemove(GSBody gsBody, bool removeFromDisplays = true, bool destroyDisplayGO = false)
        {
            ge.PhyLoopCompleteCallbackAdd(BodyRemoveCb, gsBody);
            gsBodiesById.Remove(gsBody.Id());
            if (removeFromDisplays) {
                foreach (GSDisplay gsd in sceneDisplays) {
                    gsd.DisplayBodyRemoveByBodyId(gsBody.Id(), destroyGO: destroyDisplayGO);
                }
            }
            gsBody.IdSet(GSBody.NO_ID);
        }

        /// <summary>
        /// Get the GSBody associated with the given id.
        /// </summary>
        /// <param name="id"></param>
        /// <returns></returns>
        public GSBody GSBodyById(int id)
        {
            if (gsBodiesById.TryGetValue(id, out GSBody gsBody)) {
                return gsBody;
            }
            return null;
        }

        /// <summary>
        /// Get a copy of the dictionary of GSBody by id.
        /// </summary>
        /// <returns></returns>
        public Dictionary<int, GSBody> GSBodyIdDictionary()
        {
            return new Dictionary<int, GSBody>(gsBodiesById);
        }

        private void BodyRemoveCb(GECore geFromCb, object gsBody)
        {
            geFromCb.BodyRemove(((GSBody)gsBody).Id());
        }

        // If want raw GE values, get GE and ask it directly. GSC deals in world space
        public bool BodyState(GSBody gsBody, ref GEBodyState state, GBUnits.Units units)
        {
            bool ok = ge.StateById(gsBody.Id(), ref state);
            if (ok) {
                if (units != defaultUnits) {
                    state.r *= GBUnits.DistanceConversion(defaultUnits, units, cr3bpSysData);
                    state.v *= GBUnits.VelocityConversion(defaultUnits, units, cr3bpSysData);
                }
            }
            return ok;
        }

        /// <summary>
        /// Used to add a trajectory after the controller has started. (Those in display bodies
        /// added during startup will be handled in startup code).
        ///
        /// If there have been no trajectories added so far, then need to setup the parallel GE
        /// to compute them.
        ///
        /// If we have a GE doing traj evolution, it needs to restart from the current GE state so the
        /// new trajectory can be included.
        ///
        /// In addition, if a body with mass is added (that could affect existing trajectories) also need
        /// to restart.
        /// 
        /// </summary>
        /// <param name="bodyId"></param>
        public void TrajectoryAdd(int bodyId)
        {
            // do we have traj?
            if (!trajBodies.Contains(bodyId)) {
                trajBodies.Add(bodyId);
                trajBodies.Sort();
                setupTrajectoryInfo = true;
            }
        }

        /// <summary>
        /// Remove a trajectory from the physics engine. 
        /// </summary>
        /// <param name="bodyId"></param>
        public void TrajectoryRemove(int bodyId)
        {
            if (trajBodies.IndexOf(bodyId) >= 0) {
                trajBodies.Remove(bodyId);
                ge.TrajectoryRemove(bodyId);
            }
        }

        /// <summary>
		/// *IF* all bodies are on rails the time can be set to a value in the future or
		/// past and on the next physics evolution cycle all bodies will jump to the
		/// new locations.
		///
		/// The caller needs to ensure this is not done when GE is running, so it might be
		/// necessary to do this as a PhysLoop callback.
		/// 
		/// </summary>
		/// <param name="time"></param>
        public bool TimeWorldSet(double time)
        {
            elapsedWorldTime = time;
            return ge.TimeWorldSet(time);
        }

        /// <summary>
        /// Retrieve the time by which internal GEPhysicsCore time exceeds the reported world time
        /// due to dt granularity. For a system that is all on rails this will be zero. 
        ///
        /// Result will be at most dt
        /// </summary>
        public double TimeWorldOvershoot()
        {
            return ge.TimeWorld() - elapsedWorldTime;
        }


        /// <summary>
        /// Compute the elapsed world time from the last physics update to now. This assumes there
        /// was not a pause in the corresponding interval.
        /// </summary>
        public double TimeWorldDeltaSinceLastPhyLoop()
        {
            return (Time.timeSinceLevelLoadAsDouble - lastUnityEvolveTime) * timeZoomFactor[timeZoom] * worldTimePerGameSec;
        }

        public double TimeUnityToWorld(double unityDelta)
        {
            return unityDelta * timeZoomFactor[timeZoom] * worldTimePerGameSec;
        }

        /// <summary>
        /// Log the internal state of GECore and all registered display classes
        /// to the console.
        /// </summary>
        /// <param name="ge"></param>
        /// <param name="arg"></param>
        public void LogDebugInfo(GECore ge, object arg = null)
        {
            Debug.Log(this.ge.DumpAll(worldScale: dumpInWorldUnits, gsc: this));
            foreach (GSDisplay gsd in sceneDisplays) {
                Debug.Log(gsd.DumpAll());
            }
        }

        /// <summary>
        /// Standard Update loop in Unity engine
        /// - check for debug key press (if active)
        ///
        /// If immediate mode: evolve and call display updates
        ///
        /// else Schedule jobs. These are completed in LateUpdate and display update is there
        /// </summary>
        void Update()
        {
            if (debugKeys) {
                if (Input.GetKeyDown(KeyCode.D)) {
                    ge.PhyLoopCompleteCallbackAdd(LogDebugInfo);
                }
                if (Input.GetKeyDown(KeyCode.P))
                    paused = !paused;
                for (int i = 0; i < 10; i++) {
                    if (Input.GetKeyDown(KeyCode.Alpha1 + i)) {
                        timeZoom = i;
                        break;
                    }
                }
            }
            if (paused) {
                foreach (GSDisplay gsd in sceneDisplays) {
                    gsd.DisplayUpdate(ge, elapsedWorldTime);
                }
                return;
            }

            if (primaryController == null) {
                elapsedWorldTime += Time.deltaTime * timeZoomFactor[timeZoom] * worldTimePerGameSec;
            } else {
                elapsedWorldTime = primaryController.WorldTime();
            }
            lastUnityEvolveTime = Time.timeSinceLevelLoadAsDouble;

            List<int> massiveIndices = ge.MassiveIndicesGet();
            ref GEPhysicsCore.GEBodies bodies = ref ge.BodiesGetAll();

            switch (evolveMode) {
                case EvolveMode.IMMEDIATE:
                    //Debug.LogFormat("evolve to t_game={0} elapsedPhy={1}", elapsedGameTime, elapsedGETime);
                    if (setupTrajectoryInfo)
                        SetupTrajectoryInfo();
                    ge.EvolveNow(elapsedWorldTime);
                    foreach (GSDisplay gsd in sceneDisplays) {
                        EvolveParticles(massiveIndices, gsd);
                        gsd.DisplayUpdate(ge, elapsedWorldTime);
                        gsd.ParticlesUpdate(orbitScale);
                    }
                    break;

                case EvolveMode.IJOB_LATEUPDATE:
                    // HACK: Do immediate particle evolution. When do code below particle creation does not happen
                    // Feels like Unity jank...
                    // Particles need a COPY in Job mode (so we can evolve particles in parallel with bodies, trajectories etc.)
                    foreach (GSDisplay gsd in sceneDisplays) {
                        EvolveParticles(massiveIndices, gsd);
                        gsd.ParticlesUpdate(orbitScale);
                    }
                    if (setupTrajectoryInfo)
                        SetupTrajectoryInfo();
                    ge.Schedule(elapsedWorldTime);
                    break;
#if NEEDS_DEBUG
                case EvolveMode.IJOB_PARTICLES:
                    // Particles need a COPY in Job mode (so we can evolve particles in parallel with bodies, trajectories etc.)
                    if (haveParticles) {
                        massiveIndices = ge.MassiveIndicesGet();
                        NativeArray<GSParticles.MassiveBodyData> massiveData = GSParticles.MassiveBodyDataInit(massiveIndices, ref bodies);
                        foreach (GSDisplay gsd in sceneDisplays) {
                            particleSystems = gsd.ParticleSystems();
                            particleJobHandles.Clear();
                            if (particleSystems.Count > 0) {
                                double fromTime = ge.TimeGE();
                                // slight inaccuracy here, since we don't know how much particle integration overshot...
                                foreach (GSParticles gsp in particleSystems) {
                                    gsp.ParallelJobSetup(massiveIndices, ref bodies);
                                    if (gsp.pJob.particleCount > 0) {
                                        gsp.pJob.massiveBodies = massiveData;
                                        gsp.pJob.SetParticleTimes( fromTime, toTime: elapsedWorldTime);
                                        particleJobHandles.Add(gsp.pJob.Schedule());
                                    }
                                }
                            }
                        }
                    }
                    if (setupTrajectoryInfo)
                        SetupTrajectoryInfo();
                    ge.Schedule(elapsedWorldTime);
                    break;
#endif
            }

        }


        private void EvolveParticles(List<int> massiveIndices, GSDisplay gsd)
        {
            List<GSParticles> particleSystems = gsd.ParticleSystems();
            if (particleSystems.Count > 0) {
                if (massiveIndices == null) {
                    massiveIndices = ge.MassiveIndicesGet();
                }
                double evolvedPhyTime = ge.GEScaler().ScaleTimeWorldToGE(elapsedWorldTime);
                foreach (GSParticles gsp in particleSystems) {
                    gsp.Evolve(evolvedPhyTime, massiveIndices, ref ge.PhysicsJob());
                }
            }
            gsd.ParticlesUpdate(orbitScale);
        }


        void LateUpdate()
        {
            if (paused)
                return;

            if (evolveMode != EvolveMode.IMMEDIATE) {
                ge.Complete();
                lastUnityEvolveTime = Time.timeSinceLevelLoadAsDouble;
                //           if (haveParticles && IJOB_LATE_PARTICLES mode) {
                //foreach (JobHandle jh in particleJobHandles) {
                //	jh.Complete();
                //}
                //foreach (GSDisplay gsd in sceneDisplays) {
                //	foreach (GSParticles gsp in gsd.ParticleSystems()) {
                //		if (gsp.pJob.massiveBodies.IsCreated) {
                //			gsp.pJob.massiveBodies.Dispose();
                //			break;
                //		}
                //	}
                //	gsd.ParticlesUpdate(orbitScale);
                //               }
                //           }
                foreach (GSDisplay gsd in sceneDisplays) {
                    gsd.DisplayUpdate(ge, elapsedWorldTime);
                }

            }
        }

        private void OnDestroy()
        {
            // Need to release the arrays in gravitySystem
            if (ge != null) {
                ge.Dispose();
            }
        }

        /// <summary>
        /// Retrieve the current time zoom level and associated zoom factor. 
        /// </summary>
        /// <returns></returns>
        public (int, float) TimeZoomAndFactor()
        {
            return (timeZoom, timeZoomFactor[timeZoom]);
        }

        /// <summary>
        /// Return the current simulation time in time units for the chosen 
        /// default world units.
        /// </summary>
        /// <returns></returns>
        public double WorldTime()
        {
            return elapsedWorldTime;
        }

        /// <summary>
        /// Return the current world time expressed as a Juliam date. 
        /// </summary>
        /// <returns></returns>
        public double JDTime()
        {
            return startTimeJD + TimeUtils.SecToJD(elapsedWorldTime);
        }

        public double StartTimeJD()
        {
            return startTimeJD;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="gsc"></param>
        public void PrimaryControllerSet(GSController gsc)
        {
            primaryController = gsc;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public GSController PrimaryController()
        {
            return primaryController;
        }

        public double WorldTimePerGameSec()
        {
            return worldTimePerGameSec;
        }

        //######## GEListenerIF ###########

        /// <summary>
        /// 
        /// </summary>
        /// <param name="id"></param>
        /// <param name="ge"></param>
        /// <param name="pEvent"></param>
        void GEListenerIF.BodyRemoved(int id, GECore ge, GEPhysicsCore.PhysEvent pEvent)
        {
            Debug.LogFormat("Body {0} removed event={1}", id, pEvent.type);
            // remove body and tell scene displays about this
            foreach (GSDisplay gsd in sceneDisplays) {
                gsd.DisplayBodyRemoveByBodyId(id);
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="id"></param>
        /// <param name="ge"></param>
        /// <param name="pEvent"></param>
        void GEListenerIF.PhysicsEvent(int id, GECore ge, GEPhysicsCore.PhysEvent pEvent)
        {
            switch (pEvent.type) {
                case GEPhysicsCore.EventType.KEPLER_ERROR:
                    Debug.LogErrorFormat("Kepler error for bodyId={0} around {1}. Time before t0", pEvent.bodyId, pEvent.bodyId_secondary);
                    break;
            }
        }

#if UNITY_EDITOR
        public GSBody[] EditorGSBodies()
        {
            GSBody[] toAdd = null;
            switch (addMode) {
                case AddMode.CHILDREN:
                    toAdd = gameObject.GetComponentsInChildren<GSBody>();
                    break;

                case AddMode.LIST:
                    toAdd = addBodies;
                    break;

                case AddMode.SCENE:
                    toAdd = FindObjectsByType<GSBody>(FindObjectsSortMode.None);
                    break;
            }

            return toAdd;
        }

        public double EditorLatestSGP4Epoch()
        {
            double startJD = 0;
            GSBody[] bodies = EditorGSBodies();
            foreach (GSBody body in bodies) {
                if (body.bodyInitData.initData == BodyInitData.InitDataType.TWO_LINE_ELEMENT) {
                    startJD = System.Math.Max(startJD, body.bodyInitData.SGP4StartEpoch());
                }
            }
            return startJD;
        }


        /// <summary>
        /// Scan the objects based on the Add Mode and find max mass in the scene and the average
        /// orbital size.
		///
		/// This also sets the values in the controller.
		/// 
        /// </summary>
        public (double mass, double orbit_size) EditorAutoscale()
        {
            GSBody[] toAdd = EditorGSBodies(); ;
            double mass = 0;
            double orbit_size_sum = 0.0;
            double num_orbits = 0.0;
            foreach (GSBody gsBody in toAdd) {
                mass = Math.Max(mass, gsBody.mass);
                if (gsBody.centerBody != null) {
                    num_orbits += 1.0;
                    orbit_size_sum += gsBody.bodyInitData.EditorGetSize();
                }
            }
            orbitMass = mass;
            double orbit_size = 1.0;
            if (num_orbits > 0) {
                orbit_size = orbit_size_sum / num_orbits;
            }
            orbitScale = orbit_size;
            Debug.LogFormat("Autoscale {0} => mass={1} orbitSize={2} numOrbits={3}",
                gameObject.name, orbitMass, orbit_size, num_orbits);
            return (orbitMass, orbit_size);
        }

        /// <summary>
        /// For scene display positioning of objects, need to determine the world position of the requested gsBody.
        ///
        /// This can get a bit elaborate, since the body may have an orbit defining where it is and the center
        /// of the orbit may itself be in orbit and so on.
        ///
        /// Since we're in the editor and have not yet started, we need to figure this all out "on the fly".
        /// 
        /// </summary>
        /// <param name="gsBody"></param>
        /// <returns></returns>
        public static Vector3 EditorPosition(GSBody gsBody, GSController gsc)
        {
            if (gsBody == null)
                return Vector3.zero;
            // TODO: Handle non-child mode
            if (gsc == null) {
                gsc = GetGSCForBody(gsBody);
            }
            double lenScale = GBUnits.DistanceConversion(gsBody.bodyInitData.units, gsc.defaultUnits, gsc.EditorCR3BPSysData());
            Vector3 pos;
            if (gsBody.bodyInitData.initData != BodyInitData.InitDataType.RV_ABSOLUTE) {
                pos = (float)lenScale * gsBody.EditorPositionRelative();
                pos += EditorPosition(gsBody.centerBody, gsc);
            } else {
                pos = GravityMath.Double3ToVector3(lenScale * gsBody.bodyInitData.r);
            }
            return pos;
        }

        public static GSController GetGSCForBody(GSBody gsBody)
        {
            // TODO: Non- child mode
            GSController gsc = null;
            Transform transform = gsBody.transform;
            while (transform.parent != null) {
                transform = transform.parent;
                gsc = transform.GetComponent<GSController>();
                if (gsc != null)
                    break;
            }
            return gsc;
        }

        public double EditorGetGWorldUnits()
        {
            return GBUnits.GForUnits(defaultUnits);
        }

        public double EditorComputeDt()
        {
            // so far with units all in seconds, this is just one
            double dt = 2.0 * math.PI / stepsPerOrbit;
            return dt;
        }

        public CR3BP.CR3BPSystemData EditorCR3BPSysData()
        {
            return CR3BP.SystemData(cr3bpSystem);
        }
#endif


    }
}
