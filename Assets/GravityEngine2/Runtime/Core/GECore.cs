using System;
using System.Collections.Generic;
using Unity.Collections;    // Require Collections from the Package manager
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

/*! \mainpage Gravity Engine 2
 *
 * \section intro_sec Introduction
 *
 * This is the Doxygen code documentation for the Runtime elements of GE2. 
 *
 * All GE2 code is in the namespace GravityEngine2.
 *
 * There is additional code in the Editor/ and Samples/ directory that is not indexed here to 
 * keep this documentation lookup does not get too cluttered.
 *
 * \section install_sec User Documentation
 *
 * Online user documentation can be found at: 
 * https://nbodyphysics.com/ge2/html/intro.html
 *
 * Blog: https://nbodyphysics.com
 *
 * Support: nbodyphysics@gmail.com
 */

namespace GravityEngine2 {
    /// <summary>
    /// Struct to hold the state information of a body being evolved in the GECore. Typically the information
    /// is in world units (unless the request to GEcore.StateById() specifically asked for GE internal units).
    /// 
    /// The values are typically converted to world units automatically when requested from 
    /// GECore. 
    /// 
    /// </summary>
    public struct GEBodyState {
        public double3 r;
        public double3 v;
        public double t;

        public GEBodyState(double3 r, double3 v, double t = 0.0)
        {
            this.r = r;
            this.v = v;
            this.t = t;
        }

        public Vector3 RasVector3()
        {
            return new Vector3((float)r.x, (float)r.y, (float)r.z);
        }

        public bool HasNaN()
        {
            return double.IsNaN(r.x) || double.IsNaN(r.y) || double.IsNaN(r.z) ||
                   double.IsNaN(v.x) || double.IsNaN(v.y) || double.IsNaN(v.z) ||
                   double.IsNaN(t);
        }

        public string LogString()
        {
            return $"r={r} v={v} t={t}";
        }
    }

    public struct PatchData {
        public double tStartWorld;
        public double tEndWorld;
        public GEPhysicsCore.Propagator propType;

        public int centerId;

        public string LogString()
        {
            return $"tStartWorld={tStartWorld} tEndWorld={tEndWorld} propType={propType}";
        }
    }

    /// <summary>
    /// Gravity Engine Core
    /// Central class for evolution using gravity and propagators. 
    /// 
    /// Handles:
    /// - evolution directly or as a job
    /// - management of physics data structures for bodies, maneuvers, patches etc. 
    /// - handles body additions with a variety of initial conditions (RV, COE, ephemeris, TLE etc.)
    /// - the physics evolution is handed to @see GEPhysicsCore
    /// 
    /// This class can be used directly or @see GSController will instantiate a instance.
    /// 
    /// Since execution may be via the Unity job system a large part of what GECore does is to arrange
    /// data into arrays suitable for use in the job system. This aids in memory locality and speed. Hence
    /// there are often parallel data stores for info in GECore as lists and then an array version of the 
    /// info for GEPhysicsCore. 
    /// 
    /// </summary>
    public class GECore {
        private bool DEBUG = true;

        public Integrators.Type integrator;

        private NativeArray<double> gePhysConfig;
        // Array for entries in gePhysConfig 
        public const int DT_PARAM = 0;      // integration time step
        public const int T_PARAM = 1;       // current time in GE time units
        public const int T_END_PARAM = 2;   // end time in GE time units
        public const int T_END_JD = 3;      // end time in Julian date for SGP4
        public const int TRAJ_INDEX_PARAM = 4;  // live index for current location in recordedOutput
        public const int TRAJ_NUMSTEPS_PARM = 5;    // max size of recorded output buffer in traj mode
        public const int TRAJ_NUM_UPDATED_PARAM = 6; // number of entries updated on most recent call
        public const int TRAJ_INTERVAL_PARAM = 7;
        public const int START_TIME_JD = 8;
        public const int NUM_PARAMS = 9;

        private GEPhysicsCore.GEBodies bodies;

        private GEPhysicsCore.GEPhysicsJob gePhysicsJob;

        public enum ReferenceFrame {
            INERTIAL,   // default
            COROTATING_CR3BP,  // Circular Restricted Three Body Problem
        }

        private ReferenceFrame refFrame = ReferenceFrame.INERTIAL;

        /// <summary>
		/// freeBodies hold the integer values of empty slots in the bodies array.
		/// It is a Stack() so that single threaded controller code can remove a
		/// body and then re-add it with different details (propagator etc.) and be
		/// assured tha the re-add will get the same body ID. This ensures that GSBody
		/// and display body references stay the same.
		/// 
		/// </summary>
        private Stack<int> freeBodies;  // free entries in the bodies array

        // Index lists
        // Each type of body has an index list. This is the index in the bodies array of the state info
        // for a given body.
        //
        // GE maintains as a List object but the twin in GEPhysicsJob is a simple NativeArray<int>
        // index arrays in List form for easy add/del. Copied to arrays in GEPhysicsJob
        private List<int> colliders;
        private List<int> extAccels;


        public const double DT_DEFAULT = 2.0 * math.PI / 500.0; // 500 steps per orbit
        private double dt = DT_DEFAULT;

        //! Flag that indicates GE is running
        private bool running;

        private bool verbose = true;

        /// <summary>
        /// The most flexible way to ask GE to do things is via a callback when the physics loop
        /// is complete. This allows the scene to toggle between IMMEDIATE modes and job mode
        /// without code changes. When in immediate mode the callbacks are not stored and immediatly
        /// run. 
        ///
        /// This does result in unnecessary indirection when using only IMMEDIATE mode, so it can
        /// be skipped if IMMEDIATE mode will not be changed during development.
        /// 
        /// </summary>
        /// <param name="ge"></param>
        /// <param name="arg"></param>

        public delegate void PhysLoopCompleteCallback(GECore ge, object arg = null);

        private struct PhysLoopCompleteCB {
            public PhysLoopCompleteCallback cb;
            public object arg;

            public PhysLoopCompleteCB(PhysLoopCompleteCallback cb, object arg = null)
            {
                this.cb = cb;
                this.arg = arg;
            }
        }
        private List<PhysLoopCompleteCB> physLoopCompleteCallbacks;

        private int next_maneuver_id = 0;

        // Handle for Nbody and Kepler chained together (Kepler needs Nbody result)
        private JobHandle jobHandle;

        // number of bodies
        private int numBodies = 0;

        private double t_physics;

        // Manage Kepler, Pkepler and SGP4 prop info within GEPhysicsCore
        private StructPool<KeplerPropagator.PropInfo> keplerPool;
        private StructPool<PKeplerPropagator.PropInfo> pkeplerPool;
        private StructPool<KeplerCOEPropagator.PropInfo> keplerCoePool;
        private StructPool<SGP4Propagator.PropInfo> sgp4Pool;
        private StructPool<EphemerisPropagator.PropInfo> ephemPool;

        private StructPool<RotationPropagator.PropInfo> rotationPool;

        // Patch info
        private StructPool<GEPhysicsCore.PatchInfo> patchPool;

        // Collsion Info
        private StructPool<GEPhysicsCore.ColliderInfo> collisionInfoPool;

        // External Accel Info (can have more than one per body!)
        private StructPool<ExternalAccel.EADesc> extAccelPool;

        /// <summary>
        /// A GE may have a clone that handles trajectory prediction. This clone computes the future path in
        /// a circular record buffer. Under certain circumstances (maneuvers, setting state, add/remove) the
        /// trajectory clone will need to restart from the current state of the source.
        /// </summary>
        private GECore geTrajectory;

        // TRAJECTORY
        // In trajectory mode the GE will run from the current time to (time + trajAhead) recording output
        // at intervals specified by timeInterval. When first evolved it will do this in a burst to get all the
        // way to (time + timeAhead). Once there it will evolve incrementally into the future and use the recorded
        // output as a wrap-around buffer.
        //
        // currentEntry is last entry written to
        //
        // If the state of the reference system changes (e.g. a ship does a maneuver) then the entire future path
        // is recomputed. Note that for objects using deterministic propagators it is better to use the propagator
        // directly to determine the future path of the object. Trajectories are best used when there are N-body
        // graviational effects.

        private bool trajectory;    // this is the GE that is handling the traj.
        private double traj_timeAheadGE;
        private double traj_timeIntervalGE;
        // flag set in parent if need to restart traj before next Evolve/Schedule starts
        private bool trajReset;

        public struct GEConfig {
            public int MAX_BODIES;      // Initial size of bodies array
            public int GROW_BY;         // Size to increase bodies by
            public int MAX_MANEUVERS;   // Initial size of maneuver list
            public int MAX_EXTACCEL;
            public bool isCreated;
            public ReferenceFrame refFrame;

            // Collision event reporting: Can screen events in GEPhysicsCore to reduce event spam on each RunStep by
            // ensuring each unique collision is reported only once. This is enabled by default
            public bool uniqueCollisionEvents;

            public double DT;
            public double scaleLtoKm;   // factor to scale internal GE length to Km (for SGP4, PKEPLER esp.)
            public double scaleTtoSec;   // factor to scale internal time to sec (for SGP4, PKEPLER esp.)
            public double scaleKmsecToV;
            public double startTimeJD;  // start time in Julian Date

            // trajectory mode

            public GEConfig(double dt, double scaleLtoKm = 1.0, double scaleKmsecToV = 1.0, double scaleTtoSec = 1.0)
            {
                MAX_BODIES = 64;
                GROW_BY = 32;
                MAX_MANEUVERS = 32;
                MAX_EXTACCEL = 32;
                isCreated = true;
                DT = dt;
                refFrame = ReferenceFrame.INERTIAL;
                this.scaleLtoKm = scaleLtoKm;
                this.scaleTtoSec = scaleTtoSec;
                this.scaleKmsecToV = scaleKmsecToV;
                startTimeJD = 0.0;
                uniqueCollisionEvents = true;
            }
        }
        private GEConfig geConfig;
        private GEListenerIF geListener;

        private List<GEManeuverStruct> masterManeuverList;
        // Keep a copy of GEManeuver OBJECT until it executes to allow for callbacks because struct
        // in gePhysicsJob cannot hold a reference to them (non Native)
        // (these are AFTER the timeslice where the maneuver was executed and are not "live")
        private List<GEManeuver> pendingManeuvers;

        private List<PhysicsEventCallback> physEventListeners;

        private bool preCalcDone;

        private GBUnits.GEScaler geScaler;

        /// <summary>
        /// Construct a GECore to evolve bodies. The majority of the parameters relate to the scaling to be configured
        /// for numerical integration. Generally the information in/out is expressed in the defaultUnits (world units). 
        /// The numerical integration is best when "typical values" of mass and length are of order 1. Internally 
        /// physical parameters are scaled to obtain this. 
        /// 
        /// </summary>
        /// <param name="integrator">specify numerical inegration scheme</param>
        /// <param name="defaultUnits"></param>
        /// <param name="lengthScale">length in world units that will be 1 physics unit</param>
        /// <param name="massScale">mass in world units that will be 1 mass unit</param>
        /// <param name="stepsPerOrbit">suggested number of time steps per circular orbit of radius lengthScale</param>
        /// <param name="startTimeJD">start time in Julian date format (only needed when satellite info in TLEs)</param>
        public GECore(Integrators.Type integrator,
                            GBUnits.Units defaultUnits,
                            double lengthScale,
                            double massScale,
                            double stepsPerOrbit,
                            double startTimeJD = 0 // only needed if using SGP4
                            )
        {
            refFrame = ReferenceFrame.INERTIAL;
            this.integrator = integrator;
            geScaler = new GBUnits.GEScaler(defaultUnits, massScale, lengthScale);
            // SI scaling (GE needs this if using PKEPLER or SGP4 props)
            double scaleLtoKm = GBUnits.DistanceConversion(defaultUnits, GBUnits.Units.SI_km) * lengthScale;
            // for now all world units have second as as their time base
            double scaleTtoSec = geScaler.ScaleTimeGEToWorld(1.0);
            double scaleKmsecToV = geScaler.ScaleVelocityWorldToGE(1.0) *
                            GBUnits.DistanceConversion(GBUnits.Units.SI_km, defaultUnits);
            dt = 2.0 * math.PI / stepsPerOrbit;
            geConfig = new GEConfig(dt, scaleLtoKm, scaleKmsecToV, scaleTtoSec) {
                startTimeJD = startTimeJD,
            };

            gePhysicsJob = new GEPhysicsCore.GEPhysicsJob();
            gePhysicsJob.Init(geConfig, integrator);
            BodiesSetup(geConfig);

            // time
            gePhysicsJob.parms[DT_PARAM] = dt;
            gePhysicsJob.parms[T_PARAM] = 0.0;
            gePhysicsJob.parms[T_END_PARAM] = 0.0;
            gePhysicsJob.parms[START_TIME_JD] = geConfig.startTimeJD;
            gePhysConfig = gePhysicsJob.parms;
            t_physics = 0.0;

            physLoopCompleteCallbacks = new List<PhysLoopCompleteCB>();

            physEventListeners = new List<PhysicsEventCallback>();

            masterManeuverList = new List<GEManeuverStruct>();
            pendingManeuvers = new List<GEManeuver>();

            running = false;

            gePhysicsJob.ModeSet(GEPhysicsCore.ExecuteMode.NORMAL);
#pragma warning disable 162     // disable unreachable code warning
            if (DEBUG) {
                Debug.LogFormat("GE INIT: dt={0:F6} worldU={1} orbitScale={2} massScale={3}",
                    dt, defaultUnits, lengthScale, massScale);
            }
#pragma warning restore 162     // apply an impulse to the indicated NBody
        }

        /// <summary>
        /// Construct a GECore to perform evolution in the circular-restricted three body
        /// coordinate system (CR3BP). In this system the primary and secondary bodies are
        /// fixed and the equations of evolution model gravity and include terms to 
        /// account for the rotating frame. 
        /// 
        /// This constructor explcitly adds the primary and secondary bodies based on the 
        /// information in the cr3bpSystem parameter. 
        /// 
        /// </summary>
        /// <param name="integrator"></param>
        /// <param name="defaultUnits"></param>
        /// <param name="cr3bpSystem"></param>
        /// <param name="stepsPerOrbit"></param>
        /// <param name="startTimeJD"></param>
        /// <exception cref="NotSupportedException"></exception>
        public GECore(Integrators.Type integrator,
                        GBUnits.Units defaultUnits,
                        CR3BP.CR3BPSystemData cr3bpSystem,
                        double stepsPerOrbit,
                        double startTimeJD)
        {
            refFrame = ReferenceFrame.COROTATING_CR3BP;
            // TODO: think about steps per orbit!! Check this is working ok
            this.integrator = integrator;
            // assume SI_km, then adjust
            double massScale = cr3bpSystem.totalMass;
            if (defaultUnits != GBUnits.Units.CR3BP_SCALE) {
                throw new NotSupportedException("TODO: Init CR3BP with non-CR3BP units");
            }
            geScaler = new GBUnits.GEScaler(defaultUnits, cr3bpSystem);
            // SI scaling (GE needs this if using PKEPLER or SGP4 props)
            double scaleLtoKm = GBUnits.DistanceConversion(defaultUnits, GBUnits.Units.SI_km, cr3bpSystem);
            // for now all world units have second as as their time base
            double scaleTtoSec = geScaler.ScaleTimeGEToWorld(1.0);
            // BUG???
            double scaleKmsecToV = geScaler.ScaleVelocityWorldToGE(1.0) *
                            GBUnits.DistanceConversion(GBUnits.Units.SI_km, defaultUnits, cr3bpSystem);
            dt = 2.0 * math.PI / stepsPerOrbit;
            geConfig = new GEConfig(dt, scaleLtoKm, scaleKmsecToV, scaleTtoSec) {
                startTimeJD = startTimeJD,
                refFrame = refFrame
            };
            // Setup internals
            gePhysicsJob = new GEPhysicsCore.GEPhysicsJob();
            gePhysicsJob.Init(geConfig, integrator);
            BodiesSetup(geConfig);

            // add primary and secondary as fixed bodies (they will be implicit in the CR3BP integrator)
            // Integrator will assume primary has bodyId=0, secondary=1
            //
            // GSController will NOT add these as part of it's normal body sequence. 
            double mu = cr3bpSystem.massRatio;
            // normalize to 1 unit separation and total mass=1
            double3 r_p = new double3(-mu, 0, 0);
            double3 r_s = new double3(1.0 - mu, 0, 0);
            int id0 = BodyAdd(r_p, double3.zero, 1.0 - mu, isFixed: true);
            int id1 = BodyAdd(r_s, double3.zero, mu, isFixed: true);
            // be paranoid
            if ((id0 != 0) || (id1 != 1)) {
                Debug.LogError("Did not get expected body ids. Internal error");
                return;
            }

            // time
            gePhysicsJob.parms[DT_PARAM] = dt;
            gePhysicsJob.parms[T_PARAM] = 0.0;
            gePhysicsJob.parms[T_END_PARAM] = 0.0;
            gePhysicsJob.parms[START_TIME_JD] = geConfig.startTimeJD;
            gePhysConfig = gePhysicsJob.parms;

            physLoopCompleteCallbacks = new List<PhysLoopCompleteCB>();

            physEventListeners = new List<PhysicsEventCallback>();

            masterManeuverList = new List<GEManeuverStruct>();
            pendingManeuvers = new List<GEManeuver>();

            running = false;

            gePhysicsJob.ModeSet(GEPhysicsCore.ExecuteMode.NORMAL);
#pragma warning disable 162     // disable unreachable code warning
            if (DEBUG) {
                Debug.LogFormat("GE INIT: dt={0:F6} worldU={1} cr3bp={2} massScale={3}",
                    dt, defaultUnits, cr3bpSystem.name, massScale);
            }
#pragma warning restore 162     // apply an impulse to the indicated NBody
        }

        /// <summary>
        /// COPY CONSTRUCTOR
        /// Create a new GE as a clone of an existing GE.
        /// - the clone will have the same id mapping for bodies and internal structure
        /// - NotRunningCallbacks may optionally NOT be copied (since likely want to attach new ones)
        /// 
        /// </summary>
        /// <param name="ge"></param>
        /// <param name="copyCallbacks"></param>
        public GECore(GECore ge, bool copyCallbacks = false)
        {
            geConfig = ge.geConfig; // struct so makes a copy
            geScaler = ge.geScaler; // copy
            this.dt = geConfig.DT;

            gePhysicsJob = new GEPhysicsCore.GEPhysicsJob();
            gePhysicsJob.InitFrom(ge.gePhysicsJob);
            bodies = gePhysicsJob.bodies;
            gePhysConfig = gePhysicsJob.parms;
            CopyPoolSetup(ge);

            colliders = new List<int>(ge.colliders);
            extAccels = new List<int>(ge.extAccels);

            freeBodies = new Stack<int>(ge.freeBodies);

            refFrame = ge.refFrame;

            physLoopCompleteCallbacks = new List<PhysLoopCompleteCB>();
            if (copyCallbacks) {
                foreach (PhysLoopCompleteCB plc in ge.physLoopCompleteCallbacks)
                    physLoopCompleteCallbacks.Add(plc);
            }
            physEventListeners = new List<PhysicsEventCallback>();

            masterManeuverList = new List<GEManeuverStruct>();
            foreach (GEManeuverStruct gms in ge.masterManeuverList)
                masterManeuverList.Add(gms);
            pendingManeuvers = new List<GEManeuver>();
            foreach (GEManeuver gem in ge.pendingManeuvers)
                pendingManeuvers.Add(gem);

            running = false;
        }

        /// Get the GEScalar. Used when code needs to determine conversions from physics space (GEC internal)
        /// to world units.
        /// </summary>
        /// <returns>GEScalar</returns>
        public GBUnits.GEScaler GEScaler()
        {
            return geScaler;
        }

        private void BodiesSetup(GEConfig config)
        {
            bodies = gePhysicsJob.bodies;

            keplerPool = new StructPool<KeplerPropagator.PropInfo>();
            gePhysicsJob.keplerPropInfo = new NativeArray<KeplerPropagator.PropInfo>(config.MAX_BODIES, Allocator.Persistent);
            keplerPool.Init(gePhysicsJob.keplerPropInfo, growBy: config.GROW_BY);

            pkeplerPool = new StructPool<PKeplerPropagator.PropInfo>();
            gePhysicsJob.pKeplerPropInfo = new NativeArray<PKeplerPropagator.PropInfo>(config.MAX_BODIES, Allocator.Persistent);
            pkeplerPool.Init(gePhysicsJob.pKeplerPropInfo, growBy: config.GROW_BY);

            keplerCoePool = new StructPool<KeplerCOEPropagator.PropInfo>();
            gePhysicsJob.keplerCoePropInfo = new NativeArray<KeplerCOEPropagator.PropInfo>(config.MAX_BODIES, Allocator.Persistent);
            keplerCoePool.Init(gePhysicsJob.keplerCoePropInfo, growBy: config.GROW_BY);

            sgp4Pool = new StructPool<SGP4Propagator.PropInfo>();
            gePhysicsJob.sgp4PropInfo = new NativeArray<SGP4Propagator.PropInfo>(config.MAX_BODIES, Allocator.Persistent);
            sgp4Pool.Init(gePhysicsJob.sgp4PropInfo, growBy: config.GROW_BY);

            patchPool = new StructPool<GEPhysicsCore.PatchInfo>();
            patchPool.Init(gePhysicsJob.patches, config.GROW_BY);

            collisionInfoPool = new StructPool<GEPhysicsCore.ColliderInfo>();
            collisionInfoPool.Init(gePhysicsJob.collisionInfo, config.GROW_BY);

            ephemPool = new StructPool<EphemerisPropagator.PropInfo>();
            gePhysicsJob.ephemPropInfo = new NativeArray<EphemerisPropagator.PropInfo>(config.MAX_BODIES, Allocator.Persistent);
            ephemPool.Init(gePhysicsJob.ephemPropInfo, config.GROW_BY);

            rotationPool = new StructPool<RotationPropagator.PropInfo>();
            gePhysicsJob.rotationPropInfo = new NativeArray<RotationPropagator.PropInfo>(config.MAX_BODIES, Allocator.Persistent);
            rotationPool.Init(gePhysicsJob.rotationPropInfo, config.GROW_BY);

            extAccelPool = new StructPool<ExternalAccel.EADesc>();
            extAccelPool.Init(gePhysicsJob.extADesc, config.GROW_BY);

            freeBodies = new Stack<int>(bodies.r.Length);
            // keep low IDs at the top of the stack
            for (int i = bodies.r.Length - 1; i >= 0; i--)
                freeBodies.Push(i);

            colliders = new List<int>();
            extAccels = new List<int>();
        }

        private void CopyPoolSetup(GECore ge)
        {
            keplerPool = new StructPool<KeplerPropagator.PropInfo>(ge.keplerPool);
            pkeplerPool = new StructPool<PKeplerPropagator.PropInfo>(ge.pkeplerPool);
            keplerCoePool = new StructPool<KeplerCOEPropagator.PropInfo>(ge.keplerCoePool);
            sgp4Pool = new StructPool<SGP4Propagator.PropInfo>(ge.sgp4Pool);
            ephemPool = new StructPool<EphemerisPropagator.PropInfo>(ge.ephemPool);
            rotationPool = new StructPool<RotationPropagator.PropInfo>(ge.rotationPool);
            patchPool = new StructPool<GEPhysicsCore.PatchInfo>(ge.patchPool);
            collisionInfoPool = new StructPool<GEPhysicsCore.ColliderInfo>(ge.collisionInfoPool);
            extAccelPool = new StructPool<ExternalAccel.EADesc>(ge.extAccelPool);
        }

        private int AllocBodyIndex()
        {
            if (freeBodies.Count <= 0)
                GrowBodies();
            return freeBodies.Pop();
        }

        /// <summary>
        /// Add a listener.
        ///
        /// Currently used to get a callback when a body is removed due to a collision.
        /// 
        /// </summary>
        /// <param name="geListener"></param>
        public void ListenerAdd(GEListenerIF geListener)
        {
            this.geListener = geListener;
        }

        public GECore GeTrajectory()
        {
            return geTrajectory;
        }

        public static bool PropAllowsEpochTime(GEPhysicsCore.Propagator propagator)
        {
            switch (propagator) {
                case GEPhysicsCore.Propagator.KEPLER:
                case GEPhysicsCore.Propagator.KEPLER_COE:
                case GEPhysicsCore.Propagator.PKEPLER:
                case GEPhysicsCore.Propagator.SGP4_RAILS:
                    return true;
                default:
                    return false;
            }
        }

        /// <summary>
        /// Configure this GE to evolve forward by "timeAhead" recording positions at intervals of
        /// timeAhead/numSteps.
        ///
        /// This is also used when a new body is added: build a new list of bodiesToRecord and run a new setup.
        /// [A trajectory reset is needed and more memory needs to be allocated for the circular buffer so
        /// there is simple way to 'just' do an add.]
        ///
        /// Typical usage:
        ///     1) Clone an existing GE (into trajClone, say) and set the clone with ConfigTrajectory
        ///     2) Register the trajectory clone with the intial GE so that if there are changes (body add/del etc.)
        ///        the clone can recompute it's trajectories
        ///     3) Have the display/evolve code run the evolution for trajClone the same way it does for GE
        ///     4) Display code then retrieves the update position info from trajClone and displays
        ///     
        /// </summary>
        /// <param name="t_ahead">time into future to determine trajectoories</param>
        /// <param name="numSteps">number of intermediate values to asses</param>
        /// <param name="bodiesToRecord">indices of bodies to gather trajectory information for</param>
        public void TrajectorySetup(double t_ahead, int numSteps, List<int> bodiesToRecord)
        {
            if (geTrajectory != null) {
                geTrajectory.Dispose();
            }
            geTrajectory = new GECore(this, copyCallbacks: false);
            geTrajectory.trajectory = true;
            geTrajectory.TrajectorySetupInternal(t_ahead, numSteps, bodiesToRecord);
            // source needs these to do evolve properly
            if (DEBUG) {
                Debug.LogFormat("Trajectory setup: t_start={0:G7} t_end={1:G7} t_ahead={2:G7}",
                    geTrajectory.gePhysConfig[T_PARAM], geTrajectory.gePhysConfig[T_END_PARAM], geTrajectory.traj_timeAheadGE);
            }
        }

        /// <summary>
        /// Configure the GECore for trajectory recording. This is called by the parent GECore instance.
        /// BE AWARE that the gePhyConfig here is a COPY of the parent GE's gePhysConfig.
        /// </summary>
        /// <param name="t_ahead"></param>
        /// <param name="numSteps"></param>
        /// <param name="bodiesToRecord"></param>
        private void TrajectorySetupInternal(double t_ahead, int numSteps, List<int> bodiesToRecord)
        {
            traj_timeAheadGE = geScaler.ScaleTimeWorldToGE(t_ahead);
            traj_timeIntervalGE = traj_timeAheadGE / numSteps;
            gePhysicsJob.ModeSet(GEPhysicsCore.ExecuteMode.TRAJECTORY);
            gePhysConfig[TRAJ_NUMSTEPS_PARM] = numSteps;
            gePhysConfig[TRAJ_INDEX_PARAM] = numSteps - 1; // last entry written, want to wrap on startup
            gePhysConfig[TRAJ_INTERVAL_PARAM] = traj_timeIntervalGE;

            // setup recordBodies
            if (gePhysicsJob.recordBodies.IsCreated)
                gePhysicsJob.recordBodies.Dispose();
            gePhysicsJob.recordBodies = new NativeArray<int>(bodiesToRecord.Count, Allocator.Persistent);
            for (int i = 0; i < bodiesToRecord.Count; i++) {
                gePhysicsJob.recordBodies[i] = bodiesToRecord[i];
            }
            gePhysicsJob.PrepareForRecording(recordLoop: true);
        }


        /// <summary>
        /// Remove a trajectory from the trajectory recording. This will not require a trajectory reset
        /// so it can be done without too much fuss.
        /// 
        /// </summary>
        /// <param name="bodyId"></param>
        public void TrajectoryRemove(int bodyId)
        {
            if (geTrajectory != null) {
                if (!geTrajectory.TrajectoryInternalRemove(bodyId)) {
                    // no trajectories left
                    Debug.Log("Dispose");
                    geTrajectory.Dispose();
                    geTrajectory = null;
                }
            }
            // Depending on Dispose order may get a case where asked to remove traj from already disposed geTraj
        }

        /// <summary>
        /// Remove the indicate bodyId for trajectories and compress existing data with
        /// resetting the traj evolution. 
        /// </summary>
        /// <param name="bodyId"></param>
        /// <returns>flag indicating still have trajectories</returns>
        private bool TrajectoryInternalRemove(int bodyId)
        {
            if (gePhysicsJob.recordBodies.Length > 1) {
                // can compress the existing array and shuffle the data down
                // recordTimes are not affected
                int numSteps = (int)gePhysConfig[TRAJ_NUMSTEPS_PARM];

                List<int> newBodies = new List<int>(gePhysicsJob.recordBodies);
                int removeIndex = newBodies.IndexOf(bodyId);
                newBodies.Remove(bodyId);
                int numBodies = newBodies.Count;
                NativeArray<int> oldRecordBodies = gePhysicsJob.recordBodies;
                NativeArray<GEBodyState> oldRecordedState = gePhysicsJob.recordedState;

                // new record bodies array
                gePhysicsJob.recordBodies = new NativeArray<int>(numBodies, Allocator.Persistent);
                for (int i = 0; i < numBodies; i++) {
                    gePhysicsJob.recordBodies[i] = newBodies[i];
                }

                // new recorded state, but copy the state values to avoid a recompute of all
                // trajectories
                gePhysicsJob.recordedState = new NativeArray<GEBodyState>(
                        numSteps * numBodies, Allocator.Persistent);
                int k;
                for (int i = 0; i < numSteps; i++) {
                    for (int j = 0; j < newBodies.Count; j++) {
                        k = j;
                        if (j >= removeIndex)
                            k = j + 1;
                        gePhysicsJob.recordedState[i * numBodies + j] =
                            oldRecordedState[i * (numBodies + 1) + k];
                    }
                }
                oldRecordBodies.Dispose();
                oldRecordedState.Dispose();
                return true;
            }
            return false;
        }

        /// <summary>
		/// Called for the GE instance that is the parent of a trajectory.
		///
		/// Basic idea is to ditch the existing recorded paths (by resetting the write pointer and the
		/// time value) to force a recompute of the paths from the current parent GE state.
		///
		/// The current GE body state is copied across. Propagators are assumed to be "history free" so they
		/// do not need to be reset. 
		///
		/// As the trajectory "ran ahead" it may have consumed maneuvers or switched to new patches, hence
		/// a need to re-copy those from the parent GE state.
		/// </summary>
		/// <param name="geParent"></param>
        private void TrajectoryReset()
        {
            // grab config info from previous trajectory GECore
            int numSteps = (int)geTrajectory.gePhysConfig[TRAJ_NUMSTEPS_PARM];
            double tAhead = geScaler.ScaleTimeGEToWorld(geTrajectory.traj_timeAheadGE);
            List<int> bodiesToRecord = new List<int>(geTrajectory.gePhysicsJob.recordBodies);
            TrajectorySetup(tAhead, numSteps, bodiesToRecord);
            trajReset = false;
#pragma warning disable 162     // disable unreachable code warning
            if (DEBUG) {
                Debug.Log(geTrajectory.DumpAll("Trajectory reset: GETrajectory =>"));
            }
#pragma warning restore 162 
        }

        /// <summary>
        /// Number of bodies active in recording function. 
        /// </summary>
        /// <returns></returns>
        public int NumRecordedBodies()
        {
            return gePhysicsJob.recordBodies.Length;
        }

        /// <summary>
        /// Numerical integration timestep value.
        /// </summary>
        /// <returns></returns>
        public double Dt()
        {
            return dt;
        }

        /// <summary>
        /// Configure the internal timestep used in numerical integration. 
        /// </summary>
        /// <param name="dt"></param>
        public void DtSet(double dt)
        {

            this.dt = dt;
            geConfig.DT = dt;
            gePhysicsJob.parms[DT_PARAM] = dt;
#pragma warning disable 162     // disable unreachable code warning
            if (DEBUG) {
                Debug.LogFormat("Set DT to {0}", dt);
            }
#pragma warning restore 162     // apply an impulse to the indicated NBody
        }

        /// <summary>
        /// Time in internal GECore units.
        /// 
        /// When GEC is configured for ideal scaling (center mass = 1, orbit radius = 1) then the scaling will result in 
        /// an orbit period of 2 Pi (6.28-ish). 
        /// </summary>
        /// <returns></returns>
        public double TimeGE()
        {
            return t_physics;
        }

        /// <summary>
        /// Time in world units (defaultUnits when GEC was created). 
        /// 
        /// When in Job mode this function cannot be called when the job is running since time is evolving in the job system. In 
        /// that case use this function within code added to a physics loop complete callback.
        /// </summary>
        /// <returns></returns>
        public double TimeWorld()
        {
            if (running) {
                // can't just grab shared time from running engine
                return double.NaN;
            }
            return geScaler.ScaleTimeGEToWorld(t_physics);
        }

        /// <summary>
        /// Provide the time for one period of the reference (scale) orbit in world time units.
        /// </summary>
        /// <returns></returns>
        public double OrbitPeriodWorldTime()
        {
            return geScaler.ScaleTimeGEToWorld(2.0 * math.PI);
        }

        /// <summary>
        /// Time expressed as Julian days.
        /// </summary>
        /// <returns></returns>
        public double TimeJulianDays()
        {
            // assume world time is seconds
            double elapsedJD = TimeUtils.SecToJD(geScaler.ScaleTimeGEToWorld(t_physics));
            return geConfig.startTimeJD + elapsedJD;
        }

        /// <summary>
		/// Set the world time.
		///
		/// This is only permitted if all bodies have determinsitic propagators
		/// i.e. everything is "on-rails". In such cases the time can be jumped to the
        /// future or the past. 
		///
		/// Note:
		/// - any consumed maneuvers will not be undone. This is not a true rewind.
		/// - if a true rewind is needed for some rails objects, consider enabling
		///   patches for those bodies
		///   
		/// </summary>
		/// <param name="worldTime"></param>
		/// <returns></returns>
        public bool TimeWorldSet(double worldTime)
        {
            if (running) {
                Debug.LogError("Cannot change time while physics is running");
                return false;
            }

            if (OnRails()) {
                gePhysConfig[T_PARAM] = geScaler.ScaleTimeWorldToGE(worldTime);
                t_physics = gePhysConfig[T_PARAM];
                if (geTrajectory != null) {
                    trajReset = true;
                }
                return true;
            }
            return false;
        }

        /// <summary>
        /// Determine if all bodies are using deterministic propagators aka all "On Rails".
        /// Conversly ensure no bodies are evolving using numerical integration of gravitational 
        /// forces.
        /// 
        /// </summary>
        /// <returns></returns>
        public bool OnRails()
        {
            bool onRails = (gePhysicsJob.lenMassiveBodies.Value == 0) && (gePhysicsJob.lenMasslessBodies.Value == 0);
            // FIXED bodies are also consider on rails. Are all massive/massless fixed?
            if (!onRails) {
                bool offRails = false;
                foreach (int i in gePhysicsJob.massiveBodies) {
                    if (!(bodies.propType[i] == GEPhysicsCore.Propagator.FIXED)) {
                        offRails = true;
                        break;
                    }
                }
                foreach (int i in gePhysicsJob.masslessBodies) {
                    if (!(bodies.propType[i] == GEPhysicsCore.Propagator.FIXED)) {
                        offRails = true;
                        break;
                    }
                }
                onRails = !offRails;
            }
            return onRails;
        }

        public bool BodyOnRails(int bodyId)
        {
            if (bodies.patchIndex[bodyId] >= 0) {
                return true;
            }
            if (bodies.propType[bodyId] == GEPhysicsCore.Propagator.GRAVITY) {
                return false;
            }
            return true;
        }

        public ReferenceFrame RefFrame()
        {
            return refFrame;
        }

        /// <summary>
        /// Call prior to beginning evolution in case integrator needs to set up internal info. 
        /// </summary>
        public void PreCalcIntegration()
        {
            //geJob.PreCalc();
            preCalcDone = true;
        }

        /// <summary>
        /// Determine if GECore is currently running a physical loop. 
        /// </summary>
        /// <returns></returns>
        public bool IsRunning()
        {
            return running;
        }

        /// <summary>
        /// Add a callback to be invoked when GE physics loop completes running. This callback will be run once
        /// when the current physics loop completes (or if in batch mode will be run immediately).
        ///
        /// This is used when GE is using an IJob on another thread to ensure that changes to state, adding,
        /// removing etc are not done while GE is using the physics data.
        ///
        /// Physics data from a coherent snapshot can be retreived when the physics loop is running. It cannot be set.
        /// This makes code to e.g. compare distance between bodies to run in Update() methods without adding
        /// callbacks.
        /// 
        /// If GE is in immediate mode this code will simply call the cb immediatly. Changes can always be wrapped this way
        /// and the control code does not need to care if GE is running physics or not. 
        /// 
        /// </summary>
        /// <param name="cb"></param>
        public void PhyLoopCompleteCallbackAdd(PhysLoopCompleteCallback cb, object arg = null)
        {
            if (running) {
                physLoopCompleteCallbacks.Add(new PhysLoopCompleteCB(cb, arg));
            } else {
                cb(this, arg);
            }
        }

        /// <summary>
        /// An external acceleration function can be added to a body. This acceleration may be
        /// a "self-force" (e.g. thrust from an ion drive) or a force from this bodyId to other 
        /// bodies (e.g. the J2 force due to a non-spherical planet). This is specified by the 
        /// eaType (SELF or ON_OTHER). 
        /// 
        /// The return value is an index to the external acceleration. Updates to the acceleration 
        /// parameters require a reference to the external acceleration. (A given body ID may have more
        /// that one external acceleration e.g. rocket engine and atmospheric drag). 
        /// 
        /// The application of the acceleration takes place in the depths of GEPhysicsCore numerical
        /// integration and this may use the Job system: Consequently 
        /// - any parameter information must be presented as a block of double3[]. 
        /// - function pointers are not permitted [*]. Any user defined forces will require an extension
        /// in @see ExtAccelFactory
        /// 
        /// Each function can take a block of double3[] as parameters and scratch memory.
        /// 
        /// [*] technically an unsafe function pointer can be used, but the Asset Store does not
        /// permit assets to declare unsafe code. 
        /// 
        /// </summary>
        /// <param name="bodyId">identity of body to compute external acceleration</param>
        /// <param name="eaType">type of force (self/on-other)</param>
        /// <param name="accelType">enumerated type to slect force</param>
        /// <param name="accelData">parameters for force calculation</param>
        /// <returns>id for the acceleration</returns>
        public int ExternalAccelerationAdd(int bodyId,
                                ExternalAccel.ExtAccelType eaType,
                                ExternalAccel.AccelType accelType,
                                double3[] accelData = null)
        {
            if (running) {
                Debug.LogError("Cannot add a force while running");
                return -1; ;
            }

            if (accelType == ExternalAccel.AccelType.BOOSTER) {
                // check units not DL (need to convert to/from SI)
                if (geScaler.WorldUnits() == GBUnits.Units.DL) {
                    Debug.LogError("Cannot add a booster with DL units");
                    return -1;
                }
            }

            ExternalAccel.EADesc eaDesc = new ExternalAccel.EADesc();
            eaDesc.bodyId = bodyId;
            eaDesc.type = eaType;
            eaDesc.forceId = accelType;
            if (accelData != null) {
                ExternalAccel.ExtAccelDataAlloc(accelData, ref gePhysicsJob, ref eaDesc);
            }
            if (eaType == ExternalAccel.ExtAccelType.SELF_INTEGRATED) {
                bodies.self_integrating[bodyId] = true;
            }
            int extAccelId = extAccelPool.Alloc(ref gePhysicsJob.extADesc);
            gePhysicsJob.extADesc[extAccelId] = eaDesc;
            // now need to update the index list
            extAccels.Add(extAccelId);
            ExtAccelIndexUpdate();
            return extAccelId;
        }

        /// <summary>
		/// Return a copy of the external acceleration params. 
		/// </summary>
		/// <param name="extAccelId">the id of the external acceleration (NOT bodyId!)</param>
		/// <returns></returns>
        public double3[] ExternalAccelerationData(int extAccelId)
        {
            if (running) {
                Debug.LogError("Data not stable while running");
                return null;
            }
            return ExternalAccel.ExtAccelData(ref gePhysicsJob, extAccelId);
        }

        public int ExternalAccelerationDataOffset(int extAccelId)
        {
            if (extAccelId < 0) {
                return -1;
            }
            return gePhysicsJob.extADesc[extAccelId].paramBase;
        }

        /// <summary>
        /// Update the external acceleration parameters for a specific external acceleration. 
        /// </summary>
        /// <param name="extAccelId"></param>
        /// <param name="data"></param>
        public void ExternalAccelerationDataUpdate(int extAccelId, double3[] data)
        {
            if (running) {
                Debug.LogError("Cannot add a force while running");
                return;
            }
            ExternalAccel.ExtAccelDataUpdate(ref gePhysicsJob, extAccelId, data);
        }

        /// <summary>
        /// Remove the external acceleration.
        /// </summary>
        /// <param name="extAccelId"></param>
        public void ExternalAccelerationRemove(int extAccelId)
        {
            if (running) {
                Debug.LogError("Cannot add a force while running");
                return;
            }

            // Get the body ID and type before removing
            int bodyId = gePhysicsJob.extADesc[extAccelId].bodyId;
            if (bodyId < 0) {
                // already removed
                return;
            }
            bool wasSelfIntegrating = gePhysicsJob.extADesc[extAccelId].type == ExternalAccel.ExtAccelType.SELF_INTEGRATED;

            extAccels.Remove(extAccelId);
            extAccelPool.Free(extAccelId);
            ExtAccelIndexUpdate();
            ExternalAccel.ExtAccelDataFree(ref gePhysicsJob, extAccelId);

            // If this was a self-integrating external acceleration, check if the body still has any
            if (wasSelfIntegrating) {
                bool stillHasSelfIntegrating = false;
                foreach (int remainingExtAccelId in extAccels) {
                    if (gePhysicsJob.extADesc[remainingExtAccelId].bodyId == bodyId &&
                        gePhysicsJob.extADesc[remainingExtAccelId].type == ExternalAccel.ExtAccelType.SELF_INTEGRATED) {
                        stillHasSelfIntegrating = true;
                        break;
                    }
                }
                // Clear the self_integrating flag if no more self-integrating external accelerations
                if (!stillHasSelfIntegrating) {
                    bodies.self_integrating[bodyId] = false;
                }
            }
        }

        /// <summary>
        /// Get the total number of external accelerations in the system.
        /// </summary>
        /// <returns>Number of external accelerations</returns>
        public int ExternalAccelerationCount()
        {
            return extAccels.Count;
        }

        private void ExtAccelIndexUpdate()
        {
            // update geJob
            gePhysicsJob.extAccelIndices.Dispose();
            gePhysicsJob.extAccelIndices = new NativeArray<int>(extAccels.Count, Allocator.Persistent);
            for (int i = 0; i < extAccels.Count; i++) {
                gePhysicsJob.extAccelIndices[i] = extAccels[i];
            }
            gePhysicsJob.lenExtAccelIndices.Value = extAccels.Count;
        }

        private void GrowBodies()
        {
            int n = gePhysicsJob.bodies.r.Length;
            for (int i = 0; i < geConfig.GROW_BY; i++) {
                freeBodies.Push(n + i);
            }
            if (DEBUG)
                Debug.Log("Growing GEPhysicsCore to " + (gePhysicsJob.bodies.r.Length + geConfig.GROW_BY));
            gePhysicsJob.GrowBodies(geConfig.GROW_BY);
            // changed inner arrays, so need a new copy
            bodies = gePhysicsJob.bodies;
        }

        /// <summary>
        /// Get information from the GE physics config. 
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        public double GetParm(int index)
        {
            return gePhysConfig[index];
        }

        /// <summary>
        /// Get a reference to the GEPhysicsJob. 
        /// 
        /// Used by GSController for particle update. 
        /// 
        /// User use at your own risk. 
        /// </summary>
        /// <returns></returns>
        public ref GEPhysicsCore.GEPhysicsJob PhysicsJob()
        {
            return ref gePhysicsJob;
        }

        /// <summary>
        /// Return the total number of bodies in GE (of all types)
        /// </summary>
        /// <returns></returns>
        public int NumBodies()
        {
            return numBodies;
        }

        /// Adding Bodies

        /// <summary>
        /// Add a body with absolute world R, V, mass state information to GE with world
        /// information expressed in the defaultUnits specified in the GECore constructor.
        ///
        /// There is no general propagator choice in this case because all the propagators
        /// are relative to some center body. To add with a propagtor use one of the
        /// BodyAddInOrbitWithRVRelative or BodyAddInOrbitWithCOE methods.
        /// are relative to some center body. To add with a propagator use one of 
        /// @see BodyAddInOrbitWithRVRelative
        /// @see BodyAddInOrbitWithCOE
        /// @see BodyAddInOrbitWithTLE
        /// 
        /// </summary>
        /// <param name="rWorld">position in World default units</param>
        /// <param name="vWorld">velocity in World default units</param>
        /// <param name="massWorld">world mass (may be zero)</param>
        /// <param name="isFixed">(optional) body is fixed and will not be moved by GE</param>
        /// <returns></returns>
        public int BodyAdd(double3 rWorld,
                            double3 vWorld,
                            double massWorld = 0,
                            bool isFixed = false)
        {
            if (running) {
                Debug.LogWarning("Adding body while GE is running. Rejected.");
                return -1;
            }
            double3 r = rWorld * geScaler.ScaleLenWorldToGE(1.0);
            double3 v = vWorld * geScaler.ScaleVelocityWorldToGE(1.0);
            numBodies++;
            if (numBodies >= gePhysicsJob.bodies.r.Length) {
                GrowBodies();
            }
            int id = AllocBodyIndex();

            GEPhysicsCore.Propagator prop = GEPhysicsCore.Propagator.GRAVITY;
            if (isFixed)
                prop = GEPhysicsCore.Propagator.FIXED;

            double massGe = geScaler.ScaleMassWorldToGE(massWorld);
            gePhysicsJob.BodyAdd(id, r, v, massGe, prop, propIndex: -1, patchIndex: -1);

            if (preCalcDone)
                gePhysicsJob.PreCalcForAdd(id);
            if (geTrajectory != null) {
                trajReset = true;
            }
            return id;
        }

        // AddBodyInOrbitWith*
        // This group of methods is for adding a body given a specific type of initial data (RV, COE, TLE etc.)
        // For each type of initial data the way in which a given propagator is created and inited varies,
        // hence the common "feel" for these routines.
        //
        // These methods set up a propagator and then pass off to an internal method to do all the plumbing of
        // getting the initial data and the prop info into the physics core. 


        /// <summary>
        /// Add a body by specifying state relative to a second body. Typically this is used to place a body in 
        /// orbit around a body when the specific R, V is known. 
        /// 
        /// To put a body in orbit using orbit parameters <see cref="BodyAddInOrbitWithCOE"/>  
        /// 
        /// Patching:
        /// Bodies that have deterministic (i.e. no numerical integration) propagators are candidates to be "patched". 
        /// A patched body records subsequent changes in state (r, v) by appending a propagator segment to the body. 
        /// The patch has an associated start and end time and the evolution code uses the patch corresponding to the 
        /// current time. 
        /// 
        /// Patching is typically used to either:
        /// - handle a maneuver sequence in a way that allows time to be jumped forward or backward
        /// - implement a sequence of orbital propagators (aka conics) to model movement from one gravity
        /// system to another where only one system at a time exerts a gravitational force. 
        /// 
        /// </summary>
        /// <param name="rWorld">relative position in default units</param>
        /// <param name="vWorld">relative velocity in default units</param>
        /// <param name="centerId">body id of center body</param>
        /// <param name="prop">propagator type</param>
        /// <param name="massWorld">mass of object to add</param>
        /// <param name="isPatched">subsequent changes should be added as patches</param>
        /// <param name="time_patch_start">time of first patch</param>
        /// <returns></returns>

        public int BodyAddInOrbitWithRVRelative(double3 rWorld,
                           double3 vWorld,
                           int centerId,
                           GEPhysicsCore.Propagator prop,
                           double massWorld = 0.0,
                           bool isPatched = false,
                           double time_patch_start = 0.0,
                           GEPhysicsCore.EarlyPropagation earlyPropagation = GEPhysicsCore.EarlyPropagation.ERROR_INACTIVATE,
                           double epochWorldTime = double.NaN)
        {
            int pIndex = -1;
            double3 r = rWorld * geScaler.ScaleLenWorldToGE(1.0);
            double3 v = vWorld * geScaler.ScaleVelocityWorldToGE(1.0);

            double t_prop_start = t_physics;
            bool hasEpochTime = !math.isnan(epochWorldTime);
            if (hasEpochTime) {
                if (epochWorldTime < geScaler.ScaleTimeGEToWorld(t_physics) && !GEPhysicsCore.IsEarlyPropogatable(prop)) {
                    Debug.LogError($"Cannot add body at an earlier time than the propagator {prop} allows");
                    return -1;
                }
                t_prop_start = geScaler.ScaleTimeWorldToGE(epochWorldTime);
            }

            double mu_center = bodies.mu[centerId];
            switch (prop) {
                case GEPhysicsCore.Propagator.GRAVITY:
                    break;

                case GEPhysicsCore.Propagator.KEPLER:
                    // allocate Kepler prop info
                    pIndex = keplerPool.Alloc(ref gePhysicsJob.keplerPropInfo);
                    int keplerDepth = 0;
                    if (bodies.propType[centerId] == GEPhysicsCore.Propagator.KEPLER) {
                        keplerDepth = gePhysicsJob.keplerPropInfo[bodies.propIndex[centerId]].keplerDepth + 1;
                    }
                    gePhysicsJob.keplerPropInfo[pIndex] =
                            new KeplerPropagator.PropInfo(r, v, t_prop_start, mu_center, centerId, keplerDepth);
                    if (hasEpochTime) {
                        int status;
                        (status, r, v) = KeplerPropagator.RVforTime(gePhysicsJob.keplerPropInfo[pIndex].rvt, t_physics);
                        if (status != 0) {
                            Debug.LogError("Prop to GE time had status=" + status);
                            return -1;
                        }
                    }
                    break;

                case GEPhysicsCore.Propagator.KEPLER_COE:
                    Orbital.COE coeK = Orbital.RVtoCOE(r, v, bodies.mu[centerId]);
                    pIndex = keplerCoePool.Alloc(ref gePhysicsJob.keplerCoePropInfo);
                    gePhysicsJob.keplerCoePropInfo[pIndex] =
                            new KeplerCOEPropagator.PropInfo(coeK, t_prop_start, centerId);
                    if (hasEpochTime) {
                        (r, v) = KeplerCOEPropagator.KeplerProp(ref gePhysicsJob.keplerCoePropInfo, pIndex, t_physics);
                    }
                    break;

                case GEPhysicsCore.Propagator.PKEPLER:
                    Orbital.COE coe = Orbital.RVtoCOE(r, v, mu_center);
                    coe.ScaleLength(geConfig.scaleLtoKm);
                    pIndex = pkeplerPool.Alloc(ref gePhysicsJob.pKeplerPropInfo);
                    gePhysicsJob.pKeplerPropInfo[pIndex] =
                            new PKeplerPropagator.PropInfo(coe, t_prop_start, centerId);
                    if (hasEpochTime) {
                        Orbital.COEStruct coeDummy;
                        (r, v, coeDummy) = PKeplerPropagator.PKeplerProp(ref gePhysicsJob.pKeplerPropInfo, pIndex, t_physics);
                    }
                    break;

                case GEPhysicsCore.Propagator.SGP4_RAILS:
                    throw new NotImplementedException("SGP4_RAILS not implemented for BodyAddInOrbitWithRVRelative");

                case GEPhysicsCore.Propagator.FIXED:
                    // maybe want a fixed object at some relative place ??
                    break;



                default:
                    throw new NotImplementedException("no case for propagator " + prop);
            }

            double massGe = geScaler.ScaleMassWorldToGE(massWorld);
            return BodyAddWithProp(r, v, centerId, prop, massGe, isPatched, pIndex, earlyPropagation: earlyPropagation);

        }

        /// <summary>
        /// Add a body to the surface of a planet. The propagator is implicitly PLANET_SURFACE. 
        /// 
        /// The body is added at a specified latitude and longitude and the planet's rotation is taken into account. 
        /// 
        /// </summary>
        /// <param name="latitudeDeg"></param>
        /// <param name="longitudeDeg"></param>
        /// <param name="centerId"></param>
        /// <param name="radiusWorld">radius (including altitude)</param>
        /// <param name="axis">rotation axis</param>
        /// <param name="phi0">initial rotation angle</param>
        /// <param name="omegaWorld">rotation rate</param>
        /// <param name="isPatched"></param>
        /// <param name="time_patch_start"></param>
        /// <returns></returns>
        public int BodyAddOnPlanetSurface(double latitudeDeg,
                   double longitudeDeg,
                   int centerId,
                   double radiusWorld,
                   double3 axis,
                   double phi0 = 0.0,
                   double omegaWorld = 0.0,
                   bool isPatched = false,
                   double time_patch_start = 0.0)
        {
            int pIndex = rotationPool.Alloc(ref gePhysicsJob.rotationPropInfo);
            // convert to GE units  
            double radiusGE = radiusWorld * geScaler.ScaleLenWorldToGE(1.0);
            // rad/s so convert by multiplying by the inverse of the time scale
            double omegaGE = omegaWorld * geScaler.ScaleTimeGEToWorld(1.0);
            gePhysicsJob.rotationPropInfo[pIndex] =
                new RotationPropagator.PropInfo(centerId, axis, omegaGE, phi0, latitudeDeg, longitudeDeg, radiusGE);
            GEBodyState state = new GEBodyState();
            state.r = double3.zero;
            state.v = double3.zero;
            RotationPropagator.EvolveRelative(t_physics, pIndex, ref gePhysicsJob.rotationPropInfo, ref state);
            double3 r = state.r;
            double3 v = state.v;
            return BodyAddWithProp(r, v, centerId,
                                        GEPhysicsCore.Propagator.PLANET_SURFACE,
                                        massGe: 0.0,
                                        isPatched,
                                        pIndex);

        }

        /// <summary>
        /// Add a body in orbit relative to a center body using COE (classical orbital elements, a, eccentricy, inclination etc.)
        /// 
        /// For patching information see comments in <see cref="BodyAddInOrbitWithRVRelative"/> 
        /// </summary>
        /// <param name="coeWorld">orbit elements in default units</param>
        /// <param name="centerId">id of center body of orbit</param>
        /// <param name="prop">propagator</param>
        /// <param name="massWorld">mass of body to add</param>
        /// <param name="isPatched">updates to this body as patches</param>
        /// <param name="time_patch_start">time patches start</param>
        /// <returns></returns>
        public int BodyAddInOrbitWithCOE(Orbital.COE coeWorld,
                                int centerId,
                                GEPhysicsCore.Propagator prop,
                                double massWorld = 0.0,
                                bool isPatched = false,
                                double time_patch_start = 0.0,
                                GEPhysicsCore.EarlyPropagation earlyPropagation = GEPhysicsCore.EarlyPropagation.ERROR_INACTIVATE,
                                double epochWorldTime = double.NaN)
        {
            if (running) {
                Debug.LogWarning("AddBodyInOrbit while GE is running. Rejected.");
                return -1;
            }
            double tEpochGE = t_physics;
            bool propToStartTime = false;
            bool hasEpochTime = !math.isnan(epochWorldTime);
            if (hasEpochTime) {
                if (epochWorldTime > geScaler.ScaleTimeGEToWorld(t_physics) && !GEPhysicsCore.IsEarlyPropogatable(prop)) {
                    Debug.LogError($"Cannot add body at an earlier time than the propagator {prop} allows");
                    return -1;
                }
                tEpochGE = geScaler.ScaleTimeWorldToGE(epochWorldTime);
                propToStartTime = true;
            }

            Orbital.COE coeGE = new Orbital.COE(coeWorld);
            coeGE.ScaleLength(geScaler.ScaleLenWorldToGE(1.0));
            coeGE.mu = bodies.mu[centerId];
            (double3 r, double3 v) = Orbital.COEtoRVRelative(coeGE);

            int pIndex = -1;
            switch (prop) {
                case GEPhysicsCore.Propagator.GRAVITY:
                    if (propToStartTime) {
                        Debug.LogError("GRAVITY mode canot use an EPOCH time");
                        return -1;
                    }
                    // no prop to alloc
                    if (coeGE.startEpochJD > 0) {
                        // Are we starting at the start epoch? That's ok
                        if (math.abs(geConfig.startTimeJD - coeGE.startEpochJD) > 1E-3) {
                            Debug.LogError("GRAVITY mode and GE start does not match COE start");
                            return -1;
                        }
                    }
                    if (refFrame == ReferenceFrame.COROTATING_CR3BP) {
                        // r, v are in phys (CR3BP) units. 
                        (r, v) = CR3BP.FrameInertialToRotating(r, v, t_physics);
                    }
                    break;

                case GEPhysicsCore.Propagator.KEPLER:
                    // allocate Kepler prop info
                    double mu_center = bodies.mu[centerId];
                    pIndex = keplerPool.Alloc(ref gePhysicsJob.keplerPropInfo);

                    int keplerDepth = 0;
                    if (bodies.propType[centerId] == GEPhysicsCore.Propagator.KEPLER) {
                        keplerDepth = gePhysicsJob.keplerPropInfo[bodies.propIndex[centerId]].keplerDepth + 1;
                    }
                    gePhysicsJob.keplerPropInfo[pIndex] =
                            new KeplerPropagator.PropInfo(r, v, tEpochGE, mu_center, centerId, keplerDepth);
                    if (propToStartTime) {
                        int status;
                        (status, r, v) = KeplerPropagator.RVforTime(gePhysicsJob.keplerPropInfo[pIndex].rvt, tEpochGE);
                        if (status != 0) {
                            Debug.LogError("Prop to GE time had status=" + status);
                            return -1;
                        }
                    }
                    break;

                case GEPhysicsCore.Propagator.KEPLER_COE:
                    Orbital.COE coeK = Orbital.RVtoCOE(r, v, bodies.mu[centerId]);
                    pIndex = keplerCoePool.Alloc(ref gePhysicsJob.keplerCoePropInfo);
                    gePhysicsJob.keplerCoePropInfo[pIndex] =
                            new KeplerCOEPropagator.PropInfo(coeK, tEpochGE, centerId);
                    if (propToStartTime) {
                        (r, v) = KeplerCOEPropagator.KeplerProp(ref gePhysicsJob.keplerCoePropInfo, pIndex, t_physics);
                    }
                    break;

                case GEPhysicsCore.Propagator.PKEPLER:
                    if (coeGE.startEpochJD > 0) {
                        Debug.LogError("Cannot use COE startJD and PKEPLER");
                        return -1;
                    }
                    pIndex = pkeplerPool.Alloc(ref gePhysicsJob.pKeplerPropInfo);
                    coeGE.ScaleLength(geConfig.scaleLtoKm);
                    // COE mu needs to be in SI. Must be for Earth
                    coeGE.mu = GBUnits.earthMuKm;
                    // t_start for PKepler is in GE units
                    gePhysicsJob.pKeplerPropInfo[pIndex] =
                            new PKeplerPropagator.PropInfo(coeGE, tEpochGE, centerId);
                    if (propToStartTime) {
                        Orbital.COEStruct coeDummy;
                        (r, v, coeDummy) = PKeplerPropagator.PKeplerProp(ref gePhysicsJob.pKeplerPropInfo, pIndex, t_physics);
                    }
                    break;

                case GEPhysicsCore.Propagator.SGP4_RAILS:
                    if (coeGE.startEpochJD > 0) {
                        Debug.LogError("Cannot use COE startJD and SGP4");
                        return -1;
                    }
                    SGP4SatData satData = new SGP4SatData();
                    double timeJD = TimeJulianDays();
                    coeGE.ScaleLength(geConfig.scaleLtoKm);
                    coeGE.mu = GBUnits.earthMuKm;
                    double t_prop_start_sec = geScaler.ScaleTimeGEToWorld(tEpochGE);
                    double epochJD = TimeUtils.SecToJD(t_prop_start_sec) + geConfig.startTimeJD;
                    satData.InitFromCOE(coeGE, epochJD);
                    pIndex = sgp4Pool.Alloc(ref gePhysicsJob.sgp4PropInfo);
                    gePhysicsJob.sgp4PropInfo[pIndex] = new SGP4Propagator.PropInfo(satData, centerId);
                    if (propToStartTime) {
                        Orbital.COEStruct coeDummy;
                        int status;
                        (status, r, v, coeDummy) = SGP4Propagator.RVforTime(ref satData, timeJD, geConfig.scaleLtoKm, geConfig.scaleKmsecToV);
                        if (status != 0) {
                            Debug.LogError("Prop to GE time had status=" + status);
                            return -1;
                        }
                    }
                    break;

                case GEPhysicsCore.Propagator.FIXED:
                    // odd, but allow it
                    break;

                default:
                    throw new NotImplementedException("no case for propagator " + prop);
            }
            double massGe = geScaler.ScaleMassWorldToGE(massWorld);
            return BodyAddWithProp(r, v, centerId, prop, massGe, isPatched, pIndex, earlyPropagation);
        }

        private (int status, double3 r, double3) PropKeplerToGEStartTime(Orbital.COE coe, double3 r0, double3 v0, double epochTime)
        {
            double3 r = double3.zero;
            double3 v = double3.zero;
            if (epochTime > 0) {
                // Evolve to the start time
                KeplerPropagator.RVT kProp = new KeplerPropagator.RVT(r0, v0, t0: 0.0, coe.mu);
                int status;
                (status, r, v) = KeplerPropagator.RVforTime(kProp, epochTime - t_physics);
                if (status != 0) {
                    Debug.LogError("Prop to GE time had status=" + status);
                    return (1, r, v);
                }
                return (0, r, v);
            }
            return (0, r0, v0);
        }

        /// <summary>
        /// Add a body in orbit around Earth (only Earth!) using a two-line element data structure. 
        /// 
        /// Earth satellite information is often expressed in a two-line forms (e.g. celestrak.com) which harkens back 
        /// to two 80 column punch cards (some API choices are "sticky"!). 
        /// 
        /// Use of this method requires that the center body have the mass of the Earth. It also requires that the start time
        /// of the GECore in Julian days has been configured. Satellite data provides the orbit information at a specific date/time
        /// and this time must be earlier than the start time of GECore. 
        /// 
        ///         
        /// For patching information see comments in <see cref="BodyAddInOrbitWithRVRelative"/> 
        ///
        /// </summary>
        /// <param name="tleData">string representing the two line element info</param>
        /// <param name="centerId">id of center body</param>
        /// <param name="prop">propagator</param>
        /// <param name="isPatched">handle updates to state as patches</param>
        /// <param name="time_patch_start"></param>
        /// <returns></returns>
        public int BodyAddInOrbitWithTLE(string tleData,
                            int centerId,
                            GEPhysicsCore.Propagator prop,
                            bool isPatched = false,
                            double time_patch_start = 0.0,
                            GEPhysicsCore.EarlyPropagation earlyPropagation = GEPhysicsCore.EarlyPropagation.ERROR_INACTIVATE,
                            double epochWorldTime = double.NaN)
        {
            if (running) {
                Debug.LogWarning("AddBodyInOrbit while GE is running. Rejected.");
                return -1;
            }
            SGP4SatData satData = new SGP4SatData();
            if (!SGP4utils_GE2.TLEtoSatData(tleData, ref satData)) {
                Debug.LogError("Could not init TLE for " + tleData);
                return -1;
            }
            // TLE is defined at a specific epoch time and GE start time will typically not align with this. 
            // (1) If TLE time is earlier than GE start time then need to evolve the initial state (we can always do this)
            // (2) If TLE time is later we *might* be ok, if the propagator allow early prop and the user has set EARLY_PROPAGATE
            //
            // Use the propagator to do this evolution. (May not be SGP if e.g. TLE data for Kepler prop)
            double epochTimeJD;
            double timeNowJD = TimeJulianDays();
            // Are we using time in the TLE or did the user set an epoch time?
            bool useTleEpoch = !math.isnan(epochWorldTime);
            if (useTleEpoch) {
                epochTimeJD = TimeUtils.SecToJD(epochWorldTime) + geConfig.startTimeJD;
                satData.jdsatepoch = epochTimeJD;
            } else {
                epochTimeJD = satData.jdsatepoch;
            }

            if (epochTimeJD > timeNowJD) {
                // (2) TLE time is later than GE start time
                if (!(GEPhysicsCore.IsEarlyPropogatable(prop) && earlyPropagation == GEPhysicsCore.EarlyPropagation.PROPAGATE)) {
                    Debug.LogError("Cannot add body at a later time than the propagator " + prop + " allows with early=" + earlyPropagation);
                    return -1;
                }
            }
            double tEpochGE = (epochTimeJD - geConfig.startTimeJD) * TimeUtils.SEC_PER_DAY;

            int pIndex = -1;
            // Get the state at the TLE epoch
            (int status, double3 r, double3 v, Orbital.COEStruct coeS) =
                SGP4Propagator.RVforTime(ref satData, satData.jdsatepoch, geConfig.scaleLtoKm, geConfig.scaleKmsecToV);
            if (status != 0) {
                Debug.LogErrorFormat("Failed init. Is Start time before TLE epoch? TLE={0}", tleData);
                return -1;
            }

            // If prop is not SGP4 then need to get COE from TLE and then do usual in orbit add
            switch (prop) {
                case GEPhysicsCore.Propagator.SGP4_RAILS:
                    // body IS using SGP4. Init a dedicated SGP4 prop and add explicitly
                    // will need to set the startTimeJD ?? Should this just be a seperate Set in GE??
                    // Center pos will be added by AddBodyInOrbit, relative is ok here.
                    pIndex = sgp4Pool.Alloc(ref gePhysicsJob.sgp4PropInfo);
                    gePhysicsJob.sgp4PropInfo[pIndex] = new SGP4Propagator.PropInfo(satData, centerId);
                    // prop to current time
                    Orbital.COEStruct coeDummy;
                    (status, r, v, coeDummy) = SGP4Propagator.RVforTime(ref satData, timeNowJD, geConfig.scaleLtoKm, geConfig.scaleKmsecToV);
                    if (status != 0) {
                        Debug.LogError("Prop to GE time had status=" + status);
                        return -1;
                    }
                    break;

                case GEPhysicsCore.Propagator.KEPLER:
                    // copy across initial orbit elements and use startPhase from above
                    // this ensures we get the "as written" COE and not the post-init ones
                    Orbital.COE coeKm = new Orbital.COE();
                    satData.FillInCOEKmWithInitialData(ref coeKm);
                    coeKm.ScaleLength(1.0 / geConfig.scaleLtoKm);
                    double mu_center = bodies.mu[centerId];
                    coeKm.mu = mu_center;
                    pIndex = keplerPool.Alloc(ref gePhysicsJob.keplerPropInfo);
                    int keplerDepth = 0;
                    if (bodies.propType[centerId] == GEPhysicsCore.Propagator.KEPLER) {
                        keplerDepth = gePhysicsJob.keplerPropInfo[bodies.propIndex[centerId]].keplerDepth + 1;
                    }
                    gePhysicsJob.keplerPropInfo[pIndex] =
                            new KeplerPropagator.PropInfo(r, v, tEpochGE, mu_center, centerId, keplerDepth);
                    // prop to current time
                    (status, r, v) = KeplerPropagator.RVforTime(gePhysicsJob.keplerPropInfo[pIndex].rvt, t_physics);
                    if (status != 0) {
                        Debug.LogError("Prop to GE time had status=" + status);
                        return -1;
                    }
                    break;

                case GEPhysicsCore.Propagator.PKEPLER:
                    Orbital.COE coeKm2 = new Orbital.COE();
                    satData.FillInCOEKmWithInitialData(ref coeKm2);
                    pIndex = pkeplerPool.Alloc(ref gePhysicsJob.pKeplerPropInfo);
                    gePhysicsJob.pKeplerPropInfo[pIndex] =
                            new PKeplerPropagator.PropInfo(coeKm2, tEpochGE, centerId);
                    // prop to current time
                    (double3 r2, double3 v2, Orbital.COEStruct coeDummy2) = PKeplerPropagator.PKeplerProp(ref gePhysicsJob.pKeplerPropInfo, pIndex, t_physics);
                    r = r2;
                    v = v2;
                    break;

                case GEPhysicsCore.Propagator.FIXED:
                    Debug.LogError("Cannot init fixed body with SGP4");
                    return -1;

                case GEPhysicsCore.Propagator.GRAVITY:
                    // already have R, V from above
                    break;

                case GEPhysicsCore.Propagator.KEPLER_COE:
                    double mu_center_kcoe_tle = bodies.mu[centerId];
                    Orbital.COE coeKm3 = Orbital.RVtoCOE(r, v, mu_center_kcoe_tle);
                    pIndex = keplerCoePool.Alloc(ref gePhysicsJob.keplerCoePropInfo);
                    gePhysicsJob.keplerCoePropInfo[pIndex] =
                            new KeplerCOEPropagator.PropInfo(coeKm3, tEpochGE, centerId);
                    // prop to current time
                    (r, v) = KeplerCOEPropagator.KeplerProp(ref gePhysicsJob.keplerCoePropInfo, pIndex, t_physics);
                    if (status != 0) {
                        Debug.LogError("Prop to GE time had status=" + status);
                        return -1;
                    }
                    break;

                default:
                    throw new NotImplementedException("unknown case " + prop);
            }
            return BodyAddWithProp(r, v, centerId, prop, 0.0, isPatched, pIndex, earlyPropagation);
        }

        /// <summary>
        /// Add a body with a table of ephemeris data (a table of position and velocity at future times). Some websites (JPL)
        /// and third-party software tools can emit an ephemeris table. 
        /// 
        /// Ephemeris data is wrapped in a class.
        ///      
        /// For patching information see comments in <see cref="BodyAddInOrbitWithRVRelative"/> 
        ///         
        /// </summary>
        /// <param name="ephemData">ephemeris data in world units</param>
        /// <param name="cid">center body id (optional)</param>
        /// <param name="m">mass of body</param>
        /// <param name="isPatched">add as a patch to the body</param>
        /// <param name="time_patch_start"></param>
        /// <returns></returns>
        public int BodyAddWithEphemeris(EphemerisData ephemData,
                    int cid,
                    double m,
                    bool isPatched = false,
                    double time_patch_start = 0.0)
        {
            GEPhysicsCore.Propagator prop = GEPhysicsCore.Propagator.EPHEMERIS;

            // Define an EphemProp
            int propIndex = ephemPool.Alloc(ref gePhysicsJob.ephemPropInfo);
            EphemerisPropagator.EphemDataAlloc(ephemData, ref gePhysicsJob, geScaler, cid, propIndex);

            numBodies++;
            if (numBodies >= gePhysicsJob.bodies.r.Length) {
                GrowBodies();
            }

            // determine the state at the current time
            GEBodyState state = new GEBodyState();
            double mu = 0;
            if (cid >= 0) {
                mu = bodies.mu[cid];
            }
            EphemerisPropagator.EvolveRelative(t_physics, propIndex, ref gePhysicsJob.ephemPropInfo, ref gePhysicsJob.ephemerisData, ref state, mu);

            double3 r_world = state.r;
            double3 v_world = state.v;
            if (cid >= 0) {
                r_world += bodies.r[cid];
                v_world += bodies.v[cid];
            }
            double mGE = geScaler.ScaleMassWorldToGE(m);
            return BodyAddWithProp(r_world, v_world, cid, prop, mGE, isPatched, propIndex);
        }

        /// <summary>
        /// Internal: Add a body with a propagator that has already been allocated. Can be done as
        /// direct body or as a patch.
        /// </summary>
        /// <param name="rGe"></param>
        /// <param name="vGe"></param>
        /// <param name="centerId"></param>
        /// <param name="prop"></param>
        /// <param name="massGe"></param>
        /// <param name="isPatched"></param>
        /// <param name="time_patch_start"></param>
        /// <param name="propIndex"></param>
        /// <returns></returns>
        private int BodyAddWithProp(double3 rGe,
                           double3 vGe,
                           int centerId,
                           GEPhysicsCore.Propagator prop,
                           double massGe,
                           bool isPatched,
                           int propIndex,
                           GEPhysicsCore.EarlyPropagation earlyPropagation = GEPhysicsCore.EarlyPropagation.ERROR_INACTIVATE)
        {
            if (running) {
                Debug.LogWarning("AddBodyInOrbit while GE is running. Rejected.");
                return -1;
            }
            numBodies++;
            if (numBodies >= gePhysicsJob.bodies.r.Length) {
                GrowBodies();
            }

            int id = AllocBodyIndex();
            double3 r_world = rGe + bodies.r[centerId];
            double3 v_world = vGe + bodies.v[centerId];

            // Create a patch entry. This one will be first
            int patchIndex = -1;
            if (isPatched) {
                gePhysicsJob.PatchAdd(id);  // add bodyto index list
                patchIndex = patchPool.Alloc(ref gePhysicsJob.patches);
                bodies.patchIndex[id] = propIndex;
                GEPhysicsCore.PatchInfo patchInfo = new GEPhysicsCore.PatchInfo {
                    timeStartGE = t_physics,
                    timeEndGE = -1.0,
                    prevEntry = -1,
                    nextEntry = -1,
                    prop = prop,
                    propIndex = propIndex
                };
                gePhysicsJob.patches[patchIndex] = patchInfo;
            }
            gePhysicsJob.BodyAdd(id, r_world, v_world, massGe, prop, propIndex, patchIndex, earlyPropagation);
            if (geTrajectory != null) {
                trajReset = true;
            }
            return id;
        }

        /// <summary>
        /// Add a patch specified via classical orbital elements (COE) at the indicated start time. 
        /// 
        /// This can only be performed on a body for which patch mode is enabled. 
        ///
        /// Wrapper on the RV AddPatch.
        /// 
        /// </summary>
        /// <param name="id"></param>
        /// <param name="oe"></param>
        /// <param name="centerId"></param>
        /// <param name="prop"></param>
        /// <param name="time_start"></param>
        /// <param name="time_end"></param>
        /// <param name="mass"></param>
        public void PatchAddToBody(int id,
                                   Orbital.COE oe,
                                   int centerId,
                                   GEPhysicsCore.Propagator prop,
                                   double time_start)
        {
            oe.mu = MuWorld(centerId);
            (double3 r, double3 v) = Orbital.COEtoRVRelative(oe);
            PatchCreateAndAdd(id, centerId, new GEBodyState(r, v), prop, time_start);
        }


        /// <summary>
        /// Remove all patches on a body that start at or after tFrom.
        /// 
        /// This routine will never remove the first patch for a body - since this is how we know that 
        /// body is patched. Better to use PatchCreateAndAdd to add a patch at the starting time. This will 
        /// result in the replacement of the first patch with the new one. 
        /// 
        /// </summary>
        /// <param name="id"></param>
        /// <param name="tFrom"></param>
        public void PatchesRemoveFromBody(int id, double tFrom)
        {
            if (running) {
                Debug.LogWarning("PatchesRemoveFromBody while GE is running. Rejected.");
                return;
            }
            if (bodies.patchIndex[id] < 0) {
                Debug.LogError("Body not marked as patched id=" + id);
                return;
            }
            double tFromGE = geScaler.ScaleTimeWorldToGE(tFrom);

            int p_first = gePhysicsJob.PatchFirst(bodies.patchIndex[id]);
            int p = gePhysicsJob.PatchForTime(tFromGE, bodies.patchIndex[id]);

            // Either 
            // (1) This patch starts at exactly the time to remove (so we will remove it and any later patches)
            // (2) This patch starts before the time to remove and we want to remove everything after it (if there are any)
            // (3) The time we want to add is before the current patch, so remove all patches from here
            if (math.abs(gePhysicsJob.patches[p].timeStartGE - tFromGE) < 1E-9) {
                // remove this patch, unless it's the first patch
                if (p != p_first) {
                    PatchesRemoveFromIndex(p);
                } else {
                    PatchesRemoveFromIndex(gePhysicsJob.patches[p_first].nextEntry);
                }
                return;
            } else {
                PatchesRemoveFromIndex(gePhysicsJob.patches[p].nextEntry);
            }
            // always point to the start. Evolve will find the right patch for the current time
            bodies.patchIndex[id] = p_first;

        }

        private void PatchesRemoveFromIndex(int p)
        {
            if (p < 0)
                return;
            int prevPatch = gePhysicsJob.patches[p].prevEntry;
            if (prevPatch >= 0) {
                // set entry prior to this to no end time and no next entry
                GEPhysicsCore.PatchInfo patchInfo = gePhysicsJob.patches[prevPatch];
                patchInfo.nextEntry = -1;
                patchInfo.timeEndGE = -1.0;
                gePhysicsJob.patches[prevPatch] = patchInfo;
            }
            while (p >= 0) {
                Debug.LogFormat("Removing patch {0} at tGE={1}", p, gePhysicsJob.patches[p].timeStartGE);
                patchPool.Free(p);
                int nextp = gePhysicsJob.patches[p].nextEntry;
                switch (gePhysicsJob.patches[p].prop) {
                    case GEPhysicsCore.Propagator.KEPLER:
                        keplerPool.Free(gePhysicsJob.patches[p].propIndex);
                        break;

                    case GEPhysicsCore.Propagator.PKEPLER:
                        pkeplerPool.Free(gePhysicsJob.patches[p].propIndex);
                        break;

                    case GEPhysicsCore.Propagator.KEPLER_COE:
                        keplerCoePool.Free(gePhysicsJob.patches[p].propIndex);
                        break;

                    case GEPhysicsCore.Propagator.SGP4_RAILS:
                        sgp4Pool.Free(gePhysicsJob.patches[p].propIndex);
                        break;

                    case GEPhysicsCore.Propagator.EPHEMERIS:
                        EphemerisPropagator.EphemDataFree(ref gePhysicsJob, gePhysicsJob.patches[p].propIndex);
                        ephemPool.Free(gePhysicsJob.patches[p].propIndex);
                        break;

                    default:
                        throw new NotImplementedException("unknown case " + gePhysicsJob.patches[p].prop);
                }
                // null out this patch entry
                GEPhysicsCore.PatchInfo patchInfo = new GEPhysicsCore.PatchInfo {
                    prevEntry = -1,
                    nextEntry = -1,
                    timeStartGE = -1,
                    timeEndGE = -1,
                    prop = GEPhysicsCore.Propagator.UNASSIGNED,
                    propIndex = -1
                };
                gePhysicsJob.patches[p] = patchInfo;
                p = nextp;
            }

        }

        /// <summary>
        /// Remove a body from GECore. 
        ///
        /// </summary>
        /// <param name="id"></param>
        public bool BodyRemove(int id)
        {
            if (running) {
                Debug.LogWarning("BodyRemove while GE is running. Rejected.");
                return false;
            }
            // Do index list updates
            GEPhysicsCore.Propagator prop = bodies.propType[id];
            if (prop == GEPhysicsCore.Propagator.UNASSIGNED) {
                Debug.LogWarning("BodyRemove called on unassigned body. id=" + id);
                return false;
            }

            freeBodies.Push(id);

            // Patches
            if (bodies.patchIndex[id] >= 0) {
                // remove all the patches for this body
                int f = gePhysicsJob.PatchFirst(bodies.patchIndex[id]);
                PatchesRemoveFromIndex(f);
                bodies.patchIndex[id] = -1;
                gePhysicsJob.PatchListRemove(id);
            } else {
                // Propagators
                switch (prop) {
                    case GEPhysicsCore.Propagator.KEPLER:
                        keplerPool.Free(bodies.propIndex[id]);
                        break;
                    case GEPhysicsCore.Propagator.KEPLER_COE:
                        keplerCoePool.Free(bodies.propIndex[id]);
                        break;
                    case GEPhysicsCore.Propagator.PKEPLER:
                        pkeplerPool.Free(bodies.propIndex[id]);
                        break;
                    case GEPhysicsCore.Propagator.SGP4_RAILS:
                        sgp4Pool.Free(bodies.propIndex[id]);
                        break;
                    case GEPhysicsCore.Propagator.PLANET_SURFACE:
                        rotationPool.Free(bodies.propIndex[id]);
                        break;
                    case GEPhysicsCore.Propagator.EPHEMERIS:
                        EphemerisPropagator.EphemDataFree(ref gePhysicsJob, bodies.propIndex[id]);
                        ephemPool.Free(bodies.propIndex[id]);
                        break;
                    default:
                        break;
                }
                bodies.propType[id] = GEPhysicsCore.Propagator.UNASSIGNED;
            }
            gePhysicsJob.IndexListRemove(prop, id);

            // External Accelerations
            // Remove all external accelerations associated with this body
            List<int> extAccelsToRemove = new List<int>();
            bool hadSelfIntegrating = false;
            foreach (int extAccelId in extAccels) {
                if (gePhysicsJob.extADesc[extAccelId].bodyId == id) {
                    extAccelsToRemove.Add(extAccelId);
                    // Check if this was a self-integrating external acceleration
                    if (gePhysicsJob.extADesc[extAccelId].type == ExternalAccel.ExtAccelType.SELF_INTEGRATED) {
                        hadSelfIntegrating = true;
                    }
                }
            }
            // Remove the external accelerations (iterate backwards to avoid index issues)
            for (int i = extAccelsToRemove.Count - 1; i >= 0; i--) {
                int extAccelId = extAccelsToRemove[i];
                extAccels.Remove(extAccelId);
                extAccelPool.Free(extAccelId);
                ExternalAccel.ExtAccelDataFree(ref gePhysicsJob, extAccelId);
            }
            // Update the index list if any external accelerations were removed
            if (extAccelsToRemove.Count > 0) {
                ExtAccelIndexUpdate();
            }
            // Clear the self_integrating flag if this body had self-integrating external accelerations
            if (hadSelfIntegrating) {
                bodies.self_integrating[id] = false;
            }

            // Colliders
            // Just have a list of collision info objects, need to scan to see if any reference this id
            // (no back ref). Only one collider per body.
            int toRemove = -1;
            foreach (int i in colliders) {
                if (gePhysicsJob.collisionInfo[i].id == id) {
                    toRemove = i;
                    break;
                }
            }
            if (toRemove >= 0) {
                colliders.Remove(toRemove);
                collisionInfoPool.Free(toRemove);
                CollidersUpdate();
            }

            numBodies--;
            if (geTrajectory != null) {
                geTrajectory.BodyRemove(id);
            }

            return true;
        }

        /// <summary>
        /// Add collision detection to a body. This results in the GEPhysicsCore performing
        /// comparisions to other bodies that habve collision detection enabled. 
        ///
        /// Collision detection is performed by checking if the separation of the bodies 
        /// is less than the sum of their radii (ie. as if there were spherical colliders).
        /// 
        /// This collision detection is internal to GECore and is not related to the Unity 
        /// physics collision detection system.
        ///
        /// The mass provided to the collision system is *not* necessarily the same as the
        /// gravitational mass. For example it makes sense to treat a satellite as having zero
        /// gravitational mass (so force of satellite on Earth is not computed) but for the
        /// purpose of a collision a mass e.g. 2000 kg is appropriate.
        ///
        /// </summary>
        /// <param name="id">body id</param>
        /// <param name="radius">spherical size of this body in default World units</param>
        /// <param name="bounceF">bounce factor for elastic collisions</param>
        /// <param name="collisionType">interaction type (absorb/bounce)</param>
        /// <param name="mass">inertial mass</param>
        public void ColliderAddToBody(int id,
                                        double radius,
                                        double bounceF,
                                        GEPhysicsCore.CollisionType collisionType,
                                        double mass)
        {
            int c = collisionInfoPool.Alloc(ref gePhysicsJob.collisionInfo);
            GEPhysicsCore.ColliderInfo cInfo = new GEPhysicsCore.ColliderInfo {
                id = id,
                cType = collisionType,
                radius = geScaler.ScaleLenWorldToGE(radius),
                bounceFactor = bounceF,
                // mass is in default world units, since in bounce it's units cancel out.
                massInertial = mass
            };
            gePhysicsJob.collisionInfo[c] = cInfo;
            colliders.Add(c);
            // update geJob
            CollidersUpdate();
        }


        private void CollidersUpdate()
        {
            // update geJob
            gePhysicsJob.colliders.Dispose();
            gePhysicsJob.colliders = new NativeArray<int>(colliders.Count, Allocator.Persistent);
            for (int i = 0; i < colliders.Count; i++) {
                gePhysicsJob.colliders[i] = colliders[i];
            }
        }

        private int CenterId(int id)
        {
            int centerId = -1;
            switch (bodies.propType[id]) {
                case GEPhysicsCore.Propagator.KEPLER:
                    centerId = gePhysicsJob.keplerPropInfo[bodies.propIndex[id]].centerId;
                    break;
                case GEPhysicsCore.Propagator.PKEPLER:
                    centerId = gePhysicsJob.pKeplerPropInfo[bodies.propIndex[id]].centerId;
                    break;
                case GEPhysicsCore.Propagator.SGP4_RAILS:
                    centerId = gePhysicsJob.sgp4PropInfo[bodies.propIndex[id]].centerId;
                    break;
                default:
                    centerId = -1;
                    break;
            }
            return centerId;
        }

        /// <summary>
        /// Add a maneuver to the list of pending maneuvers. If no timeOffset is given it is assumed
        /// the t_relative in the maneuver is to be added to the current time to establish the overall
        /// maneuver time. 
        /// 
        /// If an explicit timeOffset is provided, use it.
        ///
        /// If the body the maneuver is being applied to is patched, and the maneuver has isPatched then instead of adding
        /// a maneuver we can add a patch. It is assumed that maneuvers for a given id are added in increasing time order. 
        /// (Note: GEPhysicsJob cannot easily create a patch "on the fly" since it would
        /// need to dynamically change it's data structures and let GE know. This could be solved but defer for now).
        ///
        /// Note that the GEManeuver instances are kept in GE but must be converted into a GEManeuverStruct for the
        /// IJob (it cannot handle classes).
        /// 
        /// Life Cycle:
        /// Maneuvers are added to pendingManeuvers. GECore then copies this list of manuvers into a NativeList<GEManeuverStruct>.
        /// Once a maneuver has been executed by the physics core it generates a PhysEvent. PhysicsLoopComplete() examines
        /// those maneuvers that have been executed and runs their callbacks (if present). These are then removed from the masterManeuverList
        /// and the pending
        /// 
        /// Note that this means the callbacks will occur when Evolve() loop completes and not when the maneuver occurs. As a result the
        /// current GECore state will not correspond to the state at the time of the maneuver. The exact time, position and before/after velocity
        /// of the maneuver is contained in the PhysEvent that reports the maneuver was completed. 
        /// 
        /// </summary>
        /// <param name="id">id of body</param>
        /// <param name="m">maneuver to be added</param>
        /// <param name="timeOffsetWorldTime">(optional) specific offset for maneuver. By default current world time is used.</param>
        public void ManeuverAdd(int id, GEManeuver m, double timeOffsetWorldTime = double.NaN)
        {
            if (running) {
                Debug.LogWarning("AddManeuver while GE is running. Rejected.");
                return;
            }
            double timeOffsetGE;
            if (double.IsNaN(timeOffsetWorldTime)) {
                timeOffsetGE = t_physics;
            } else {
                timeOffsetGE = geScaler.ScaleTimeWorldToGE(timeOffsetWorldTime);
            }
            bool addPatch = m.hasRelativeRV && bodies.patchIndex[id] >= 0;
            if (bodies.patchIndex[id] >= 0 && !m.hasRelativeRV) {
                Debug.LogErrorFormat("Maneuver {0} on patched body without relative RV. Manuever rejected.", m.info);
                return;
            }

            if (addPatch) {
                if (!m.hasRelativeRV) {
                    Debug.LogErrorFormat("Maneuver {0} on patched body without relative RV. Manuever rejected.", m.info);
                    return;
                }
                GEBodyState mState = new GEBodyState(m.r_relative, m.v_relative, m.t_relative);
                double tWorld = geScaler.ScaleTimeGEToWorld(timeOffsetGE);
                tWorld += m.t_relative;
                PatchCreateAndAdd(id, m.centerId, mState, m.prop, tWorld);
#pragma warning disable 162     // disable unreachable code warning
                if (DEBUG) {
                    Debug.LogFormat("Add maneuver as patch {0} t_rel={1} type={2} vP={3} prop={4}",
                                    m.info, m.t_relative, m.type, m.velocityParam, m.prop);
                    Debug.Log(DumpAll("Added patch"));
                }
#pragma warning restore 162
            } else {
                // NOT A PATCHED BODY
                // by default add the current time to all maneuver times
#pragma warning disable 162     // disable unreachable code warning
                if (DEBUG) {
                    if (m.prop != GEPhysicsCore.Propagator.UNASSIGNED && m.prop != bodies.propType[id]) {
                        Debug.LogWarningFormat("Maneuver {0} has prop {1} but body {2} has prop {3}. Do you need patch mode?",
                                            m.info, m.prop, id, bodies.propType[id]);
                    }
                }
#pragma warning restore 162

                m.uniqueId = next_maneuver_id++;
                // wrap around (remote possibility)
                if (next_maneuver_id < 0) {
                    next_maneuver_id = 0;
                    m.uniqueId = 0;
                }
                pendingManeuvers.Add(m);
#pragma warning disable 162     // disable unreachable code warning
                if (DEBUG) {
                    Debug.LogFormat("Add maneuver {0} t_rel={1} type={2} vP={3}",
                                    m.info, m.t_relative, m.type, m.velocityParam);
                }
#pragma warning restore 162
                GEManeuverStruct mStruct = new GEManeuverStruct(m, id, geScaler, timeOffsetGE);
                // need to maintain maneuver list in time order but alas NativeList has no insertAt
                // hence we keep a non-Native shadow
                int addAt = 0;
                while ((addAt < masterManeuverList.Count) && (mStruct.t > masterManeuverList[addAt].t))
                    addAt++;
                masterManeuverList.Insert(addAt, mStruct);
                gePhysicsJob.maneuvers.Clear();
                foreach (GEManeuverStruct maneuver in masterManeuverList)
                    gePhysicsJob.maneuvers.Add(maneuver);
            }
            if (geTrajectory != null) {
                trajReset = true;
            }
        }

        /// <summary>
        /// Create a patch for a patched body. State is in world units relative to the center of the body.
        ///
        /// A patch is added at the end of the patch list. If the patch list
        /// contains patches that preceed tWorld then they are removed. 
        /// 
        /// If the propagator is EPHEMERIS then the ephemData is used to initialize the propagator.
        /// 
        /// </summary>
        /// <param name="id"></param>
        /// <param name="centerId"></param>
        /// <param name="mState"></param>
        /// <param name="prop"></param>
        /// <param name="tWorld"></param>
        /// <param name="ephemData"></param>
        /// <param name="earlyPropagation"></param>
        /// <param name="epochWorldTime"></param>
        public void PatchCreateAndAdd(int id,
                                    int centerId,
                                    GEBodyState mState,
                                    GEPhysicsCore.Propagator prop,
                                    double tWorld,
                                    EphemerisData ephemData = null,
                                    GEPhysicsCore.EarlyPropagation earlyPropagation = GEPhysicsCore.EarlyPropagation.ERROR_INACTIVATE,
                                    double epochWorldTime = double.NaN)
        {
            // PATCHed body
            // add a patch. We assume any maneuver/patch removal and time ordering is handled externally 
            int p = bodies.patchIndex[id];
            GEPhysicsCore.Propagator oldProp = bodies.propType[id];

            // need to know current patch for this time, since if we replace it we need to change the current patch
            // reference and potentially change the propagator (i.e. move from one body type list to another)
            double tGE = geScaler.ScaleTimeWorldToGE(tWorld);
            int p_first = gePhysicsJob.PatchFirst(p);

            // remove all patches beyond the one for this time (may remove this patch)
            int p_first_to_remove = gePhysicsJob.PatchForTime(tGE, p);
            // if this patch is not FOR THIS TIME, then we need to remove the next patch
            if (p_first_to_remove >= 0) {
                if (math.abs(gePhysicsJob.patches[p_first_to_remove].timeStartGE - tGE) > 1E-9) {
                    p_first_to_remove = gePhysicsJob.patches[p_first_to_remove].nextEntry;
                }
            } else {
                if (tGE < gePhysicsJob.patches[p_first].timeStartGE) {
                    p_first_to_remove = p_first;
                }
            }
            PatchesRemoveFromIndex(p_first_to_remove);
            int p_last = gePhysicsJob.PatchLast(p_first);

            // Create a propagator for this segment
            if (centerId < 0) {
                Debug.LogError("Center ID not assigned for patched maneuver. Manuever rejected. ");
                return;
            }
            // What propagator to use?
            if (prop == GEPhysicsCore.Propagator.UNASSIGNED) {
                prop = bodies.propType[id];
            }

            // Add a patch based on this maneuver
            int p_new = patchPool.Alloc(ref gePhysicsJob.patches);
            int propIndex = CreatePropagatorForPatch(mState, prop, tGE, centerId, ephemData);
            if (propIndex < 0) {
                return;
            }
            GEPhysicsCore.PatchInfo patch = new() {
                nextEntry = -1,
                prevEntry = p_last,
                prop = prop,
                propIndex = propIndex,
                timeStartGE = tGE,
                timeEndGE = -1
            };
            gePhysicsJob.patches[p_new] = patch;

            if (p_last >= 0) {
                GEPhysicsCore.PatchInfo currentPatch = gePhysicsJob.patches[p_last];
                currentPatch.nextEntry = p_new;
                currentPatch.timeEndGE = patch.timeStartGE;
                gePhysicsJob.patches[p_last] = currentPatch;
            }
            // by default point to the start if it was not replaced
            if (gePhysicsJob.patches[p_first].timeStartGE >= 0) {
                bodies.patchIndex[id] = p_first;
            } else {
                bodies.patchIndex[id] = p_new;
            }

            // if this patch is active now, set to current patch
            if (math.abs(t_physics - patch.timeStartGE) < 1E-9) {
                bodies.patchIndex[id] = p_new;
                bodies.propIndex[id] = propIndex;
                bodies.propType[id] = prop;
                bodies.r[id] = mState.r;
                bodies.v[id] = mState.v;
                if (prop != oldProp) {
                    // need to change the propagator
                    gePhysicsJob.IndexListRemove(oldProp, id);
                    gePhysicsJob.IndexListAdd(prop, id);
                }
            }
            if (tGE < t_physics) {
                // need to set new position and velocity
                GEBodyState state = new GEBodyState();
                double tWorldNow = geScaler.ScaleTimeGEToWorld(t_physics);
                StateByIdAtTime(id, tWorldNow, ref state, geUnits: true);
                bodies.r[id] = state.r;
                bodies.v[id] = state.v;
            }
#pragma warning disable 162     // disable unreachable code warning
            if (DEBUG) {
                Debug.LogFormat($"PatchCreateAndAdd id={id} p_new={p_new} tStart={tWorld}");
            }
#pragma warning restore 162
        }


        private void PatchAppendFixedEndpoint(int id, double3 r, double3 v, double tGE)
        {
            // PATCHed body
            // add a patch. We assume any maneuver/patch removal and time ordering is handled externally 
            int p_last = gePhysicsJob.PatchLast(bodies.patchIndex[id]);

            // create a new patch
            GEPhysicsCore.PatchInfo patch = new() {
                nextEntry = -1,
                prevEntry = p_last,
                timeStartGE = tGE,
                timeEndGE = -1,
                prop = GEPhysicsCore.Propagator.FIXED,
                propIndex = -1,
                r_fixed = r,
                v_fixed = v
            };
            int p_new = patchPool.Alloc(ref gePhysicsJob.patches);
            gePhysicsJob.patches[p_new] = patch;

            // link to old patch
            GEPhysicsCore.PatchInfo currentPatch = gePhysicsJob.patches[p_last];
            currentPatch.nextEntry = p_new;
            currentPatch.timeEndGE = patch.timeStartGE;
            gePhysicsJob.patches[p_last] = currentPatch;

            // set the current patch
            bodies.patchIndex[id] = p_new;
            bodies.propIndex[id] = patch.propIndex;
            bodies.propType[id] = patch.prop;
            bodies.r[id] = r;
            bodies.v[id] = v;
        }

        /// <summary>
        /// Copy patches from one body to another. The patches are copied from the source body at the specified time to
        /// the end the patch list. 
        /// 
        /// If the fromTime is "now", then the current prop info (active patch, prop type etc.) are updated. 
        /// </summary>
        /// <param name="toBodyId"></param>
        /// <param name="fromBodyId"></param>
        /// <param name="fromTime"></param>
        public void PatchesCopyFrom(int toBodyId, int fromBodyId, double fromWorldTime)
        {
            // PATCHed body
            // add a patch. We assume any maneuver/patch removal and time ordering is handled externally 
            int p = bodies.patchIndex[toBodyId];

            // need to know current patch for this time, since if we replace it we need to change the current patch
            // reference and potentially change the propagator (i.e. move from one body type list to another)
            double tGE = geScaler.ScaleTimeWorldToGE(fromWorldTime);
            int p_for_t = gePhysicsJob.PatchForTime(tGE, p);

            // remove all patches beyond the one for this time (may remove this patch)
            PatchesRemoveFromBody(toBodyId, fromWorldTime);

            int prev = p_for_t;
            // if the p_for_t was removed, then prevEntry is the previous patch
            if (gePhysicsJob.patches[p_for_t].timeStartGE < 0) {
                prev = gePhysicsJob.patches[p_for_t].prevEntry;
            }

            int p_from = gePhysicsJob.PatchForTime(tGE, bodies.patchIndex[fromBodyId]);

            while (p_from >= 0) {
                GEPhysicsCore.PatchInfo patch = new() {
                    nextEntry = -1,
                    prevEntry = prev,
                    timeStartGE = gePhysicsJob.patches[p_from].timeStartGE,
                    timeEndGE = -1,
                    prop = gePhysicsJob.patches[p_from].prop,
                };
                // need to make a copy of the propagator
                patch.propIndex = CopyPropagator(gePhysicsJob.patches[p_from].prop, gePhysicsJob.patches[p_from].propIndex);
                int p_new = patchPool.Alloc(ref gePhysicsJob.patches);
                gePhysicsJob.patches[p_new] = patch;
                // current patch
                if (prev >= 0) {
                    GEPhysicsCore.PatchInfo currentPatch = gePhysicsJob.patches[prev];
                    currentPatch.nextEntry = p_new;
                    currentPatch.timeEndGE = patch.timeStartGE;
                    gePhysicsJob.patches[prev] = currentPatch;
                } else {
                    // we needed to re-add the initial patch, so 
                    bodies.patchIndex[toBodyId] = p_new;
                    bodies.propIndex[toBodyId] = patch.propIndex;
                    bodies.propType[toBodyId] = patch.prop;
                }
                p_from = gePhysicsJob.patches[p_from].nextEntry;
                prev = p_new;
            }

        }

        private int CopyPropagator(GEPhysicsCore.Propagator prop, int propFromIndex)
        {
            switch (prop) {
                case GEPhysicsCore.Propagator.KEPLER:
                    int p = keplerPool.Alloc(ref gePhysicsJob.keplerPropInfo);
                    gePhysicsJob.keplerPropInfo[p] = new KeplerPropagator.PropInfo(
                        gePhysicsJob.keplerPropInfo[propFromIndex]);
                    return p;

                case GEPhysicsCore.Propagator.KEPLER_COE:
                    p = keplerCoePool.Alloc(ref gePhysicsJob.keplerCoePropInfo);
                    gePhysicsJob.keplerCoePropInfo[p] = new KeplerCOEPropagator.PropInfo(
                        gePhysicsJob.keplerCoePropInfo[propFromIndex]);
                    return p;

                case GEPhysicsCore.Propagator.PKEPLER:
                    p = pkeplerPool.Alloc(ref gePhysicsJob.pKeplerPropInfo);
                    gePhysicsJob.pKeplerPropInfo[p] = new PKeplerPropagator.PropInfo(
                        gePhysicsJob.pKeplerPropInfo[propFromIndex]);
                    return p;

                case GEPhysicsCore.Propagator.SGP4_RAILS:
                    p = sgp4Pool.Alloc(ref gePhysicsJob.sgp4PropInfo);
                    gePhysicsJob.sgp4PropInfo[p] = new SGP4Propagator.PropInfo(
                        gePhysicsJob.sgp4PropInfo[propFromIndex]);
                    return p;
            }
            return -1;
        }

        private int CreatePropagatorForPatch(GEBodyState mState, GEPhysicsCore.Propagator prop, double tstartGE, int centerId, EphemerisData ephemData = null)
        {
            switch (prop) {
                case GEPhysicsCore.Propagator.KEPLER:
                    return CreateKeplerPropagator(mState, tstartGE, centerId);

                case GEPhysicsCore.Propagator.KEPLER_COE:
                    return CreateKeplerCoePropagator(mState, tstartGE, centerId);

                case GEPhysicsCore.Propagator.PKEPLER:
                    return CreatePKeplerPropagator(mState, tstartGE, centerId);

                case GEPhysicsCore.Propagator.SGP4_RAILS:
                    return CreateSGP4Propagator(mState, centerId);

                case GEPhysicsCore.Propagator.EPHEMERIS:
                    return CreateEphemerisPropagator(mState, tstartGE, centerId, ephemData);

                case GEPhysicsCore.Propagator.FIXED:
                    // used when e.g. an SGP4 satellite decays and want to hold last position so 
                    // time reverse can be used to recover pre-decayed state
                    Debug.LogError("FIXED propagator not supported for patched maneuver");
                    return -1;

                default:
                    Debug.LogErrorFormat("Unsupported propagator {0} for patched maneuver", prop);
                    return -1;
            }
        }

        private int CreateKeplerPropagator(GEBodyState mState, double tstartGE, int centerId)
        {
            int propIndex = keplerPool.Alloc(ref gePhysicsJob.keplerPropInfo);
            double3 r = mState.r * geScaler.ScaleLenWorldToGE(1.0);
            double3 v = mState.v * geScaler.ScaleVelocityWorldToGE(1.0);
            int keplerDepth = 0;
            if (bodies.propType[centerId] == GEPhysicsCore.Propagator.KEPLER) {
                keplerDepth = gePhysicsJob.keplerPropInfo[bodies.propIndex[centerId]].keplerDepth + 1;
            }
            gePhysicsJob.keplerPropInfo[propIndex] =
                    new KeplerPropagator.PropInfo(r, v, tstartGE, bodies.mu[centerId], centerId, keplerDepth);
            return propIndex;
        }

        private int CreateKeplerCoePropagator(GEBodyState mState, double tstartGE, int centerId)
        {
            int propIndex = keplerCoePool.Alloc(ref gePhysicsJob.keplerCoePropInfo);
            Orbital.COE coe = Orbital.RVtoCOE(mState.r, mState.v, bodies.mu[centerId]);
            gePhysicsJob.keplerCoePropInfo[propIndex] = new KeplerCOEPropagator.PropInfo(coe, tstartGE, centerId);
            return propIndex;
        }

        private int CreatePKeplerPropagator(GEBodyState mState, double tstartGE, int centerId)
        {
            int propIndex = pkeplerPool.Alloc(ref gePhysicsJob.pKeplerPropInfo);
            // maneuver is in world units, convert to km and km/s
            GBUnits.Units wUnits = geScaler.WorldUnits();
            double3 rKm = mState.r * GBUnits.DistanceConversion(wUnits, GBUnits.Units.SI_km);
            double3 vKm = mState.v * GBUnits.VelocityConversion(wUnits, GBUnits.Units.SI_km);
            Orbital.COE coeKm_init = Orbital.RVtoCOE(rKm, vKm, GBUnits.earthMuKm);
            gePhysicsJob.pKeplerPropInfo[propIndex] = new PKeplerPropagator.PropInfo(coeKm_init, tstartGE, centerId);
            return propIndex;
        }

        private int CreateSGP4Propagator(GEBodyState mState, int centerId)
        {
            SGP4SatData satData = new SGP4SatData();
            // assuming t_relative is in seconds
            double timeJD = TimeUtils.SecToJD(mState.t) + TimeJulianDays();
            GBUnits.Units wUnits = geScaler.WorldUnits();
            double3 rKm = mState.r * GBUnits.DistanceConversion(wUnits, GBUnits.Units.SI_km);
            double3 vKm = mState.v * GBUnits.VelocityConversion(wUnits, GBUnits.Units.SI_km);
            Orbital.COE coeSgp = Orbital.RVtoCOE(rKm, vKm, GBUnits.earthMuKm);
            satData.InitFromCOE(coeSgp, timeJD);
            int propIndex = sgp4Pool.Alloc(ref gePhysicsJob.sgp4PropInfo);
            gePhysicsJob.sgp4PropInfo[propIndex] = new SGP4Propagator.PropInfo(satData, centerId);
            return propIndex;
        }

        private int CreateEphemerisPropagator(GEBodyState mState, double tstartGE, int centerId, EphemerisData ephemData)
        {
            int propIndex = ephemPool.Alloc(ref gePhysicsJob.ephemPropInfo);
            EphemerisPropagator.EphemDataAlloc(ephemData, ref gePhysicsJob, geScaler, centerId, propIndex);
            return propIndex;
        }

        /// <summary>
        /// Get the patch data for a body at a specific world time.
        /// 
        /// The patch data is a struct that contains the start and end times of the patch and the propagator type.
        /// </summary>
        /// <param name="id"></param>
        /// <param name="tWorld"></param>
        /// <returns></returns>
        public PatchData PatchDataForIdAtTime(int id, double tWorld)
        {
            double tGE = geScaler.ScaleTimeWorldToGE(tWorld);
            int p = gePhysicsJob.PatchForTime(tGE, bodies.patchIndex[id]);
            if (p < 0) {
                // does the body allow early prop?
                bool allowEarlyProp = bodies.earlyPropagation[id] == GEPhysicsCore.EarlyPropagation.PROPAGATE;
                if (allowEarlyProp) {
                    p = gePhysicsJob.PatchFirst(bodies.patchIndex[id]); ;
                } else {
                    return new PatchData {
                        tStartWorld = double.NaN,
                        tEndWorld = double.NaN,
                        propType = GEPhysicsCore.Propagator.UNASSIGNED,
                        centerId = -1
                    };
                }

            }
            GEPhysicsCore.PatchInfo patch = gePhysicsJob.patches[p];
            return new PatchData {
                tStartWorld = geScaler.ScaleTimeGEToWorld(patch.timeStartGE),
                tEndWorld = geScaler.ScaleTimeGEToWorld(patch.timeEndGE),
                propType = patch.prop,
                centerId = patch.CenterId(gePhysicsJob)
            };
        }


        internal int PatchIndexForIdAtTime(int id, double tWorld)
        {
            double tGE = geScaler.ScaleTimeWorldToGE(tWorld);
            int p = gePhysicsJob.PatchForTime(tGE, bodies.patchIndex[id]);
            if (p < 0) {
                // does the body allow early prop?
                bool allowEarlyProp = bodies.earlyPropagation[id] == GEPhysicsCore.EarlyPropagation.PROPAGATE;
                if (allowEarlyProp) {
                    p = gePhysicsJob.PatchFirst(bodies.patchIndex[id]); ;
                } else {
                    return -1;
                }

            }
            GEPhysicsCore.PatchInfo patch = gePhysicsJob.patches[p];
            return p;
        }

        /// <summary>
        /// Get the patch data for a body as a list of PatchData structs.
        /// 
        /// The patch data is a struct that contains the start and end times of the patch and the propagator type.
        /// </summary>
        /// <param name="id"></param>
        /// <returns></returns>
        public List<PatchData> PatchesForId(int id)
        {
            List<PatchData> patches = new List<PatchData>();
            if (bodies.patchIndex[id] < 0) {
                return patches;
            }
            int p = gePhysicsJob.PatchFirst(bodies.patchIndex[id]);
            while (p >= 0) {
                PatchData patch = new PatchData {
                    tStartWorld = geScaler.ScaleTimeGEToWorld(gePhysicsJob.patches[p].timeStartGE),
                    tEndWorld = geScaler.ScaleTimeGEToWorld(gePhysicsJob.patches[p].timeEndGE),
                    propType = gePhysicsJob.patches[p].prop,
                    centerId = gePhysicsJob.patches[p].CenterId(gePhysicsJob)
                };
                patches.Add(patch);
                p = gePhysicsJob.patches[p].nextEntry;
            }
            return patches;
        }

        public void ManeuverListAdd(List<GEManeuver> maneuvers, int id, double timeOffsetWorldTime = double.NaN)
        {
            foreach (GEManeuver m in maneuvers) {
                ManeuverAdd(id, m, timeOffsetWorldTime);
            }
        }

        /// <summary>
        /// Fill in a state using the id (unique to this GE). The state is at the current world
        /// time. Note that the current world time may slightly exceed the time that was requested
        /// in the preceeding Evolve() or Schedule() due to the finite step sixe in the numerical 
        /// integration. 
        ///
        /// Return value is in default world units unless geUnits are explicitly requested. 
        /// </summary>
        /// <param name="id">id of body</param>
        /// <param name="state">struct to hold state info (ref)</param>
        /// <param name="geUnits">return value in internal GE units (by default will be in world units)</param>
        /// <param name="maintainCoRo">maintain the co-rotating frame (default false)</param>
        /// <returns></returns>
        public bool StateById(int id, ref GEBodyState state, bool geUnits = false, bool maintainCoRo = false)
        {
            if (running) {
                Debug.LogWarningFormat("Get state while running for id {0}. Queue command instead??", id);
                return false;
            }

            if (bodies.propType[id] == GEPhysicsCore.Propagator.UNASSIGNED) {
                state.r = new double3(double.NaN, double.NaN, double.NaN);
                state.v = state.r;
                state.t = double.NaN;
                return false;
            } else {
                state.r = bodies.r[id];
                state.v = bodies.v[id];
                state.t = t_physics;
            }
            // When in CR3BP most queries (transfers etc.) want the inertial R, V. Provide this unless the
            // optional arg countermands it. 
            if (!maintainCoRo && refFrame == ReferenceFrame.COROTATING_CR3BP) {
                (double3 r, double3 v) = CR3BP.FrameRotatingToInertial(state.r, state.v, state.t);
                state.r = r;
                state.v = v;
            }
            if (!geUnits) {
                state.r *= geScaler.ScaleLenGEToWorld(1.0);
                state.v *= geScaler.ScaleVelocityGEToWorld(1.0);
                state.t *= geScaler.ScaleTimeGEToWorld(1.0);
            }
            return true;
        }

        /// <summary>
        /// Get the type of the body. Does not change at run time, so ok to just return it now.
        /// </summary>
        /// <param name="id"></param>
        /// <returns></returns>
        public GEPhysicsCore.Propagator PropagatorTypeById(int id)
        {
            return bodies.propType[id];
        }

        /// <summary>
        /// Determine the state at a time that is slightly earlier than the current world time. 
        ///
        /// This can be used when a high precision state is required. For example:
        /// - GECore is asked to evolve to t=10.1
        /// - due to internal dt step it evolves to t=10.13
        /// - the state at exactly t=10.1 is of interest
        /// In this case asking for the state at time 10.1 will result in an linear interpolation 
        /// of r and v by the delta. 
        ///
        /// </summary>
        /// <param name="id">body to retrieve state for</param>
        /// <param name="state">struct to hold state information (ref)</param>
        /// <param name="atTime">specific world time to intepolate state to (less than current world time)</param>
        /// <param name="geUnits">return value in internal GE units (by default will be in world units)</param>
        /// <param name="maintainCoRo">maintain the co-rotating frame (default false)</param>
        /// <returns></returns>
        public bool StateByIdInterpolated(int id, ref GEBodyState state, double atTime, bool geUnits = false, bool maintainCoRo = false)
        {
            if (running) {
                Debug.LogWarningFormat("Get state while running for id {0}. Queue command instead??", id);
                return false;
            }
            // assuming atTime is earlier than time now
            double deltaT = geUnits ?
                gePhysConfig[T_PARAM] - atTime :
                geScaler.ScaleTimeGEToWorld(atTime) - gePhysConfig[T_PARAM];

            state.t = atTime;

            double3 a = gePhysicsJob.Acceleration(id);

            if (bodies.propType[id] == GEPhysicsCore.Propagator.UNASSIGNED) {
                state.r = new double3(double.NaN, double.NaN, double.NaN);
                state.v = state.r;
                return false;
            } else {
                if (!maintainCoRo && refFrame == ReferenceFrame.COROTATING_CR3BP) {
                    (double3 r, double3 v) = CR3BP.FrameRotatingToInertial(state.r, state.v, state.t);
                    state.r = r;
                    state.v = v;
                }
                state.r = bodies.r[id] - deltaT * state.v;
                state.v = bodies.v[id] - deltaT * a;
            }

            if (!geUnits) {
                state.r *= geScaler.ScaleLenGEToWorld(1.0);
                state.v *= geScaler.ScaleVelocityGEToWorld(1.0);
            }
            return true;
        }

        /// <summary>
        /// Get the world position of a body of specified id.
        /// </summary>
        /// <param name="id"></param>
        /// <param name="maintainCoRo">maintain the co-rotating frame (default false)</param>
        /// <returns></returns>
        public double3 RWorldById(int id, bool maintainCoRo = false)
        {
            if (running) {
                Debug.LogWarningFormat("Get state while running for id {0}. Queue command instead??", id);
                return new double3(double.NaN, double.NaN, double.NaN);
            }

            if (bodies.propType[id] == GEPhysicsCore.Propagator.UNASSIGNED) {
                return new double3(double.NaN, double.NaN, double.NaN);
            }
            double3 r = bodies.r[id];
            if (!maintainCoRo && refFrame == ReferenceFrame.COROTATING_CR3BP) {
                (double3 r1, double3 v) = CR3BP.FrameRotatingToInertial(r, bodies.v[id], t_physics);
                r = r1;
            }
            return r * geScaler.ScaleLenGEToWorld(1.0);
        }

        /// <summary>
        /// Determine the relative state of a body at a specific world time. This time may be in the past or future.
        ///
        /// If the propagator does not support this, the state will be set to NaN.
        ///
        /// This only does relative propagation. TODO: extend to absolute by propagating the center body etc.
        /// </summary>
        /// <param name="t">world time to propagate to</param>
        /// <param name="id">id of body to propagate</param>
        /// <param name="state">struct to hold state info (ref)</param>
        /// <param name="geUnits">return value in internal GE units (by default will be in world units)</param>
        /// <param name="maintainCoRo">maintain the co-rotating frame (default false)</param>
        /// <returns>flag indicating state could be computed</returns>
        public bool StateByIdAtTime(int id, double tWorld, ref GEBodyState state, bool geUnits = false, bool maintainCoRo = false)
        {
            if (running) {
                Debug.LogWarningFormat("Get state while running for id {0}. Queue command instead??", id);
                return false;
            }

            if (bodies.propType[id] == GEPhysicsCore.Propagator.UNASSIGNED) {
                state.r = new double3(double.NaN, double.NaN, double.NaN);
                state.v = state.r;
                state.t = double.NaN;
                return false;
            }

            int propIndex;
            double tGE = geScaler.ScaleTimeWorldToGE(tWorld);
            GEPhysicsCore.Propagator propType = bodies.propType[id];
            // if the body is patched, find the patch that is active at the requested time
            if (bodies.patchIndex[id] >= 0) {
                int patch = PatchIndexForIdAtTime(id, tWorld);
                propIndex = gePhysicsJob.patches[patch].propIndex;
                propType = gePhysicsJob.patches[patch].prop;
            } else {
                propIndex = bodies.propIndex[id];
            }
            int center_id = -1;
            switch (propType) {
                case GEPhysicsCore.Propagator.KEPLER:
                    KeplerPropagator.EvolveRelative(tGE, propIndex, ref gePhysicsJob.keplerPropInfo, ref state);
                    center_id = gePhysicsJob.keplerPropInfo[propIndex].centerId;
                    break;

                case GEPhysicsCore.Propagator.KEPLER_COE:
                    KeplerCOEPropagator.EvolveRelative(tGE, propIndex, ref gePhysicsJob.keplerCoePropInfo, ref state);
                    center_id = gePhysicsJob.keplerCoePropInfo[propIndex].centerId;
                    break;

                case GEPhysicsCore.Propagator.EPHEMERIS:
                    center_id = gePhysicsJob.ephemPropInfo[propIndex].centerId;
                    double mu = 0;
                    if (center_id >= 0) {
                        mu = bodies.mu[center_id];
                    }
                    EphemerisPropagator.EvolveRelative(tGE, propIndex, ref gePhysicsJob.ephemPropInfo, ref gePhysicsJob.ephemerisData, ref state, mu);
                    break;

                case GEPhysicsCore.Propagator.PKEPLER:
                    PKeplerPropagator.EvolveRelative(tGE, propIndex, ref gePhysicsJob.pKeplerPropInfo, ref state, geConfig.scaleTtoSec, geConfig.scaleLtoKm, geConfig.scaleKmsecToV);
                    center_id = gePhysicsJob.pKeplerPropInfo[propIndex].centerId;
                    break;

                case GEPhysicsCore.Propagator.SGP4_RAILS:
                    SGP4Propagator.Evolve(tGE * geConfig.scaleTtoSec, ref gePhysicsJob.sgp4PropInfo, propIndex, ref state, geConfig.scaleLtoKm, geConfig.scaleKmsecToV, geConfig.startTimeJD);
                    center_id = gePhysicsJob.sgp4PropInfo[propIndex].centerId;
                    break;

                case GEPhysicsCore.Propagator.FIXED:
                    state.r = bodies.r[id];
                    state.v = bodies.v[id];
                    state.t = t_physics;
                    break;

                case GEPhysicsCore.Propagator.PLANET_SURFACE:
                    RotationPropagator.EvolveRelative(tGE, propIndex, ref gePhysicsJob.rotationPropInfo, ref state);
                    center_id = gePhysicsJob.rotationPropInfo[propIndex].centerId;
                    break;

                default:
                    state.r = new double3(double.NaN, double.NaN, double.NaN);
                    state.v = state.r;
                    state.t = double.NaN;
                    break;
            }
            // if the state is NaN, return false
            if (GravityMath.HasNaN(state.r) || GravityMath.HasNaN(state.v)) {
                return false;
            }
            // When in CR3BP most queries (transfers etc.) want the inertial R, V. Provide this unless the
            // optional arg countermands it. 
            if (!maintainCoRo && refFrame == ReferenceFrame.COROTATING_CR3BP) {
                (double3 r, double3 v) = CR3BP.FrameRotatingToInertial(state.r, state.v, state.t);
                state.r = r;
                state.v = v;
            }

            if (center_id >= 0) {
                // need to get position of center body at time in GE units
                GEBodyState centerState = new GEBodyState();
                StateByIdAtTime(center_id, tWorld, ref centerState, geUnits: true);
                state.r += centerState.r;
                state.v += centerState.v;
            }

            if (!geUnits) {
                state.r *= geScaler.ScaleLenGEToWorld(1.0);
                state.v *= geScaler.ScaleVelocityGEToWorld(1.0);
                state.t *= geScaler.ScaleTimeGEToWorld(1.0);
            }
            return true;
        }

        /// <summary>
        /// Return the propagator type for the specified id.
        /// </summary>
        /// <param name="id"></param>
        /// <returns></returns>
        public GEPhysicsCore.Propagator PropagatorTypeForId(int id)
        {
            return gePhysicsJob.bodies.propType[id];
        }

        /// <summary>
        /// Provide the state of a body relative to another body (typically the center body which 
        /// the first body orbits.)
        /// </summary>
        /// <param name="bodyId">id for which state info is requested</param>
        /// <param name="centerId">id for which state is relative to</param>
        /// <param name="bodyState">struct to hold the state information (ref)</param>
        /// <param name="geUnits">return value in internal GE units (by default will be in world units)</param>
        /// <returns></returns>
        public bool StateByIdRelative(int bodyId, int centerId, ref GEBodyState bodyState, bool geUnits = false)
        {
            bool ok = StateById(bodyId, ref bodyState, geUnits);
            if (!ok) {
                return false;
            }
            GEBodyState centerState = new GEBodyState();

            ok = StateById(centerId, ref centerState, geUnits);
            if (!ok) {
                return false;
            }
            bodyState.r -= centerState.r;
            bodyState.v -= centerState.v;
            return true;
        }

        /// <summary>
        /// Set the (r,v) state of a body with the indicated id. 
        /// 
        /// By default the state is set in absoluteworld units. If geUnits is true the state is set in internal
        /// GE units.
        /// 
        /// In some cases it is useful to set a state by specifying the (r,v) relative to another body. This
        /// is done by setting relativeCenterId to the id of the body to which the state is relative.
        /// 
        /// This can be used for all propagators with the exception of EPHEMERIS.
        /// 
        /// If the body is patched a state set will result in a new patch being added. Setting the state
        /// of a patched body must have a relative center ID specified.
        /// 
        /// If the relative centerId is provided, the state is set relative to the center body.
        /// 
        /// </summary>
        /// <param name="id"></param>
        /// <param name="s"></param>
        /// <param name="velocityOnly"></param>
        /// <param name="relativeCenterId"></param>
        /// <param name="geUnits"></param>
        /// <returns></returns>
        public bool StateSetById(int id,
                                 GEBodyState s,
                                 bool velocityOnly = false,
                                 int relativeCenterId = -1,
                                 bool geUnits = false)
        {
            if (running) {
                Debug.LogWarningFormat("Changing state while running for id {0}. Queue command instead??", id);
            }

            if (bodies.patchIndex[id] >= 0) {
                if (relativeCenterId == -1) {
                    Debug.LogErrorFormat("Cannot set state for patched body id {0} without a relative center id", id);
                    return false;
                }
                GEPhysicsCore.Propagator prop = bodies.propType[id];
                // uses world units
                double tWorld = geScaler.ScaleTimeGEToWorld(t_physics);
                PatchCreateAndAdd(id, relativeCenterId, s, prop, tWorld);
                if (!geUnits) {
                    s.r *= geScaler.ScaleLenWorldToGE(1.0);
                    s.v *= geScaler.ScaleVelocityWorldToGE(1.0);
                }
                bodies.r[id] = s.r + bodies.r[relativeCenterId];
                bodies.v[id] = s.v + bodies.v[relativeCenterId];
                return true;

            } else {
                // convert to GE internal units
                if (!geUnits) {
                    s.r *= geScaler.ScaleLenWorldToGE(1.0);
                    s.v *= geScaler.ScaleVelocityWorldToGE(1.0);
                }
                switch (bodies.propType[id]) {
                    case GEPhysicsCore.Propagator.FIXED:
                    case GEPhysicsCore.Propagator.GRAVITY:
                        // if there is a relative center, add it to the state
                        if (relativeCenterId >= 0) {
                            s.r += bodies.r[relativeCenterId];
                            s.v += bodies.v[relativeCenterId];
                        }
                        if (!velocityOnly) {
                            bodies.r[id] = s.r;
                        }
                        bodies.v[id] = s.v;
                        break;

                    case GEPhysicsCore.Propagator.SGP4_RAILS:
                    case GEPhysicsCore.Propagator.PKEPLER:
                    case GEPhysicsCore.Propagator.KEPLER:
                    case GEPhysicsCore.Propagator.KEPLER_COE:
                        if (velocityOnly) {
                            s.r = bodies.r[id];
                        }
                        // given an absolute r, v will need the relative RV wrt the center of
                        // the Kepler propagator.
                        int centerId = relativeCenterId;
                        if (relativeCenterId == -1) {
                            if (!velocityOnly) {
                                bodies.r[id] = s.r;
                            }
                            bodies.v[id] = s.v;
                            centerId = CenterId(id);
                            // absolute r, v given and RVT need the relative ones. 
                            s.r -= bodies.r[centerId];
                            s.v -= bodies.v[centerId];
                        } else {
                            // relative r, v given and bodies need the absolute ones. 
                            if (!velocityOnly) {
                                bodies.r[id] = s.r + bodies.r[relativeCenterId];
                            }
                            bodies.v[id] = s.v + bodies.v[relativeCenterId];
                        }
                        int pIndex = bodies.propIndex[id];
                        switch (bodies.propType[id]) {
                            case GEPhysicsCore.Propagator.PKEPLER:
                                Orbital.COE coeKm = Orbital.RVtoCOE(s.r, s.v, gePhysicsJob.pKeplerPropInfo[pIndex].coeKm_init.mu);
                                coeKm.ScaleLength(geConfig.scaleLtoKm);
                                gePhysicsJob.pKeplerPropInfo[pIndex] =
                                     new PKeplerPropagator.PropInfo(coeKm, t_physics, centerId);
                                break;

                            case GEPhysicsCore.Propagator.KEPLER:
                                gePhysicsJob.keplerPropInfo[pIndex] =
                                    new KeplerPropagator.PropInfo(s.r, s.v, t_physics, gePhysicsJob.keplerPropInfo[pIndex]);
                                break;

                            case GEPhysicsCore.Propagator.KEPLER_COE:
                                Orbital.COE coe = Orbital.RVtoCOE(s.r, s.v, gePhysicsJob.keplerCoePropInfo[pIndex].coeGE.mu);
                                gePhysicsJob.keplerCoePropInfo[pIndex] =
                                    new KeplerCOEPropagator.PropInfo(coe, t_physics, centerId);
                                break;

                            case GEPhysicsCore.Propagator.SGP4_RAILS:
                                Orbital.COE coeKm2 = Orbital.RVtoCOE(s.r, s.v, GBUnits.earthMuKm);
                                coeKm2.ScaleLength(geConfig.scaleLtoKm);
                                double3 rKm = s.r * geConfig.scaleLtoKm;
                                double3 vKm = s.v * geConfig.scaleKmsecToV;
                                SGP4SatData satData = new SGP4SatData();
                                double timeJD = TimeJulianDays();
                                satData.InitFromCOE(coeKm2, timeJD);
                                gePhysicsJob.sgp4PropInfo[pIndex] = new SGP4Propagator.PropInfo(satData, centerId);
                                gePhysicsJob.sgp4PropInfo[pIndex] = SGP4Propagator.ReInit(t_physics, timeJD, rKm, vKm, coeKm2, ref satData, gePhysicsJob.sgp4PropInfo[pIndex]);
                                break;

                            default:
                                throw new NotImplementedException("Internal error.");
                        }
                        break;


                    default:
                        throw new NotImplementedException("Not Supported. Cannot change state of ephemeris propagator");
                }
            }

            if (geTrajectory != null) {
                trajReset = true;
            }
            return true;
        }

        /// <summary>
        /// Apply a position and velocity transformation to all states. 
        /// 
        /// Useful to move entire system to e.g. center of mass position and velocity.
        /// </summary>
        /// <param name="r_offest"></param>
        public void RepositionAll(double3 r_offest)
        {
            if (running) {
                Debug.LogWarning("Changing state while running. Queue command instead??");
            }
            // update the instantenaous position of all prop bodies as well ( new positions from evolve will key
            // off off some reference body )
            foreach (int i in gePhysicsJob.massiveBodies) {
                bodies.r[i] += r_offest;
            }
            foreach (int i in gePhysicsJob.masslessBodies) {
                bodies.r[i] += r_offest;
            }
            foreach (int i in gePhysicsJob.keplerMassive) {
                bodies.r[i] += r_offest;
            }
            foreach (int i in gePhysicsJob.keplerMassless) {
                bodies.r[i] += r_offest;
            }
        }

        /// <summary>
        /// Return G * world mass for the body id given.
        /// </summary>
        /// <param name="id"></param>
        /// <returns>mass</returns>
        public double MuWorld(int id)
        {
            return geScaler.ScaleMuGEToWorld(bodies.mu[id]);
        }

        /// <summary>
        /// Return mass for a specified object.
        /// </summary>
        /// <param name="id"></param>
        /// <returns></returns>
        public double MassWorld(int id)
        {
            return geScaler.ScaleMassGEToWorld(bodies.mu[id]);
        }

        /// <summary>
        /// Get the classical orbital elements (COE) for the indicated body with respect to the center body.
        ///
        /// The most common way to do this is to take the R, V state of the body and use an algorithm (RVtoCOE)
        /// to determine the COE elements. This algorithm imposes a choice of when an orbit is to be considered
        /// flat or circular.
        ///
        /// A flat orbit (i=0) has it's OmegaU/RAAN set to 0.
        /// A circular orbit has it's argp set to zero (this is the reference point for the phase, nu).
        ///
        /// Of course the zero detect is not exactly zero, so there will be some threshold (usually E-6) where e.g.
        /// an orbit can transition from flat to non-flat. When this happens the argp and phase may have a discontinuity
        /// (athough their sum will remain constant). 
        ///
        /// </summary>
        /// <param name="bodyId"></param>
        /// <param name="centerId"></param>
        /// <param name="usePropCOE">If propagator has internal COE, use that instead of recomputing from R, V</param>
        /// <returns></returns>
        public Orbital.COE COE(int bodyId, int centerId, bool usePropCOE = true)
        {
            Orbital.COE coe = new Orbital.COE();

            if (bodies.propType[bodyId] == GEPhysicsCore.Propagator.GRAVITY)
                usePropCOE = false;

            // FOR NOW, until TODO below is done
            usePropCOE = false;

            if (!usePropCOE) {
                double3 r, v;
                if (refFrame == ReferenceFrame.COROTATING_CR3BP) {
                    (double3 cr, double3 cv) = CR3BP.FrameRotatingToInertial(bodies.r[centerId], bodies.v[centerId], t_physics);
                    (double3 br, double3 bv) = CR3BP.FrameRotatingToInertial(bodies.r[bodyId], bodies.v[bodyId], t_physics);
                    r = br - cr;
                    v = bv - cv;
                } else {
                    r = bodies.r[bodyId] - bodies.r[centerId];
                    v = bodies.v[bodyId] - bodies.v[centerId];
                }
                coe = Orbital.RVtoCOE(r, v, bodies.mu[centerId]);
            } else {
                switch (bodies.propType[bodyId]) {
                    case GEPhysicsCore.Propagator.KEPLER:
                        throw new NotImplementedException("TODO");

                    case GEPhysicsCore.Propagator.KEPLER_COE:
                        coe.InitFromCOEStruct(bodies.coe[bodyId]);
                        break;

                    case GEPhysicsCore.Propagator.GRAVITY:
                        break;

                    case GEPhysicsCore.Propagator.PKEPLER:
                    case GEPhysicsCore.Propagator.SGP4_RAILS:
                        coe.InitFromCOEStruct(bodies.coe[bodyId]);
                        break;

                    case GEPhysicsCore.Propagator.FIXED:
                        Debug.LogErrorFormat("Fixed object does not have a COE id={0} center={1}", bodyId, centerId);
                        break;

                    default:
                        throw new NotImplementedException("Unknown case");
                }
            }
            coe.ScaleLength(geScaler.ScaleLenGEToWorld(1.0));
            coe.ComputeRotation();
            coe.mu = geScaler.ScaleMuGEToWorld(bodies.mu[centerId]);
            return coe;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="tUntilWorld"></param>
        public void EvolveNow(double tUntilWorld)
        {
            if (trajReset) {
                TrajectoryReset();
            }

            gePhysConfig[T_END_PARAM] = geScaler.ScaleTimeWorldToGE(tUntilWorld);
            running = true;
            gePhysicsJob.Evolve();
            running = false;
            if (geTrajectory != null) {
                geTrajectory.EvolveNowTrajectory(tUntilWorld);
            }
            PhysicsLoopComplete();
        }

        private void EvolveNowTrajectory(double tUntilWorld)
        {
            gePhysConfig[T_END_PARAM] = geScaler.ScaleTimeWorldToGE(tUntilWorld) + traj_timeAheadGE;
            running = true;
            gePhysicsJob.Evolve();
            running = false;
            PhysicsLoopComplete();
        }

        /// <summary>
        /// Evolve the gravitational system in-line (may take more that one frame depending on the t_until)
        /// and record the state of the bodies described in bodies at the times in timePoints.
        /// 
        /// The result is placed in a large linear NativeArray indexed as:
        /// timePointNum * numBodies + 
        /// </summary>
        /// <param name="t_until"></param>
        /// <param name="timePoints"></param>
        /// <param name="bodies"></param>
        public void EvolveNowRecordOutput(double tUntilWorld, double[] timePoints, int[] bodies)
        {
            if (trajReset) {
                TrajectoryReset();
            }
            ScaleTimePoints(timePoints);
            gePhysConfig[T_END_PARAM] = geScaler.ScaleTimeWorldToGE(tUntilWorld);
            gePhysicsJob.ModeSet(GEPhysicsCore.ExecuteMode.RECORD);
            gePhysicsJob.recordTimes.Dispose();
            gePhysicsJob.recordTimes = new NativeArray<double>(timePoints, Allocator.Persistent);
            gePhysicsJob.recordBodies.Dispose();
            gePhysicsJob.recordBodies = new NativeArray<int>(bodies, Allocator.Persistent);
            gePhysicsJob.PrepareForRecording(recordLoop: false);
            running = true;
            gePhysicsJob.Evolve();
            // Do we need to do RunStep or can we just evolve to the next time point??
            // Easier to do RunStep since generalizes to Job case better
            running = false;
            PhysicsLoopComplete();
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public List<int> MassiveIndicesGet()
        {
            List<int> indices = new List<int>();
            for (int i = 0; i < gePhysicsJob.lenMassiveBodies.Value; i++) {
                indices.Add(gePhysicsJob.massiveBodies[i]);
            }
            for (int i = 0; i < gePhysicsJob.lenKeplerMassive.Value; i++) {
                indices.Add(gePhysicsJob.keplerMassive[i]);
            }
            for (int i = 0; i < gePhysicsJob.lenFixedBodies.Value; i++) {
                indices.Add(gePhysicsJob.fixedBodies[i]);
            }
            return indices;
        }

        /// <summary>
        /// 
        /// </summary>  
        /// <returns></returns>
        public ref GEPhysicsCore.GEBodies BodiesGetAll()
        {
            return ref bodies;
        }


        /// <summary>
        /// Get a reference to the recorded output in GE units.
        ///
        /// The output is arranged in "body major" sequence. At timePoint[0] entries 0..numBodies will hold
        /// state for each body at time zero. At tp[1] the info is at numBodies..2*numBodies-1 etc.
        ///
        /// In the case of trajectory data the array is written as a circular buffer and the last written entry
        /// can be determined by TrajectoryStatus()
        /// </summary>
        /// <returns></returns>
        public NativeArray<GEBodyState> RecordedOutputGEUnits()
        {
            return gePhysicsJob.recordedState;
        }

        /// <summary>
        /// Get the recorded output for a specific body. 
        /// </summary>
        /// <param name="bodyId"></param>
        /// <param name="worldUnits">If true, return data in world units, otherwise return data in GE units</param>
        /// <returns></returns>
        public GEBodyState[] RecordedOutputForBody(int bodyId, bool worldUnits = true)
        {
            GEBodyState[] data = new GEBodyState[gePhysicsJob.recordTimes.Length];
            double rScale = 1.0;
            double vScale = 1.0;
            double tScale = 1.0;
            if (worldUnits) {
                rScale = geScaler.ScaleLenGEToWorld(1.0);
                vScale = geScaler.ScaleVelocityGEToWorld(1.0);
                tScale = geScaler.ScaleTimeGEToWorld(1.0);
            }
            int index = gePhysicsJob.recordBodies.IndexOf(bodyId);
            if (index < 0) {
                Debug.LogErrorFormat("Body {0} not found in recorded output", bodyId);
                return null;
            }
            int stride = gePhysicsJob.recordBodies.Length;
            for (int i = 0; i < gePhysicsJob.recordTimes.Length; i++) {
                data[i] = new GEBodyState {
                    r = gePhysicsJob.recordedState[i * stride + index].r * rScale,
                    v = gePhysicsJob.recordedState[i * stride + index].v * vScale,
                    t = gePhysicsJob.recordedState[i * stride + index].t * tScale
                };
            }
            return data;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public NativeArray<int> RecordedBodies()
        {
            return gePhysicsJob.recordBodies;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public NativeArray<double> RecordedTimesGEUnits()
        {
            return gePhysicsJob.recordTimes;
        }

        /// <summary>
        /// Schedule the job to run until t_until. Do this in a way that the job can be continued with
        /// subsequent schedule calls. 
        /// </summary>
        /// <param name="tUntilWorld">end time of gravity evolution</param>
        public void Schedule(double tUntilWorld)
        {
            if (trajReset) {
                TrajectoryReset();
            }
            gePhysConfig[T_END_PARAM] = geScaler.ScaleTimeWorldToGE(tUntilWorld);
            running = true;
            jobHandle = gePhysicsJob.Schedule();
            if (geTrajectory != null) {
                geTrajectory.ScheduleTrajectory(tUntilWorld);
            }
        }

        /// <summary>
        /// Run GE evolution in "batch" mode and record the output at specified timePoints for the indicated
        /// bodies.
        /// 
        /// </summary>
        /// <param name="tUntilWorld"></param>
        /// <param name="timePoints"></param>
        /// <param name="bodies"></param>
        public void ScheduleRecordOutput(double tUntilWorld, double[] timePoints, int[] bodies)
        {
            if (trajReset) {
                TrajectoryReset();
            }
            // Scale the time points
            ScaleTimePoints(timePoints);
            gePhysicsJob.ModeSet(GEPhysicsCore.ExecuteMode.RECORD);
            gePhysicsJob.recordTimes.Dispose();
            gePhysicsJob.recordTimes = new NativeArray<double>(timePoints, Allocator.Persistent);
            gePhysicsJob.recordBodies.Dispose();
            gePhysicsJob.recordBodies = new NativeArray<int>(bodies, Allocator.Persistent);
            gePhysicsJob.PrepareForRecording(recordLoop: false);
            gePhysConfig[T_END_PARAM] = geScaler.ScaleTimeWorldToGE(tUntilWorld);
            running = true;
            jobHandle = gePhysicsJob.Schedule();
        }

        private void ScheduleTrajectory(double tUntilWorld)
        {
            gePhysConfig[T_END_PARAM] = geScaler.ScaleTimeWorldToGE(tUntilWorld) + traj_timeAheadGE;
            running = true;
            jobHandle = gePhysicsJob.Schedule();
        }


        private void ScaleTimePoints(double[] t)
        {
            double tScale = geScaler.ScaleTimeWorldToGE(1.0);
            for (int i = 0; i < t.Length; i++)
                t[i] *= tScale;
        }


        /// <summary>
        /// Signal that jobs running must complete and wait for this to happen.
        ///
        /// If a trajectory job is also being used, this will complete also.
        /// </summary>
        public void Complete()
        {
            jobHandle.Complete();
            if (geTrajectory != null) {
                geTrajectory.Complete();
            }
            running = false;
            PhysicsLoopComplete();
        }

        /// <summary>
        /// Poll to see if the jobs that were scheduled are complete. 
        /// </summary>
        /// <returns></returns>
        public bool IsCompleted()
        {
            bool complete = jobHandle.IsCompleted;
            if (geTrajectory != null)
                complete = complete && geTrajectory.IsCompleted();
            return complete;
        }

        private void PhysicsLoopComplete()
        {
            t_physics = gePhysConfig[T_PARAM];
            double rScale = geScaler.ScaleLenGEToWorld(1.0);
            double vScale = geScaler.ScaleVelocityGEToWorld(1.0);
            double tScale = geScaler.ScaleTimeGEToWorld(1.0);
            // Events: Collisions, Manuevers, SGP4 errors, Booster events etc.
            foreach (GEPhysicsCore.PhysEvent pEvent in gePhysicsJob.gepcEvents) {
                // Collisions
                // scale pEvent internals to world space
                GEPhysicsCore.PhysEvent pEventCopy = pEvent; // struct makes a copy
                pEventCopy.r *= rScale;
                pEventCopy.r_secondary *= rScale;
                pEventCopy.v *= vScale;
                pEventCopy.v_secondary *= vScale;
                pEventCopy.t *= tScale;

                if (DEBUG) {
                    Debug.LogFormat("Physics event: type={0} t={1} id={2} status={3} aux={4}",
                                pEvent.type, pEventCopy.t, pEvent.bodyId, pEvent.statusCode, pEvent.auxIndex);
                }

                switch (pEvent.type) {
                    case GEPhysicsCore.EventType.COLLISION:
                        if (pEvent.collisionType == GEPhysicsCore.CollisionType.ABSORB) {
                            // remove the secondary body
                            int remove_id = gePhysicsJob.collisionInfo[pEvent.bodyId_secondary].id;
                            BodyRemove(remove_id);  // will remove collision info also
                            geListener?.BodyRemoved(remove_id, this, pEvent);
                        }
                        break;
                    case GEPhysicsCore.EventType.KEPLER_ERROR:
                        Debug.LogWarningFormat("KEPLER_ERROR {0}", pEvent.bodyId);
                        BodyRemove(pEvent.bodyId);
                        geListener?.BodyRemoved(pEvent.bodyId, this, pEvent);
                        break;

                    case GEPhysicsCore.EventType.MANEUVER_ERROR:
                    case GEPhysicsCore.EventType.MANEUVER:
                        if (pEvent.type == GEPhysicsCore.EventType.MANEUVER_ERROR) {
                            Debug.LogWarningFormat("MANEUVER_ERROR {0}", pEvent.bodyId);
                        }
                        // find the class entry in the parallel array and run callback with the 
                        // pEvent copy that has world scale
                        for (int i = 0; i < pendingManeuvers.Count; i++) {
                            if (pendingManeuvers[i].uniqueId == pEvent.auxIndex) {
                                if (pendingManeuvers[i].doneCallback != null) {
                                    pendingManeuvers[i].doneCallback(pendingManeuvers[i], pEventCopy);
                                }
                                pendingManeuvers.RemoveAt(i);
                                break;
                            }
                        }
                        // find the entry in the master maneuver list and remove
                        for (int i = 0; i < masterManeuverList.Count; i++) {
                            if (masterManeuverList[i].uniqueId == pEvent.auxIndex) {
                                masterManeuverList.RemoveAt(i);
                                break;
                            }
                        }
                        break;

                    case GEPhysicsCore.EventType.SGP4_ERROR:
                        if (bodies.patchIndex[pEvent.bodyId] >= 0) {
                            // add a patch to hold position here for 
                            PatchAppendFixedEndpoint(pEvent.bodyId, pEvent.r, pEvent.v, pEvent.t);
                            gePhysicsJob.IndexListAdd(GEPhysicsCore.Propagator.FIXED, pEvent.bodyId);

                        } else {
                            // remove the body. 
                            BodyRemove(pEvent.bodyId);
                            geListener?.BodyRemoved(pEvent.bodyId, this, pEvent);
                        }
                        break;

                    case GEPhysicsCore.EventType.EXTERNAL_ACCELERATION_REMOVED:
                        // remove the external acceleration from the list
                        extAccels.Remove(pEvent.auxIndex);
                        extAccelPool.Free(pEvent.auxIndex);
                        ExtAccelIndexUpdate();
                        break;
                }
                if (physEventListeners.Count > 0) {
                    foreach (PhysicsEventCallback pecb in physEventListeners) {
                        if (verbose)
                            Debug.LogFormat("Calling {0} for event {1}", pecb.Method.Name, pEventCopy.LogString());
                        pecb(this, pEventCopy);
                    }
                }

            }
            gePhysicsJob.gepcEvents.Clear();
            // need to rebuild maneuver list (NativeList does not have a remove by item)
            gePhysicsJob.maneuvers.Clear();
            foreach (GEManeuverStruct maneuver in masterManeuverList)
                gePhysicsJob.maneuvers.Add(maneuver);

            // TODO: These callbacks could result in repeated traj resets. Find a way to avoid duplication.
            foreach (PhysLoopCompleteCB plc in physLoopCompleteCallbacks) {
                plc.cb(this, plc.arg);
            }
            physLoopCompleteCallbacks.Clear();
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="ge"></param>
        /// <param name="physEvent"></param>
        public delegate void PhysicsEventCallback(GECore ge, GEPhysicsCore.PhysEvent physEvent);

        /// <summary>
        /// 
        /// </summary>
        /// <param name="physEventCB"></param>
        public void PhysicsEventListenerAdd(PhysicsEventCallback physEventCB)
        {
            physEventListeners.Add(physEventCB);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="physEventCB"></param>
        public void PhysicsEventListenerRemove(PhysicsEventCallback physEventCB)
        {
            physEventListeners.Remove(physEventCB);
        }

        /// <summary>
        /// 
        /// </summary>
        public void Dispose()
        {
            GEPhysicsCore.BodiesDispose(ref bodies);
            gePhysicsJob.Dispose();
            if (geTrajectory != null) {
                geTrajectory.Dispose();
                geTrajectory = null;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="info"></param>
        /// <param name="worldScale"></param>
        /// <returns></returns>
        public string DumpAll(string info = "", bool worldScale = false, GSController gsc = null)
        {
            double lenScale = 1.0;
            double velScale = 1.0;
            if (worldScale) {
                lenScale = geScaler.ScaleLenGEToWorld(1.0);
                velScale = geScaler.ScaleVelocityGEToWorld(1.0);
            }
            System.Text.StringBuilder sb = new System.Text.StringBuilder();
            sb.Append(string.Format("*** GE DUMPALL:{0} integrator={1}\n", info, integrator));
            sb.Append(string.Format("  Start Time JD={0} nowJD={1}\n", gePhysConfig[START_TIME_JD], TimeJulianDays()));
            sb.Append(string.Format("  Parms: dt={0:G7} t={1:G7} t_end={2:G7} \n",
                gePhysConfig[DT_PARAM], gePhysConfig[T_PARAM], gePhysConfig[T_END_PARAM]));
            sb.Append(string.Format("World Time = {0:G7}\n", geScaler.ScaleTimeGEToWorld(gePhysConfig[T_PARAM])));

            // Scaling
            sb.Append(geScaler.ToString());

            // Raw internals
            if (worldScale)
                sb.Append("\nGE Data ***WORLD SCALE *** (mass excluded)\n");
            else
                sb.Append("\nGE Internal Data\n");

            sb.Append(string.Format("  Nbody Massive n={0}\n", gePhysicsJob.lenMassiveBodies.Value));
            for (int j = 0; j < gePhysicsJob.lenMassiveBodies.Value; j++) {
                int i = gePhysicsJob.massiveBodies[j];
                sb.Append(string.Format("    {0} r={1} |r| = {2} v={3} |v|={4} mu={5}\n",
                    IdAndName(i, gsc), bodies.r[i] * lenScale, math.length(bodies.r[i]),
                    bodies.v[i] * velScale, math.length(bodies.v[i]),
                    bodies.mu[i]));
            }
            sb.Append(string.Format("  Nbody Massless n={0} \n", gePhysicsJob.lenMasslessBodies.Value));
            for (int j = 0; j < gePhysicsJob.lenMasslessBodies.Value; j++) {
                int i = gePhysicsJob.masslessBodies[j];
                sb.Append(string.Format("    {0} r={1:G7} |r| = {2:G7} v={3:G7} |v|={4} si={5}\n",
                    IdAndName(i, gsc), bodies.r[i] * lenScale, math.length(bodies.r[i]),
                    bodies.v[i] * velScale, math.length(bodies.v[i]), bodies.self_integrating[i]));
            }
            sb.Append(string.Format("  FIXED n={0} :\n", gePhysicsJob.lenFixedBodies.Value));
            for (int j = 0; j < gePhysicsJob.lenFixedBodies.Value; j++) {
                int i = gePhysicsJob.fixedBodies[j];
                sb.Append(string.Format("    {0} r={1:G7} v={2:G7} m={3:G7}\n",
                    IdAndName(i, gsc), bodies.r[i] * lenScale, bodies.v[i] * velScale,
                    bodies.mu[i]));
                if (bodies.patchIndex[i] >= 0)
                    sb.Append(LogPatches(bodies.patchIndex[i]));
            }
            sb.Append(string.Format("  Kepler Massive n={0} :\n", gePhysicsJob.lenKeplerMassive.Value));
            for (int j = 0; j < gePhysicsJob.lenKeplerMassive.Value; j++) {
                int i = gePhysicsJob.keplerMassive[j];
                int p = bodies.propIndex[i];
                sb.Append(string.Format("    {0} pI={1} r={2:G7} v={3:G7} cId={4} m={5:G7} rvt.r0={6} rvt.vo={7} depth={8}\n",
                    IdAndName(i, gsc), p, bodies.r[i] * lenScale, bodies.v[i] * velScale,
                    gePhysicsJob.keplerPropInfo[i].centerId, bodies.mu[i],
                    gePhysicsJob.keplerPropInfo[i].rvt.r0 * lenScale,
                    gePhysicsJob.keplerPropInfo[i].rvt.v0 * velScale,
                    gePhysicsJob.keplerPropInfo[i].keplerDepth));
                if (bodies.patchIndex[i] >= 0)
                    sb.Append(LogPatches(bodies.patchIndex[i]));
            }

            sb.Append(string.Format("  Massless On-Rails (KEPLER) n={0} :\n", gePhysicsJob.lenKeplerMassless.Value));
            for (int j = 0; j < gePhysicsJob.lenKeplerMassless.Value; j++) {
                int i = gePhysicsJob.keplerMassless[j];
                int p = bodies.propIndex[i];
                if (p < 0) Debug.LogErrorFormat("p < 0 for massless body {0}", i);
                sb.Append(string.Format("    {0} pI={1} r={2:G7} |r|={3:#.#####} v={4:G7} cId={5} m={6:G7} rvt.r0={7} rvt.vo={8} pIndex={9} si={10}\n",
                    IdAndName(i, gsc), p, bodies.r[i] * lenScale,
                    math.length(bodies.r[i]) * lenScale,
                    bodies.v[i] * velScale,
                    gePhysicsJob.keplerPropInfo[p].centerId, bodies.mu[i],
                    gePhysicsJob.keplerPropInfo[p].rvt.r0 * lenScale,
                    gePhysicsJob.keplerPropInfo[p].rvt.v0 * velScale,
                    bodies.patchIndex[i], bodies.self_integrating[i]));
                if (bodies.patchIndex[i] >= 0)
                    sb.Append(LogPatches(bodies.patchIndex[i]));
            }
            // KEPLER_COE
            sb.Append(string.Format("  KEPLER_COE On-Rails n={0} :\n", gePhysicsJob.lenKeplerCoeBodies.Value));
            for (int j = 0; j < gePhysicsJob.lenKeplerCoeBodies.Value; j++) {
                int i = gePhysicsJob.keplerCoeBodies[j];
                int p = bodies.propIndex[i];
                sb.Append(string.Format("    {0} pI={1} r={2} v={3} m={4} pIndex={5}\n     propInfo={6}",
                    IdAndName(i, gsc), p, bodies.r[i] * lenScale, bodies.v[i] * velScale,
                    bodies.mu[i],
                    bodies.patchIndex[i],
                    gePhysicsJob.keplerCoePropInfo[p].LogString()));
                if (bodies.patchIndex[i] >= 0)
                    sb.Append(LogPatches(bodies.patchIndex[i]));
            }
            // PKEPLER
            sb.Append(string.Format("  PKEPLER On-Rails n={0} :\n", gePhysicsJob.lenPkeplerBodies.Value));
            for (int j = 0; j < gePhysicsJob.lenPkeplerBodies.Value; j++) {
                int i = gePhysicsJob.pkeplerBodies[j];
                int p = bodies.propIndex[i];
                sb.Append(string.Format("    {0} pI={1} r={2} v={3} cId={4} m={5} t_start={6} coe0={7} pIndex={8}\n     propInfo={9}",
                    IdAndName(i, gsc), p, bodies.r[i] * lenScale, bodies.v[i] * velScale,
                    gePhysicsJob.pKeplerPropInfo[p].centerId, bodies.mu[i],
                    gePhysicsJob.pKeplerPropInfo[p].t_start,
                    gePhysicsJob.pKeplerPropInfo[p].coeKm_init.ToString(),
                    bodies.patchIndex[i],
                    gePhysicsJob.pKeplerPropInfo[p].LogString()));
                if (bodies.patchIndex[i] >= 0)
                    sb.Append(LogPatches(bodies.patchIndex[i]));
            }
            // SGP4
            sb.Append(string.Format("  SGP4 On-Rails n={0} :\n", gePhysicsJob.lenSgp4Bodies.Value));
            for (int j = 0; j < gePhysicsJob.lenSgp4Bodies.Value; j++) {
                int i = gePhysicsJob.sgp4Bodies[j];
                int p = bodies.propIndex[i];
                sb.Append(string.Format("    {0} pI={1} r={2} v={3} cId={4} m={5} pIndex={6} earlyProp={7}\n     propInfo={8}",
                    IdAndName(i, gsc), p, bodies.r[i] * lenScale, bodies.v[i] * velScale,
                    gePhysicsJob.sgp4PropInfo[p].centerId, bodies.mu[i],
                    bodies.patchIndex[i], bodies.earlyPropagation[i],
                    gePhysicsJob.sgp4PropInfo[p].LogString()));
                if (bodies.patchIndex[i] >= 0)
                    sb.Append(LogPatches(bodies.patchIndex[i]));
            }

            // EPHEM
            sb.Append(string.Format("  EPHEM On-Rails n={0} :\n", gePhysicsJob.lenEphemBodies.Value));
            for (int j = 0; j < gePhysicsJob.lenEphemBodies.Value; j++) {
                int i = gePhysicsJob.ephemBodies[j];
                int p = bodies.propIndex[i];
                sb.Append(string.Format("    {0} pI={1} r={2} v={3} cId={4} m={5} pIndex={6}\n",
                    IdAndName(i, gsc), p, bodies.r[i] * lenScale, bodies.v[i] * velScale,
                    gePhysicsJob.ephemPropInfo[p].centerId, bodies.mu[i],
                    bodies.patchIndex[i]));
                if (bodies.patchIndex[i] >= 0)
                    sb.Append(LogPatches(bodies.patchIndex[i]));
            }

            // PLANET SURFACE
            sb.Append(string.Format("  PLANET SURFACE n={0} :\n", gePhysicsJob.lenRotationBodies.Value));
            for (int j = 0; j < gePhysicsJob.lenRotationBodies.Value; j++) {
                int i = gePhysicsJob.rotationBodies[j];
                int p = bodies.propIndex[i];
                sb.Append(string.Format("    {0} pI={1} r={2} |r|={3} v={4} |v|={5} cId={6} m={7} pIndex={8}\n",
                    IdAndName(i, gsc), p, bodies.r[i] * lenScale, math.length(bodies.r[i]),
                    bodies.v[i] * velScale, math.length(bodies.v[i]),
                    gePhysicsJob.rotationPropInfo[p].centerId, bodies.mu[i],
                    bodies.patchIndex[i]));
                if (bodies.patchIndex[i] >= 0)
                    sb.Append(LogPatches(bodies.patchIndex[i]));
            }

            // GEManeuvers
            sb.Append("  Manuevers n=" + gePhysicsJob.maneuvers.Length + " :\n");
            foreach (GEManeuverStruct m in gePhysicsJob.maneuvers) {
                sb.Append(string.Format("   t={0} id={1} mType={2} v={3}\n", m.t, m.id, m.type, m.velocityParam));
            }

            // Trajectories
            if (trajectory) {
                sb.Append("Trajectories:\n");
                foreach (int be in gePhysicsJob.recordBodies) {
                    sb.Append(string.Format("{0} ", be));
                }
                sb.Append("\n");
                sb.Append(string.Format("  RecordTimes.Len={0} RecordState.Len={1}\n",
                    gePhysicsJob.recordTimes.Length, gePhysicsJob.recordedState.Length));
            } else {
                sb.Append("No trajectories\n");
            }
            if (geTrajectory != null) {
                sb.Append("\n*** geTrajectory***\n");
                sb.Append(string.Format(" World time: t_ahead={0:G7}\n", geScaler.ScaleTimeGEToWorld(geTrajectory.traj_timeAheadGE)));
                sb.Append(geTrajectory.DumpAll());
            }

            // Colliders
            sb.Append("\n** Colliders **\n");
            foreach (int c in colliders) {
                sb.Append("    " + IdAndName(gePhysicsJob.collisionInfo[c].id, gsc) + " " + gePhysicsJob.collisionInfo[c].LogString() + "\n");
            }

            // External Accelerations
            sb.Append("\n** External Accelerations **\n");
            foreach (int e in extAccels) {
                sb.Append("    " + IdAndName(gePhysicsJob.extADesc[e].bodyId, gsc) + " " + gePhysicsJob.extADesc[e].LogString() + "\n");
            }
            // Listeners
            sb.Append("\n** Physics Event Listeners **\n");
            foreach (var listener in physEventListeners) {
                sb.Append("    " + listener.Method.Name + "\n");
            }

            sb.Append("\n** Physics Loop Complete Callbacks **\n");
            foreach (var callback in physLoopCompleteCallbacks) {
                sb.Append("    " + callback.cb.Method.Name + "\n");
            }
            return sb.ToString();
        }

        private string IdAndName(int id, GSController gsc)
        {
            if (gsc == null) {
                return string.Format("{0}", id);
            }
            return string.Format("{0} {1}", id, gsc.GSBodyById(id).name);
        }

        /// <summary>
        /// Debug utility for unit testing of internal book-keeping of propagators.
        /// </summary>
        /// <param name="prop"></param>
        /// <returns></returns>
        public int PropArraySize(GEPhysicsCore.Propagator prop)
        {
            switch (prop) {
                case GEPhysicsCore.Propagator.KEPLER:
                    return gePhysicsJob.keplerPropInfo.Length;
                case GEPhysicsCore.Propagator.PKEPLER:
                    return gePhysicsJob.pKeplerPropInfo.Length;
                case GEPhysicsCore.Propagator.SGP4_RAILS:
                    return gePhysicsJob.sgp4PropInfo.Length;
                case GEPhysicsCore.Propagator.PLANET_SURFACE:
                    return gePhysicsJob.rotationPropInfo.Length;
                default:
                    return 0;
            }
        }

        private string LogPatches(int pIndex)
        {
            // find fist one
            int p = pIndex;
            double tFactor = geScaler.ScaleTimeGEToWorld(1.0);
            while (gePhysicsJob.patches[p].prevEntry >= 0)
                p = gePhysicsJob.patches[p].prevEntry;
            System.Text.StringBuilder sb = new System.Text.StringBuilder();
            while (p >= 0) {
                GEPhysicsCore.PatchInfo pInfo = gePhysicsJob.patches[p];
                sb.Append(string.Format("       P> p={0} prop={1} world=[t_s={2} t_e={3}] tGE=[t_s={4} t_e={5}] (prev={6} next={7})\n",
                    p, pInfo.prop, pInfo.timeStartGE * tFactor, pInfo.timeEndGE * tFactor, pInfo.timeStartGE, pInfo.timeEndGE, pInfo.prevEntry, pInfo.nextEntry));
                if (pInfo.prop == GEPhysicsCore.Propagator.KEPLER) {
                    sb.Append(string.Format("             KEPLER cid={0} {1}\n",
                                gePhysicsJob.keplerPropInfo[pInfo.propIndex].centerId,
                                gePhysicsJob.keplerPropInfo[pInfo.propIndex].rvt.ToWorldString(geScaler)));
                } else if (pInfo.prop == GEPhysicsCore.Propagator.PKEPLER) {
                    sb.Append(string.Format("             PKEPLER {0}\n", gePhysicsJob.pKeplerPropInfo[pInfo.propIndex].LogString()));
                } else if (pInfo.prop == GEPhysicsCore.Propagator.KEPLER_COE) {
                    sb.Append(string.Format("             KEPLER_COE {0}\n", gePhysicsJob.keplerCoePropInfo[pInfo.propIndex].LogString()));
                } else if (pInfo.prop == GEPhysicsCore.Propagator.SGP4_RAILS) {
                    sb.Append(string.Format("             SGP4_RAILS {0}\n", gePhysicsJob.sgp4PropInfo[pInfo.propIndex].LogString()));
                } else if (pInfo.prop == GEPhysicsCore.Propagator.FIXED) {
                    sb.Append(string.Format("             FIXED {0} r={1} v={2}\n", pInfo.timeStartGE * tFactor, pInfo.r_fixed, pInfo.v_fixed));
                }
                p = gePhysicsJob.patches[p].nextEntry;
            }
            return sb.ToString();
        }

#if UNITY_EDITOR
        // access for unit testing
        public ref GEPhysicsCore.GEPhysicsJob GEPhysicsJob()
        {
            return ref gePhysicsJob;
        }


#endif
    }
}
