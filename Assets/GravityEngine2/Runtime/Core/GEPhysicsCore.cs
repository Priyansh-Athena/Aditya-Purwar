using Unity.Mathematics;
using Unity.Collections;
using Unity.Burst;
using Unity.Jobs;
using System;
using System.Runtime.CompilerServices;
using System.Diagnostics;

namespace GravityEngine2 {
    /// <summary>
    /// This (plus the @see Integrators) are the guts of the physics evolution. The usual usage after setup is 
    /// via scheduling the GEPhysicsJob. To allow non-job system operation an alternative is to call
    /// the EvolveNow() method within the job struct. 
    /// 
    /// The basic flow is to use RunToTime() to walk through a sequence of time slices (dt) which 
    /// has been pre-determined by the scale and dynamic range of the bodies being modelled. This in 
    /// turn breaks things into individual time steps and uses RunStep() for these.
    /// </summary>     
    [BurstCompile(CompileSynchronously = true)]
    public class GEPhysicsCore {

        /// Propagator types: Determines the algorithm that will be used to evolve the body.
        public enum Propagator { GRAVITY, KEPLER, PKEPLER, SGP4_RAILS, FIXED, EPHEMERIS, PLANET_SURFACE, KEPLER_COE, UNASSIGNED };

        public static bool IsEarlyPropogatable(Propagator p)
        {
            return (p == Propagator.KEPLER_COE) || (p == Propagator.PKEPLER) || (p == Propagator.SGP4_RAILS);
        }

        // Early propagation behaviour: Some on-rails propagators can evolve to times before the time their state was initialized
        // This is a per-body attribute in GEPhysicsCore. This enum designated what to do if evolution before the start time is attempted
        public enum EarlyPropagation { ERROR_INACTIVATE, PROPAGATE, HOLD_VALUE };

        // Propagator types that can be used for binary center of mass ()
        public enum PropagatorPKOnly { GRAVITY, KEPLER };

        public static Propagator PropagatorPKtoFull(PropagatorPKOnly pkp)
        {
            return (Propagator)pkp;
        }

        public static bool IsPatchable(Propagator p)
        {
            return ((p != Propagator.GRAVITY) && (p != Propagator.UNASSIGNED));
        }

        public static bool NeedsCenter(Propagator p)
        {
            return (p == Propagator.KEPLER) || (p == Propagator.PKEPLER) || (p == Propagator.KEPLER_COE) || (p == Propagator.SGP4_RAILS);
        }

        public static bool NeedsEarthCenter(Propagator p)
        {
            return (p == Propagator.PKEPLER) || (p == Propagator.SGP4_RAILS);
        }


        public enum ExecuteMode { NORMAL, RECORD, TRAJECTORY };

        /// <summary>
        /// Core state information for all bodies in GE (all types).
        /// 
        /// There is no ordering by type (that's handled by arrays of indices
        /// indicating where bodies of each type live).
        ///
        /// Bodies may in turn hold references to auxillary data per type
        /// e.g. info that a specific propagator requires
        ///
        /// There is only one instance of this struct in the physics job code and
        /// it crucial to use refs to avoid making copies of this struct.
        /// 
        /// </summary>
        public struct GEBodies {
            public NativeArray<double> mu;     // G*m for the body (scaled)

            // physics state is always present in R, V. COE may be updated by the propagator
            // when appropriate
            public NativeArray<double3> r;
            public NativeArray<double3> v;
            public NativeArray<Orbital.COEStruct> coe;

            // per-type data. Not all of these will be allocated
            public NativeArray<ExternalAccel.EADesc> eaDescrs;

            // Each propagator has unique state information (e.g. R0,V0,T0 or SGP4 data). This is an index into the
            // separate array of this state information. See e.g. @KeplerPropInfo
            public NativeArray<int> propIndex;
            public NativeArray<Propagator> propType;
            public NativeArray<EarlyPropagation> earlyPropagation;

            public NativeArray<int> patchIndex;

            // Flag indicating if a body has self-integrating external accelerations
            public NativeArray<bool> self_integrating;
        }


        public static void BodiesInit(int size, ref GEBodies bodies, Allocator allocator = Allocator.Persistent)
        {
            bodies.mu = new NativeArray<double>(size, allocator);
            bodies.r = new NativeArray<double3>(size, allocator);
            bodies.v = new NativeArray<double3>(size, allocator);
            bodies.coe = new NativeArray<Orbital.COEStruct>(size, allocator);

            bodies.eaDescrs = new NativeArray<ExternalAccel.EADesc>(size, allocator);

            bodies.propIndex = new NativeArray<int>(size, allocator);
            bodies.propType = new NativeArray<Propagator>(size, allocator);
            bodies.earlyPropagation = new NativeArray<EarlyPropagation>(size, allocator);
            // set prop type to UNASSIGNED
            for (int i = 0; i < size; i++) {
                bodies.propType[i] = Propagator.UNASSIGNED;
            }
            // set early propagation to INACTIVE by default
            for (int i = 0; i < size; i++) {
                bodies.earlyPropagation[i] = EarlyPropagation.ERROR_INACTIVATE;
            }
            bodies.patchIndex = new NativeArray<int>(size, allocator);
            bodies.self_integrating = new NativeArray<bool>(size, allocator);
            // initialize self_integrating to false
            for (int i = 0; i < size; i++) {
                bodies.self_integrating[i] = false;
            }
        }

        public static void BodiesInitFrom(ref GEBodies bodies, ref GEBodies from)
        {
            BodiesInit(from.r.Length, ref bodies);
            bodies.mu.CopyFrom(from.mu);
            bodies.r.CopyFrom(from.r);
            bodies.v.CopyFrom(from.v);
            bodies.coe.CopyFrom(from.coe);
            bodies.eaDescrs.CopyFrom(from.eaDescrs);

            bodies.propIndex.CopyFrom(from.propIndex);
            bodies.propType.CopyFrom(from.propType);
            bodies.earlyPropagation.CopyFrom(from.earlyPropagation);
            bodies.patchIndex.CopyFrom(from.patchIndex);
            bodies.self_integrating.CopyFrom(from.self_integrating);

        }

        public static void BodiesDispose(ref GEBodies bodies)
        {
            bodies.mu.Dispose();
            bodies.r.Dispose();
            bodies.v.Dispose();
            bodies.coe.Dispose();
            bodies.eaDescrs.Dispose();

            bodies.propIndex.Dispose();
            bodies.propType.Dispose();
            bodies.earlyPropagation.Dispose();
            bodies.patchIndex.Dispose();
            bodies.self_integrating.Dispose();
        }

        /// <summary>
        /// Patches:
        /// A body may be tagged as patched. This means that it has different propagators for different time
        /// ranges. The canonical example is a sequence of conics (Kepler propagators) that cover e.g the path
        /// out of Earth orbit, navigation through cis-lunar space and then a Kepler propagator when it is
        /// within the moon's sphere of influence.
        /// 
        /// The patch mechanism allows for arbitrary propagator sequences and is designed to allow bodies to be
        /// evolved with different propagators over different time ranges. 
        /// 
        /// Each patch has a patch info that defines the propagator, the time range over which it is applied and the
        /// index of the next and previous patches in the sequence. 
        /// </summary>
        public struct PatchInfo {
            public double timeStartGE;
            public double timeEndGE; // negative if last in sequence
            public int nextEntry;
            public int prevEntry;
            public Propagator prop;    // type of propagator for this patch
            public int propIndex;   // index of the propagator data (e.g. KeplerPropInfo entry)

            public double3 r_fixed;
            public double3 v_fixed;

            public bool InRange(double t)
            {
                if (timeStartGE <= t) {
                    if (timeEndGE < 0)
                        return true;
                    else if (t < timeEndGE)
                        return true;
                }
                return false;
            }

            internal int CenterId(GEPhysicsJob gpj)
            {
                int id = -1;
                switch (prop) {
                    case Propagator.KEPLER:
                        id = gpj.keplerPropInfo[propIndex].centerId;
                        break;
                    case Propagator.PKEPLER:
                        id = gpj.pKeplerPropInfo[propIndex].centerId;
                        break;
                    case Propagator.KEPLER_COE:
                        id = gpj.keplerCoePropInfo[propIndex].centerId;
                        break;
                    case Propagator.SGP4_RAILS:
                        id = gpj.sgp4PropInfo[propIndex].centerId;
                        break;
                    case Propagator.EPHEMERIS:
                        id = gpj.ephemPropInfo[propIndex].centerId;
                        break;
                    case Propagator.PLANET_SURFACE:
                        id = gpj.rotationPropInfo[propIndex].centerId;
                        break;
                    default:
                        break;
                }
                return id;
            }
        }



        /// <summary>
        /// Collision Data structures
        /// GEPC bodies can have a collider added. This is done by adding their id to the colliders index list.
        ///
        /// This list is then checked after each runstep to determine if collisions have occured.
        ///
        /// The handling of a collision MAY be handled entirely in the IJob framework if either ABSORB or BOUNCE is
        /// selected.
        ///
        /// A GEPCEvent is generated by a collision. This is processed by GE after each physics cycle
        /// and can be used to inform listeners about events. 
        ///
        /// FUTURE: Do we want to allow collision checking to be time sliced over multiple steps to spread load??
        /// 
        /// </summary>
        public enum CollisionType { BOUNCE, ABSORB, TRIGGER };

        public struct ColliderInfo {
            public int id;
            // radius of spherical collider to assess if collision has occured
            public double radius;
            // in bounce mode, the amount of momentum conserved (1=hard bounce, if 0 better to use absorb)
            public double bounceFactor;

            public double massInertial; // inertial mass for bounce collisions.
            public CollisionType cType;
            public bool removed;      // removed (absorbed)

            public ColliderInfo(int id, double r, CollisionType cType, double bounce = 1.0)
            {
                this.id = id;
                radius = r;
                this.cType = cType;
                bounceFactor = bounce;
                removed = false;
                massInertial = 1.0;
            }

            public ColliderInfo(ColliderInfo from)
            {
                id = from.id;
                radius = from.radius;
                cType = from.cType;
                bounceFactor = from.bounceFactor;
                removed = from.removed;
                massInertial = 1.0;
            }

            public string LogString()
            {
                return string.Format("id = {0} radius={1} type={2} removed={3}",
                    id, radius, cType, removed);
            }
        }

        /// <summary>
        /// GE Physics Core Event (PhysEvent)
        /// A mechanism for recording events that happen during physics evolution. These are then
        /// examined by GravityEngine at the end of a physics cycle. (This could be a long time if
        /// running a batch sim!).
        /// 
        /// </summary>

        public enum EventType { COLLISION, SGP4_ERROR, BOOSTER, KEPLER_ERROR, PATCH_CHANGE, MANEUVER_ERROR, MANEUVER, EXTERNAL_ACCELERATION_REMOVED, PKEPLER_ERROR, EA_REENTRY };

        public const int BODY_TYPE_UNKNOWN = -2;
        public const int MANEUVER_ON_PATCH = -3;

        public struct PhysEvent {
            public EventType type;
            public int bodyId;               // body id of body affected by the collision
            public int bodyId_secondary;     // body id of the body it collided with
            public double t;              // time of event
            public double3 r;             // world position of cid at time of event
            public double3 r_secondary;   // position of secondary body at time of event
            public double3 v;             // world position of cid at time of event
            public double3 v_secondary;   // position of secondary body at time of event
            public CollisionType collisionType;
            public int auxIndex;          // aux index (collider entry, maneuver list entry etc.)
            public int statusCode;        // status code. SGP4 status/Boostwer status

            public string LogString()
            {
                return string.Format("PhysEvent: type={0} bodyId={1} bodyId_secondary={2} t={3} r={4} v={5} statusCode={6} auxIndex={7}",
                    type, bodyId, bodyId_secondary, t, r, v, statusCode, auxIndex);
            }
        }

        /// <summary>
        /// GEPhysicsJob
        /// Central engine for moving all objects based on NBody physics integrations and/or specific propagators
        /// on a per body basis. It can be run in Job mode or can be called "in-line" from GravityEngine.
        ///
        /// The typical run structure is that a GravitySceneController calls a GravityEngine that then calls this job
        /// to evolve for the "frame time". Stand-alone code may setup a scenario to evaluate and do the whole thing
        /// in one go and then look at the result. The use of record output is typical in this case so that e.g. the path
        /// followed by some of the bodies can be recorded at intermediate points and retrived after the evolution is modelled.
        /// 
        /// 
        /// </summary>

        public struct GEPhysicsJob : IJob {
            // We pass by ref to the integrators, so much is public.

            public GEBodies bodies;
            public NativeArray<double> parms;

            // Index lists for types of bodies
            public NativeArray<int> massiveBodies;
            public NativeArray<int> masslessBodies;
            public NativeArray<int> fixedBodies;
            public NativeArray<int> keplerMassive;
            public NativeArray<int> keplerMassless;
            public NativeArray<int> pkeplerBodies;
            public NativeArray<int> keplerCoeBodies;
            public NativeArray<int> sgp4Bodies;
            public NativeArray<int> ephemBodies;
            public NativeArray<int> rotationBodies;

            // length of index lists. When we need to remove a body (collsions absorb, SGP4 orbit decay etc.)
            // we need to remove it's index from the list, shuffle down and shorten, hence an independant length
            // indication. tedious, since these now need to be passed around, but prefer not to use a List
            // in a (pointless?) optimization.
            public NativeReference<int> lenMassiveBodies;
            public NativeReference<int> lenMasslessBodies;
            public NativeReference<int> lenFixedBodies;
            public NativeReference<int> lenKeplerMassive;
            public NativeReference<int> lenKeplerMassless;
            public NativeReference<int> lenPkeplerBodies;
            public NativeReference<int> lenKeplerCoeBodies;
            public NativeReference<int> lenSgp4Bodies;
            public NativeReference<int> lenEphemBodies;
            public NativeReference<int> lenRotationBodies;

            // index list of bodies that are patched
            public NativeReference<int> lenPatchedBodies;

            // per-type and per-integrator data structures. 
            // Not all types will be allocated
            public Integrators.LeapfrogData leapfrogData;
            public Integrators.RK4Data rk4Data;

            // Info for progagators
            public NativeArray<KeplerPropagator.PropInfo> keplerPropInfo;
            public NativeArray<PKeplerPropagator.PropInfo> pKeplerPropInfo;
            public NativeArray<KeplerCOEPropagator.PropInfo> keplerCoePropInfo;
            public NativeArray<SGP4Propagator.PropInfo> sgp4PropInfo;

            public NativeArray<RotationPropagator.PropInfo> rotationPropInfo;
            [ReadOnly]
            public NativeArray<EphemerisPropagator.PropInfo> ephemPropInfo;

            // index list of bodies that are patched
            [ReadOnly]
            public NativeArray<int> patchedBodies;
            [ReadOnly]
            public NativeArray<PatchInfo> patches;

            public Integrators.Type integratorType;

            public GECore.ReferenceFrame refFrame;

            // Maneuvers
            public NativeList<GEManeuverStruct> maneuvers;

            // External Forces
            public NativeArray<ExternalAccel.EADesc> extADesc;
            public NativeArray<int> extAccelIndices;

            public NativeReference<int> lenExtAccelIndices;

            // flat array with data for ANY/ALL external accel. Refs to range in EADesc.
            public NativeArray<double3> extAData;

            // Collisions
            [ReadOnly]
            public NativeArray<int> colliders;
            [ReadOnly]
            public NativeArray<ColliderInfo> collisionInfo;

            // in a given Evolve() cycle ensure that each collision event is reported only once
            // by activly screening new events.
            private bool uniqueCollisionEvents;
            public NativeList<PhysEvent> gepcEvents;

            // Record Output
            public NativeArray<int> recordBodies;
            public NativeArray<double> recordTimes;
            // Array of data in [ numBodies x numTime ] i.e. [ b0@t0, b1@t0, ... b0@t1, b1 @t1...]
            public NativeArray<GEBodyState> recordedState;

            public bool recordLoop;

            public ExecuteMode execMode;

            // Ephemeris Data (in GE units)
            [ReadOnly]
            public NativeArray<GEBodyState> ephemerisData;

            // scale units for bodies.r[] to km (Pkepler and SGP4 prop requires this)
            private double scaleLtoKm;
            private double scaleTtoSec;
            private double scaleKmsecToV;

            private double scaleAccelSItoGE;

            public void Init(GECore.GEConfig geConfig, Integrators.Type itype)
            {
                this.integratorType = itype;
                refFrame = geConfig.refFrame;
                GEPhysicsCore.BodiesInit(geConfig.MAX_BODIES, ref bodies);
                parms = new NativeArray<double>(GECore.NUM_PARAMS, Allocator.Persistent);

                switch (itype) {
                    case Integrators.Type.LEAPFROG:
                        leapfrogData.Init(geConfig.MAX_BODIES);
                        rk4Data.Init(1);   // must init everything in a job
                        break;

                    case Integrators.Type.RK4:
                        rk4Data.Init(geConfig.MAX_BODIES);
                        leapfrogData.Init(1);
                        break;

                    default:
                        return;
                }
                uniqueCollisionEvents = geConfig.uniqueCollisionEvents;

                // TODO: (optimize) could be less greedy
                keplerPropInfo = new NativeArray<KeplerPropagator.PropInfo>(geConfig.MAX_BODIES, Allocator.Persistent);
                pKeplerPropInfo = new NativeArray<PKeplerPropagator.PropInfo>(geConfig.MAX_BODIES, Allocator.Persistent);
                keplerCoePropInfo = new NativeArray<KeplerCOEPropagator.PropInfo>(geConfig.MAX_BODIES, Allocator.Persistent);
                sgp4PropInfo = new NativeArray<SGP4Propagator.PropInfo>(geConfig.MAX_BODIES, Allocator.Persistent);
                ephemPropInfo = new NativeArray<EphemerisPropagator.PropInfo>(geConfig.MAX_BODIES, Allocator.Persistent);
                rotationPropInfo = new NativeArray<RotationPropagator.PropInfo>(geConfig.MAX_BODIES, Allocator.Persistent);

                maneuvers = new NativeList<GEManeuverStruct>(geConfig.MAX_MANEUVERS, Allocator.Persistent);

                patches = new NativeArray<PatchInfo>(geConfig.MAX_BODIES, Allocator.Persistent);

                // extAccel
                extADesc = new NativeArray<ExternalAccel.EADesc>(geConfig.MAX_EXTACCEL, Allocator.Persistent);
                extAccelIndices = new NativeArray<int>(0, Allocator.Persistent);
                extAData = new NativeArray<double3>(0, Allocator.Persistent);
                lenExtAccelIndices = new NativeReference<int>(0, Allocator.Persistent);

                // colliders
                colliders = new NativeArray<int>(0, Allocator.Persistent);
                collisionInfo = new NativeArray<ColliderInfo>(geConfig.MAX_BODIES, Allocator.Persistent);
                gepcEvents = new NativeList<PhysEvent>(5, Allocator.Persistent);    // expect a small number of collisions, will auto grow

                // every array needs to be inited, so do a dummy init (and dispose of these placeholders if reqd later)
                recordTimes = new NativeArray<double>(0, Allocator.Persistent);
                recordBodies = new NativeArray<int>(0, Allocator.Persistent);
                recordedState = new NativeArray<GEBodyState>(0, Allocator.Persistent);

                // index list of bodies are allocated to max size (will only be partially used)
                massiveBodies = new NativeArray<int>(geConfig.MAX_BODIES, Allocator.Persistent);
                masslessBodies = new NativeArray<int>(geConfig.MAX_BODIES, Allocator.Persistent);
                fixedBodies = new NativeArray<int>(geConfig.MAX_BODIES, Allocator.Persistent);
                keplerMassive = new NativeArray<int>(geConfig.MAX_BODIES, Allocator.Persistent);
                keplerMassless = new NativeArray<int>(geConfig.MAX_BODIES, Allocator.Persistent);
                pkeplerBodies = new NativeArray<int>(geConfig.MAX_BODIES, Allocator.Persistent);
                keplerCoeBodies = new NativeArray<int>(geConfig.MAX_BODIES, Allocator.Persistent);
                sgp4Bodies = new NativeArray<int>(geConfig.MAX_BODIES, Allocator.Persistent);
                ephemBodies = new NativeArray<int>(geConfig.MAX_BODIES, Allocator.Persistent);
                patchedBodies = new NativeArray<int>(geConfig.MAX_BODIES, Allocator.Persistent);
                rotationBodies = new NativeArray<int>(geConfig.MAX_BODIES, Allocator.Persistent);

                lenMassiveBodies = new NativeReference<int>(0, Allocator.Persistent);
                lenMasslessBodies = new NativeReference<int>(0, Allocator.Persistent);
                lenFixedBodies = new NativeReference<int>(0, Allocator.Persistent);
                lenKeplerMassive = new NativeReference<int>(0, Allocator.Persistent);
                lenKeplerMassless = new NativeReference<int>(0, Allocator.Persistent);
                lenPkeplerBodies = new NativeReference<int>(0, Allocator.Persistent);
                lenKeplerCoeBodies = new NativeReference<int>(0, Allocator.Persistent);
                lenSgp4Bodies = new NativeReference<int>(0, Allocator.Persistent);
                lenEphemBodies = new NativeReference<int>(0, Allocator.Persistent);
                lenPatchedBodies = new NativeReference<int>(0, Allocator.Persistent);
                lenRotationBodies = new NativeReference<int>(0, Allocator.Persistent);

                ephemerisData = new NativeArray<GEBodyState>(0, Allocator.Persistent);

                scaleLtoKm = geConfig.scaleLtoKm;
                scaleTtoSec = geConfig.scaleTtoSec;
                scaleKmsecToV = geConfig.scaleKmsecToV;
                // m/s^2 to km/sec^2 to GE L/T^2 => m/s * km/m * L/km * s/T * s/T
                // L/km is 1/scaleLtoKm
                // s/T is scaleTtoSec
                scaleAccelSItoGE = 1E-3 * scaleTtoSec * scaleTtoSec / scaleLtoKm;

            }

            public double ScaleLToKm()
            {
                return scaleLtoKm;
            }

            public double ScaleLtoSI()
            {
                return scaleLtoKm * 1E3;
            }

            public double ScaleTtoSec()
            {
                return scaleTtoSec;
            }

            public double ScaleAccelSItoGE()
            {
                return scaleAccelSItoGE;
            }

            public void InitFrom(GEPhysicsJob gpj)
            {
                integratorType = gpj.integratorType;
                refFrame = gpj.refFrame;
                GEPhysicsCore.BodiesInitFrom(ref bodies, ref gpj.bodies);
                leapfrogData.InitFrom(gpj.leapfrogData);
                rk4Data.InitFrom(gpj.rk4Data);

                parms = new NativeArray<double>(GECore.NUM_PARAMS, Allocator.Persistent);
                parms.CopyFrom(gpj.parms);

                uniqueCollisionEvents = gpj.uniqueCollisionEvents;

                // index lists
                massiveBodies = new NativeArray<int>(gpj.massiveBodies.Length, Allocator.Persistent);
                massiveBodies.CopyFrom(gpj.massiveBodies);
                masslessBodies = new NativeArray<int>(gpj.masslessBodies.Length, Allocator.Persistent);
                masslessBodies.CopyFrom(gpj.masslessBodies);
                fixedBodies = new NativeArray<int>(gpj.fixedBodies.Length, Allocator.Persistent);
                fixedBodies.CopyFrom(gpj.fixedBodies);
                keplerMassive = new NativeArray<int>(gpj.keplerMassive.Length, Allocator.Persistent);
                keplerMassive.CopyFrom(gpj.keplerMassive);
                keplerMassless = new NativeArray<int>(gpj.keplerMassless.Length, Allocator.Persistent);
                keplerMassless.CopyFrom(gpj.keplerMassless);
                pkeplerBodies = new NativeArray<int>(gpj.pkeplerBodies.Length, Allocator.Persistent);
                pkeplerBodies.CopyFrom(gpj.pkeplerBodies);
                keplerCoeBodies = new NativeArray<int>(gpj.keplerCoeBodies.Length, Allocator.Persistent);
                keplerCoeBodies.CopyFrom(gpj.keplerCoeBodies);
                sgp4Bodies = new NativeArray<int>(gpj.sgp4Bodies.Length, Allocator.Persistent);
                sgp4Bodies.CopyFrom(gpj.sgp4Bodies);
                ephemBodies = new NativeArray<int>(gpj.ephemBodies.Length, Allocator.Persistent);
                ephemBodies.CopyFrom(gpj.ephemBodies);
                patchedBodies = new NativeArray<int>(gpj.patchedBodies.Length, Allocator.Persistent);
                patchedBodies.CopyFrom(gpj.patchedBodies);
                rotationBodies = new NativeArray<int>(gpj.rotationBodies.Length, Allocator.Persistent);
                rotationBodies.CopyFrom(gpj.rotationBodies);

                lenMassiveBodies = new NativeReference<int>(gpj.lenMassiveBodies.Value, Allocator.Persistent);
                lenMasslessBodies = new NativeReference<int>(gpj.lenMasslessBodies.Value, Allocator.Persistent);
                lenFixedBodies = new NativeReference<int>(gpj.lenFixedBodies.Value, Allocator.Persistent);
                lenKeplerMassive = new NativeReference<int>(gpj.lenKeplerMassive.Value, Allocator.Persistent);
                lenKeplerMassless = new NativeReference<int>(gpj.lenKeplerMassless.Value, Allocator.Persistent);
                lenPkeplerBodies = new NativeReference<int>(gpj.lenPkeplerBodies.Value, Allocator.Persistent);
                lenKeplerCoeBodies = new NativeReference<int>(gpj.lenKeplerCoeBodies.Value, Allocator.Persistent);
                lenSgp4Bodies = new NativeReference<int>(gpj.lenSgp4Bodies.Value, Allocator.Persistent);
                lenEphemBodies = new NativeReference<int>(gpj.lenEphemBodies.Value, Allocator.Persistent);
                lenPatchedBodies = new NativeReference<int>(gpj.lenPatchedBodies.Value, Allocator.Persistent);
                lenRotationBodies = new NativeReference<int>(gpj.lenRotationBodies.Value, Allocator.Persistent);

                patches = new NativeArray<PatchInfo>(gpj.patches.Length, Allocator.Persistent);
                patches.CopyFrom(gpj.patches);

                // prop info
                keplerPropInfo = new NativeArray<KeplerPropagator.PropInfo>(gpj.keplerPropInfo.Length, Allocator.Persistent);
                keplerPropInfo.CopyFrom(gpj.keplerPropInfo);

                pKeplerPropInfo = new NativeArray<PKeplerPropagator.PropInfo>(gpj.pKeplerPropInfo.Length, Allocator.Persistent);
                pKeplerPropInfo.CopyFrom(gpj.pKeplerPropInfo);

                keplerCoePropInfo = new NativeArray<KeplerCOEPropagator.PropInfo>(gpj.keplerCoePropInfo.Length, Allocator.Persistent);
                keplerCoePropInfo.CopyFrom(gpj.keplerCoePropInfo);

                sgp4PropInfo = new NativeArray<SGP4Propagator.PropInfo>(gpj.sgp4PropInfo.Length, Allocator.Persistent);
                sgp4PropInfo.CopyFrom(gpj.sgp4PropInfo);

                ephemPropInfo = new NativeArray<EphemerisPropagator.PropInfo>(gpj.ephemPropInfo.Length, Allocator.Persistent);
                ephemPropInfo.CopyFrom(gpj.ephemPropInfo);

                rotationPropInfo = new NativeArray<RotationPropagator.PropInfo>(gpj.rotationPropInfo.Length, Allocator.Persistent);
                rotationPropInfo.CopyFrom(gpj.rotationPropInfo);

                parms = new NativeArray<double>(gpj.parms.Length, Allocator.Persistent);
                parms.CopyFrom(gpj.parms);

                maneuvers = new NativeList<GEManeuverStruct>(gpj.maneuvers.Length, Allocator.Persistent);
                maneuvers.CopyFrom(gpj.maneuvers);

                extADesc = new NativeArray<ExternalAccel.EADesc>(gpj.extADesc.Length, Allocator.Persistent);
                extADesc.CopyFrom(gpj.extADesc);
                extAccelIndices = new NativeArray<int>(gpj.extAccelIndices.Length, Allocator.Persistent);
                extAccelIndices.CopyFrom(gpj.extAccelIndices);
                extAData = new NativeArray<double3>(gpj.extAData.Length, Allocator.Persistent);
                extAData.CopyFrom(gpj.extAData);
                lenExtAccelIndices = new NativeReference<int>(gpj.lenExtAccelIndices.Value, Allocator.Persistent);

                colliders = new NativeArray<int>(0, Allocator.Persistent);
                collisionInfo = new NativeArray<ColliderInfo>(0, Allocator.Persistent);
                gepcEvents = new NativeList<PhysEvent>(5, Allocator.Persistent);    // expect a small number of 

                // copy will not duplicate recorded output
                recordTimes = new NativeArray<double>(1, Allocator.Persistent);
                recordBodies = new NativeArray<int>(1, Allocator.Persistent);
                recordedState = new NativeArray<GEBodyState>(1, Allocator.Persistent);

                // For now copy ephem data (could likely refer back to original for traj only)
                ephemerisData = new NativeArray<GEBodyState>(gpj.ephemerisData.Length, Allocator.Persistent);
                ephemerisData.CopyFrom(gpj.ephemerisData);

                scaleLtoKm = gpj.scaleLtoKm;
                scaleTtoSec = gpj.scaleTtoSec;
                scaleKmsecToV = gpj.scaleKmsecToV;
                scaleAccelSItoGE = gpj.scaleAccelSItoGE;

                execMode = gpj.execMode;
            }

            public void GrowBodies(int growBy)
            {
                NativeArray<double> mu_old = bodies.mu;
                NativeArray<double3> r_old = bodies.r;
                NativeArray<double3> v_old = bodies.v;
                NativeArray<Orbital.COEStruct> coe_old = bodies.coe;
                NativeArray<ExternalAccel.EADesc> eaDescr_old = bodies.eaDescrs;
                NativeArray<int> propIndex_old = bodies.propIndex;
                NativeArray<Propagator> propType_old = bodies.propType;
                NativeArray<EarlyPropagation> earlyPropagation_old = bodies.earlyPropagation;
                NativeArray<int> isPatched_old = bodies.patchIndex;
                NativeArray<bool> self_integrating_old = bodies.self_integrating;

                NativeArray<int> massiveBodies_old = massiveBodies;
                NativeArray<int> masslessBodies_old = masslessBodies;
                NativeArray<int> fixedBodies_old = fixedBodies;
                NativeArray<int> keplerMassive_old = keplerMassive;
                NativeArray<int> keplerMassless_old = keplerMassless;
                NativeArray<int> pkeplerBodies_old = pkeplerBodies;
                NativeArray<int> keplerCoeBodies_old = keplerCoeBodies;
                NativeArray<int> sgp4Bodies_old = sgp4Bodies;
                NativeArray<int> ephemBodies_old = ephemBodies;
                NativeArray<int> patchedBodies_old = patchedBodies;
                NativeArray<int> rotationBodies_old = rotationBodies;

                int size = bodies.r.Length + growBy;
                Allocator allocator = Allocator.Persistent;
                bodies.mu = new NativeArray<double>(size, allocator);
                bodies.r = new NativeArray<double3>(size, allocator);
                bodies.v = new NativeArray<double3>(size, allocator);
                bodies.coe = new NativeArray<Orbital.COEStruct>(size, allocator);
                bodies.eaDescrs = new NativeArray<ExternalAccel.EADesc>(size, allocator);
                bodies.propIndex = new NativeArray<int>(size, allocator);
                bodies.propType = new NativeArray<Propagator>(size, allocator);
                bodies.earlyPropagation = new NativeArray<EarlyPropagation>(size, allocator);
                // set prop type to UNASSIGNED
                for (int i = 0; i < size; i++) {
                    bodies.propType[i] = Propagator.UNASSIGNED;
                }
                // set early propagation to INACTIVE by default
                for (int i = 0; i < size; i++) {
                    bodies.earlyPropagation[i] = EarlyPropagation.ERROR_INACTIVATE;
                }
                bodies.patchIndex = new NativeArray<int>(size, allocator);
                bodies.self_integrating = new NativeArray<bool>(size, allocator);
                // initialize self_integrating to false
                for (int i = 0; i < size; i++) {
                    bodies.self_integrating[i] = false;
                }

                massiveBodies = new NativeArray<int>(size, allocator);
                masslessBodies = new NativeArray<int>(size, allocator);
                fixedBodies = new NativeArray<int>(size, allocator);
                keplerMassive = new NativeArray<int>(size, allocator);
                keplerMassless = new NativeArray<int>(size, allocator);
                pkeplerBodies = new NativeArray<int>(size, allocator);
                keplerCoeBodies = new NativeArray<int>(size, allocator);
                sgp4Bodies = new NativeArray<int>(size, allocator);
                ephemBodies = new NativeArray<int>(size, allocator);
                patchedBodies = new NativeArray<int>(size, allocator);
                rotationBodies = new NativeArray<int>(size, allocator);

                NativeArray<int>.Copy(massiveBodies_old, massiveBodies, massiveBodies_old.Length);
                NativeArray<int>.Copy(masslessBodies_old, masslessBodies, masslessBodies_old.Length);
                NativeArray<int>.Copy(fixedBodies_old, fixedBodies, fixedBodies_old.Length);
                NativeArray<int>.Copy(keplerMassive_old, keplerMassive, keplerMassive_old.Length);
                NativeArray<int>.Copy(keplerMassless_old, keplerMassless, keplerMassless_old.Length);
                NativeArray<int>.Copy(pkeplerBodies_old, pkeplerBodies, pkeplerBodies_old.Length);
                NativeArray<int>.Copy(keplerCoeBodies_old, keplerCoeBodies, keplerCoeBodies_old.Length);
                NativeArray<int>.Copy(sgp4Bodies_old, sgp4Bodies, sgp4Bodies_old.Length);
                NativeArray<int>.Copy(ephemBodies_old, ephemBodies, ephemBodies_old.Length);
                NativeArray<int>.Copy(patchedBodies_old, patchedBodies, patchedBodies_old.Length);
                NativeArray<int>.Copy(rotationBodies_old, rotationBodies, rotationBodies_old.Length);

                NativeArray<double>.Copy(mu_old, bodies.mu, mu_old.Length);
                NativeArray<double3>.Copy(r_old, bodies.r, r_old.Length);
                NativeArray<double3>.Copy(v_old, bodies.v, v_old.Length);
                NativeArray<Orbital.COEStruct>.Copy(coe_old, bodies.coe, coe_old.Length);
                NativeArray<ExternalAccel.EADesc>.Copy(eaDescr_old, bodies.eaDescrs, eaDescr_old.Length);
                NativeArray<int>.Copy(propIndex_old, bodies.propIndex, propIndex_old.Length);
                NativeArray<Propagator>.Copy(propType_old, bodies.propType, propType_old.Length);
                NativeArray<EarlyPropagation>.Copy(earlyPropagation_old, bodies.earlyPropagation, earlyPropagation_old.Length);
                NativeArray<int>.Copy(isPatched_old, bodies.patchIndex, isPatched_old.Length);
                NativeArray<bool>.Copy(self_integrating_old, bodies.self_integrating, self_integrating_old.Length);

                // dispose old arrays
                mu_old.Dispose();
                r_old.Dispose();
                v_old.Dispose();
                coe_old.Dispose();
                eaDescr_old.Dispose();
                propIndex_old.Dispose();
                propType_old.Dispose();
                earlyPropagation_old.Dispose();
                isPatched_old.Dispose();
                self_integrating_old.Dispose();

                massiveBodies_old.Dispose();
                masslessBodies_old.Dispose();
                fixedBodies_old.Dispose();
                keplerMassive_old.Dispose();
                keplerMassless_old.Dispose();
                pkeplerBodies_old.Dispose();
                keplerCoeBodies_old.Dispose();
                sgp4Bodies_old.Dispose();
                patchedBodies_old.Dispose();
                rotationBodies_old.Dispose();
                switch (integratorType) {
                    case Integrators.Type.LEAPFROG:
                        leapfrogData.GrowBy(growBy);
                        break;

                    case Integrators.Type.RK4:
                        rk4Data.GrowBy(growBy);
                        break;
                }
            }

            /// <summary>
            /// Add a body to the physics core setting the essential state information.
            ///
            /// The caller is expected to have a reference to the bodies array and to set
            /// auxilliary fields (e.g. propIndex, isPatched) after this Add call return the
            /// index.
            ///
            /// Only the basic body types are determined here. Kepler massive depth is
            /// set by the caller after adding
            /// 
            /// </summary>
            /// <param name="bodies"></param>
            /// <param name="id"></param>
            /// <param name="r"></param>
            /// <param name="v"></param>
            /// <param name="mu"></param>
            /// <param name="prop"></param>
            /// <param name="typeNum"></param>
            /// <param name="shuffled"></param>
            /// <returns></returns>
            public void BodyAdd(int id,
                                double3 r,
                                double3 v,
                                double mu,
                                Propagator prop,
                                int propIndex,
                                int patchIndex,
                                EarlyPropagation earlyPropagation = EarlyPropagation.ERROR_INACTIVATE)
            {
                bodies.r[id] = r;
                bodies.v[id] = v;
                bodies.mu[id] = mu;
                bodies.propType[id] = prop;
                bodies.propIndex[id] = propIndex;
                bodies.patchIndex[id] = patchIndex;
                bodies.earlyPropagation[id] = earlyPropagation;
                IndexListAdd(prop, id);
            }


            /// <summary>
            /// Keep all the indices in an index list together in memory by keeping a NativeArray copy.
            /// 
            /// </summary>
            /// <param name="bType"></param>
            /// <param name="indices"></param>
            public void IndexListAdd(Propagator prop, int index)
            {
                switch (prop) {
                    case Propagator.GRAVITY:
                        if (bodies.mu[index] > 0) {
                            massiveBodies[lenMassiveBodies.Value] = index;
                            lenMassiveBodies.Value += 1;
                        } else {
                            masslessBodies[lenMasslessBodies.Value] = index;
                            lenMasslessBodies.Value += 1;
                        }
                        break;

                    case Propagator.FIXED:
                        fixedBodies[lenFixedBodies.Value] = index;
                        lenFixedBodies.Value += 1;
                        break;

                    // GE will only call with massive1 BUT will have a list of all Kepler massive sorted
                    // by ascending Kepler depth. 
                    case Propagator.KEPLER:
                        if (bodies.mu[index] > 0) {
                            keplerMassive[lenKeplerMassive.Value] = index;
                            lenKeplerMassive.Value += 1;
                        } else {
                            keplerMassless[lenKeplerMassless.Value] = index;
                            lenKeplerMassless.Value += 1;
                        }
                        break;

                    case Propagator.KEPLER_COE:
                        keplerCoeBodies[lenKeplerCoeBodies.Value] = index;
                        lenKeplerCoeBodies.Value += 1;
                        break;

                    case Propagator.PKEPLER:
                        pkeplerBodies[lenPkeplerBodies.Value] = index;
                        lenPkeplerBodies.Value += 1;
                        break;

                    case Propagator.SGP4_RAILS:
                        sgp4Bodies[lenSgp4Bodies.Value] = index;
                        lenSgp4Bodies.Value += 1;
                        break;

                    case Propagator.EPHEMERIS:
                        ephemBodies[lenEphemBodies.Value] = index;
                        lenEphemBodies.Value += 1;
                        break;

                    case Propagator.PLANET_SURFACE:
                        rotationBodies[lenRotationBodies.Value] = index;
                        lenRotationBodies.Value += 1;
                        break;

                    default:
                        throw new System.NotImplementedException("no case for propagator " + prop);
                }
            }

            public void IndexListRemove(Propagator prop, int id)
            {
                switch (prop) {
                    case Propagator.GRAVITY:
                        if (bodies.mu[id] > 0) {
                            RemoveIndexByValue(ref massiveBodies, lenMassiveBodies.Value, id);
                            lenMassiveBodies.Value -= 1;
                        } else {
                            RemoveIndexByValue(ref masslessBodies, lenMasslessBodies.Value, id);
                            lenMasslessBodies.Value -= 1;
                        }
                        break;

                    case Propagator.FIXED:
                        RemoveIndexByValue(ref fixedBodies, lenFixedBodies.Value, id);
                        lenFixedBodies.Value -= 1;
                        break;

                    case Propagator.KEPLER:
                        if (bodies.mu[id] > 0) {
                            RemoveIndexByValue(ref keplerMassive, lenKeplerMassive.Value, id);
                            lenKeplerMassive.Value -= 1;
                        } else {
                            RemoveIndexByValue(ref keplerMassless, lenKeplerMassless.Value, id);
                            lenKeplerMassless.Value -= 1;
                        }
                        break;

                    case Propagator.KEPLER_COE:
                        RemoveIndexByValue(ref keplerCoeBodies, lenKeplerCoeBodies.Value, id);
                        lenKeplerCoeBodies.Value -= 1;
                        break;

                    case Propagator.PKEPLER:
                        RemoveIndexByValue(ref pkeplerBodies, lenPkeplerBodies.Value, id);
                        lenPkeplerBodies.Value -= 1;
                        break;

                    case Propagator.SGP4_RAILS:
                        RemoveIndexByValue(ref sgp4Bodies, lenSgp4Bodies.Value, id);
                        lenSgp4Bodies.Value -= 1;
                        break;

                    case Propagator.EPHEMERIS:
                        RemoveIndexByValue(ref ephemBodies, lenEphemBodies.Value, id);
                        lenEphemBodies.Value -= 1;
                        break;

                    case Propagator.PLANET_SURFACE:
                        RemoveIndexByValue(ref rotationBodies, lenRotationBodies.Value, id);
                        lenRotationBodies.Value -= 1;
                        break;

                    default:
                        throw new System.NotImplementedException("no case for propagator " + prop);
                }
            }

            public void PatchListRemove(int id)
            {
                RemoveIndexByValue(ref patchedBodies, lenPatchedBodies.Value, id);
                lenPatchedBodies.Value -= 1;
            }


            /// <summary>
            /// Maneuvers and patch changes can change the propagator used on the fly.
            /// This function switches the body type in the index list to match the new
            /// propagator.
            /// 
            /// It does not change the base type that the body was initialized with.
            /// </summary>
            /// <param name="patchIndex"></param>
            /// <param name="bodyIndex"></param>
            private void IndexListSwitch(Propagator fromProp, Propagator toProp, int bodyIndex)
            {
                IndexListRemove(fromProp, bodyIndex);
                IndexListAdd(toProp, bodyIndex);
            }

            private void RemoveIndexByValue(ref NativeArray<int> array, int len, int value)
            {
                int index = -1;
                for (int i = 0; i < len; i++) {
                    if (array[i] == value) {
                        index = i;
                        break;
                    }
                }
                if (index >= 0) {
                    array[index] = array[len - 1];
                    array[len - 1] = -1;
                }
            }


            public void PatchAdd(int index)
            {
                patchedBodies[lenPatchedBodies.Value] = index;
                lenPatchedBodies.Value += 1;
            }

            /// <summary>
            /// Given a time and a patch index, return the index of the patch that is active at that time.
            /// </summary>
            /// <param name="t"></param>
            /// <param name="p"></param>
            /// <param name="patches"></param>
            /// <returns></returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int PatchForTime(double t, int p)
            {
                if (math.abs(t - patches[p].timeStartGE) > 1E-9) {
                    if (t < patches[p].timeStartGE) {
                        // need an earlier patch
                        while (!patches[p].InRange(t)) {
                            p = patches[p].prevEntry;
                            // TODO: Error handling
                            if (p < 0)
                                break;
                        }
                    } else if ((patches[p].timeEndGE >= 0) && (t >= patches[p].timeEndGE)) {
                        // need a later patch
                        p = patches[p].nextEntry;
                        while (!patches[p].InRange(t)) {
                            p = patches[p].nextEntry;
                            if (p < 0)
                                break;
                        }

                    }
                }
                return p;
            }

            /// <summary>
            /// Given a body index, return the index of the first patch in the sequence.
            /// </summary>
            /// <param name="i"></param>
            /// <param name="patches"></param>
            /// <returns></returns>
            public int PatchFirst(int i)
            {
                if (i < 0) {
                    return -1;
                }
                if (patches[i].prop == Propagator.UNASSIGNED) {
                    return -1;
                }
                int first = i;
                while (patches[first].prevEntry >= 0) {
                    first = patches[first].prevEntry;
                }
                return first;
            }

            public int PatchLast(int i)
            {
                if (i < 0) {
                    return -1;
                }
                if (patches[i].prop == Propagator.UNASSIGNED) {
                    return -1;
                }
                int last = i;
                while (patches[last].nextEntry >= 0) {
                    last = patches[last].nextEntry;
                }
                return last;
            }

            public void PreCalcForAdd(int index)
            {

            }

            public void IntegratorSet(Integrators.Type t)
            {
                integratorType = t;
            }

            /// <summary>
            /// Create the destination array for recording state information. 
            /// </summary>
            public void PrepareForRecording(bool recordLoop)
            {
                this.recordLoop = recordLoop;
                if (recordedState.IsCreated) {
                    recordedState.Dispose();
                }
                if (recordLoop) {
                    recordedState = new NativeArray<GEBodyState>((int)(parms[GECore.TRAJ_NUMSTEPS_PARM] * recordBodies.Length),
                                            Allocator.Persistent);
                } else {
                    recordedState = new NativeArray<GEBodyState>(recordTimes.Length * recordBodies.Length, Allocator.Persistent);
                }
            }

            public void ModeSet(ExecuteMode m)
            {
                execMode = m;
            }

            public double3 Acceleration(int id)
            {
                double3 a = double3.zero;
                if ((bodies.propType[id] == Propagator.GRAVITY) ||
                    (bodies.propType[id] == Propagator.KEPLER)) {
                    if (integratorType == Integrators.Type.LEAPFROG) {
                        a = leapfrogData.a[id];
                    } else if (integratorType == Integrators.Type.RK4) {
                        a = rk4Data.CalcA(id, parms[GECore.DT_PARAM]);
                    }
                }
                return a;
            }

            private double RunStep(double t_end = 0)
            {
                // Are there maneuvers, can integrator do a variable timestep to maneuver time or not??

                // Do numerical integration: Massive, Massless
                bool numIntegration = (lenMassiveBodies.Value + lenMasslessBodies.Value) > 0;

                double t = 0.0;
                if (numIntegration) {
                    switch (refFrame) {
                        case GECore.ReferenceFrame.INERTIAL:
                            if (integratorType == Integrators.Type.LEAPFROG) {
                                t = Integrators.LeapfrogStep(ref this);
                            } else if (integratorType == Integrators.Type.RK4) {
                                t = Integrators.RK4Step(ref this);
                            }
                            break;
                        case GECore.ReferenceFrame.COROTATING_CR3BP:
                            if (integratorType == Integrators.Type.LEAPFROG) {
                                // t = CR3BP.LeapfrogStep(ref this);
                                throw new NotImplementedException("No LF for CR3BP");
                            } else if (integratorType == Integrators.Type.RK4) {
                                t = CR3BP.RK4Step(ref this);
                            }
                            break;
                    }
                } else if (t_end > 0) {
                    t = t_end;
                } else {
                    t = parms[GECore.T_PARAM] + parms[GECore.DT_PARAM];
                }

                // Handle any External Accelerations with dedicated integration
                // (these MAY have run through LF/RK4 but that value of r, v will get stomped by the self-integrating values)
                ExternalAccel.EAStateData eaState = new ExternalAccel.EAStateData();
                int bodyId;
                // deeply ugly, but need to do any Boosters first so they can update the shadow for EAR. (To cleanup make a list of just boosters)
                bool[] isBoosters = new bool[] { true, false };
                foreach (bool isBooster in isBoosters) {
                    for (int i = 0; i < lenExtAccelIndices.Value; i++) {
                        int extAccelId = extAccelIndices[i];
                        if (extADesc[extAccelId].type == ExternalAccel.ExtAccelType.SELF_INTEGRATED) {
                            if ((isBooster && extADesc[extAccelId].forceId == ExternalAccel.AccelType.BOOSTER) ||
                                (!isBooster && extADesc[extAccelId].forceId != ExternalAccel.AccelType.BOOSTER)) {
                                bodyId = extADesc[extAccelId].bodyId;
                                eaState.Init(ref bodies, bodyId);
                                int status;
                                double3 r, v;
                                (status, r, v) = ExternalAccel.ExtAccelFactorySelfIntegrated(ref eaState, extAccelId, t, parms[GECore.DT_PARAM], ref this);
                                if (GravityMath.HasNaN(r) || GravityMath.HasNaN(v)) {
                                    UnityEngine.Debug.LogErrorFormat("NaN r={0} v={1} for bodyId={2} extAccelId={3} t={4}", r, v, bodyId, extAccelId, t);
                                }
                                if (status != 0) {
                                    // need the ExtAccel to tell us how to handle a non-zero status
                                    (bool remove, bool stopSelfIntegrating) = ExternalAccel.SelfIntegratingStatusActions(status, extADesc[extAccelId].forceId);
                                    // most events (STAGING, REENTRY_COLLISION) events are generated EvolveSelfIntegrated()
                                    if (remove) {
                                        RemoveIndexByValue(ref extAccelIndices, lenExtAccelIndices.Value, extAccelId);
                                        lenExtAccelIndices.Value -= 1;
                                        bodies.self_integrating[bodyId] = false;
                                        // generate an event so GECore can remove the external acceleration from it's parallel book-keeping (sigh)
                                        PhysEvent eaEvent = new PhysEvent {
                                            bodyId = bodyId,
                                            statusCode = status,
                                            type = EventType.EXTERNAL_ACCELERATION_REMOVED,
                                            r = r,
                                            v = v,
                                            t = t,
                                            auxIndex = extAccelId
                                        };
                                        gepcEvents.Add(eaEvent);
                                    }

                                    if (stopSelfIntegrating) {
                                        bodies.self_integrating[bodyId] = false;
                                        // if body is Kepler, we need to re-init the propagator (it has mu, centerId)
                                        if (bodies.propType[bodyId] == Propagator.KEPLER) {
                                            double mu = keplerPropInfo[bodies.propIndex[bodyId]].rvt.mu;
                                            int centerId = keplerPropInfo[bodies.propIndex[bodyId]].centerId;
                                            int depth = keplerPropInfo[bodies.propIndex[bodyId]].keplerDepth;
                                            keplerPropInfo[bodies.propIndex[bodyId]] =
                                                new KeplerPropagator.PropInfo(r, v, t, mu, centerId, depth);
                                        }
                                        // TODO: Other propagators??
                                    }

                                } else {
                                    // this will OVERWRITE any massive body evolution
                                    bodies.r[bodyId] = r;
                                    bodies.v[bodyId] = v;
                                    // TODO: What if this is around a moving body??
                                    // Idea: EADescr could indicate center body id
                                }
                            }
                        }
                    }
                }

                // Scan the list of patched objects to align the patch with the current time
                for (int j = 0; j < lenPatchedBodies.Value; j++) {
                    int i = patchedBodies[j];
                    if (bodies.self_integrating[i]) {
                        continue;
                    }
                    // is the current patch in bounds?
                    int p_old = bodies.patchIndex[i];
                    if (!patches[p_old].InRange(t)) {
                        int p = PatchForTime(t, p_old);
                        if (p < 0) {
                            if (bodies.earlyPropagation[i] == EarlyPropagation.PROPAGATE) {
                                p = PatchFirst(p_old);
                            } else {
                                // Handle error
                                throw new System.NotImplementedException(
                                    string.Format("Bad patch index id={0} t_phys={1}", i, t));
                            }
                        } else {
                            bodies.patchIndex[i] = p;
                            if (patches[p].prop != bodies.propType[i]) {
                                IndexListSwitch(patches[p_old].prop, patches[p].prop, i);
                            }
                            bodies.propType[i] = patches[p].prop;
                            bodies.propIndex[i] = patches[p].propIndex;
                            PhysEvent patchEvent = new() {
                                statusCode = 0,
                                bodyId = i,
                                type = EventType.PATCH_CHANGE,
                                r = bodies.r[i],
                                v = bodies.v[i],
                                t = t,
                                auxIndex = p
                            };
                            gepcEvents.Add(patchEvent);
                        }
                    }
                }

                // Special case for a patched fixed endpoint (e.g. SGP4 decay in patch mode). We want to be able to rewind
                // to earlier times, so need handle endpoint here.
                if (lenFixedBodies.Value > 0) {
                    for (int i = 0; i < lenFixedBodies.Value; i++) {
                        int id = fixedBodies[i];
                        if (bodies.patchIndex[id] >= 0) {
                            int p = bodies.patchIndex[id];
                            if (patches[p].prop == Propagator.FIXED) {
                                bodies.r[id] = patches[p].r_fixed;
                                bodies.v[id] = patches[p].v_fixed;
                            }
                        }
                    }
                }

                if (lenKeplerMassive.Value > 0) {
                    // update RV for massive Kepler bodies. This gets us RELATIVE positions
                    KeplerPropagator.EvolveKeplerMassive(t, ref bodies, ref keplerPropInfo, ref keplerMassive, lenKeplerMassive.Value);
                }

                // TODO: Optimize. Move these to RunToTime IF we're not checking collisions & all on rails
                // Kepler MASSLESS evolution
                bool coRoFrame = refFrame == GECore.ReferenceFrame.COROTATING_CR3BP;
                if (lenKeplerMassless.Value > 0) {
                    int numEvents = gepcEvents.Length;
                    KeplerPropagator.EvolveMasslessProgagators(t, ref bodies, ref keplerPropInfo, ref patches, ref keplerMassless, lenKeplerMassless.Value, ref gepcEvents, coRoFrame: coRoFrame);
                    if (numEvents != gepcEvents.Length) {
                        // had Kepler errors. Need to remove those bodies so they stop generating errors
                        for (int i = numEvents; i < gepcEvents.Length; i++) {
                            BodyRemoveImmediate(gepcEvents[i].bodyId);
                        }
                    }
                }

                // PKEPLER evolution
                if (lenPkeplerBodies.Value > 0) {
                    int status = PKeplerPropagator.EvolveAll(t, ref bodies, ref pKeplerPropInfo, in patches, in pkeplerBodies,
                            lenPkeplerBodies.Value, scaleLtoKm, scaleKmsecToV, scaleTtoSec);
                    if (status != PKeplerPropagator.STATUS_OK) {
                        for (int a = 0; a < lenPkeplerBodies.Value; a++) {
                            int i = pkeplerBodies[a];
                            PKeplerPropagator.PropInfo pkeplerProp = pKeplerPropInfo[bodies.propIndex[i]];
                            if (pkeplerProp.status != PKeplerPropagator.STATUS_OK) {
                                BodyRemoveImmediate(i);
                                PhysEvent pkeplerEvent = new() {
                                    statusCode = pkeplerProp.status,
                                    type = EventType.PKEPLER_ERROR,
                                    bodyId = i,
                                    r = bodies.r[i],
                                    v = bodies.v[i],
                                    t = t
                                };
                                gepcEvents.Add(pkeplerEvent);
                            }
                        }
                    }
                }

                // KEPLER_COE evolution
                if (lenKeplerCoeBodies.Value > 0) {
                    int status = KeplerCOEPropagator.EvolveAll(t, ref bodies, ref keplerCoePropInfo, in patches, in keplerCoeBodies,
                            lenKeplerCoeBodies.Value);
                    if (status != KeplerCOEPropagator.STATUS_OK) {
                        for (int a = 0; a < lenKeplerCoeBodies.Value; a++) {
                            int i = keplerCoeBodies[a];
                            KeplerCOEPropagator.PropInfo kcoeProp = keplerCoePropInfo[bodies.propIndex[i]];
                            if (kcoeProp.status != KeplerCOEPropagator.STATUS_OK) {
                                BodyRemoveImmediate(i);
                                PhysEvent kcoeEvent = new() {
                                    statusCode = kcoeProp.status,
                                    type = EventType.KEPLER_ERROR,
                                    bodyId = i,
                                    r = bodies.r[i],
                                    v = bodies.v[i],
                                    t = t
                                };
                                gepcEvents.Add(kcoeEvent);
                            }
                        }
                    }
                }

                // SGP4 evolution
                if (lenSgp4Bodies.Value > 0) {
                    bool statusChange = SGP4Propagator.EvolveAll(t * scaleTtoSec,
                                                              ref bodies,
                                                              ref sgp4PropInfo,
                                                              in patches,
                                                              in sgp4Bodies,
                                                              lenSgp4Bodies.Value,
                                                              scaleLtoKm,
                                                              scaleKmsecToV,
                                                              parms[GECore.START_TIME_JD]);
                    // check for error conditions and generate an event if req'd
                    if (statusChange) {
                        for (int a = 0; a < lenSgp4Bodies.Value; a++) {
                            int i = sgp4Bodies[a];
                            SGP4Propagator.PropInfo sgp4Prop = sgp4PropInfo[bodies.propIndex[i]];
                            if (sgp4Prop.status != SGP4Propagator.OK) {
                                // creation of DECAY end patch is done in GECore.PhysicsLoopComplete()
                                BodyRemoveImmediate(i);
                                // create an event 
                                PhysEvent sgpEvent = new() {
                                    statusCode = sgp4Prop.status,
                                    type = EventType.SGP4_ERROR,
                                    bodyId = i,
                                    r = bodies.r[i],
                                    v = bodies.v[i],
                                    t = t
                                };
                                gepcEvents.Add(sgpEvent);
                            }
                        }
                    }
                }
                if (lenRotationBodies.Value > 0) {
                    RotationPropagator.EvolveAll(t, ref bodies, ref rotationPropInfo, ref rotationBodies,
                            lenRotationBodies.Value);
                }

                // if CoRo, adjust all propagators to the CoRo frame    
                if (coRoFrame) {
                    if (lenKeplerMassive.Value > 0) {
                        for (int a = 0; a < lenKeplerMassive.Value; a++) {
                            int i = keplerMassive[a];
                            (double3 r, double3 v) = CR3BP.FrameInertialToRotating(bodies.r[i], bodies.v[i], t);
                            bodies.r[i] = r;
                            bodies.v[i] = v;
                        }
                    }
                    if (lenKeplerMassless.Value > 0) {
                        for (int a = 0; a < lenKeplerMassless.Value; a++) {
                            int i = keplerMassless[a];
                            (double3 r, double3 v) = CR3BP.FrameInertialToRotating(bodies.r[i], bodies.v[i], t);
                            bodies.r[i] = r;
                            bodies.v[i] = v;
                        }
                    }
                    if (lenPkeplerBodies.Value > 0) {
                        for (int a = 0; a < lenPkeplerBodies.Value; a++) {
                            int i = pkeplerBodies[a];
                            (double3 r, double3 v) = CR3BP.FrameInertialToRotating(bodies.r[i], bodies.v[i], t);
                            bodies.r[i] = r;
                            bodies.v[i] = v;
                        }
                    }
                    if (lenSgp4Bodies.Value > 0) {
                        for (int a = 0; a < lenSgp4Bodies.Value; a++) {
                            int i = sgp4Bodies[a];
                            (double3 r, double3 v) = CR3BP.FrameInertialToRotating(bodies.r[i], bodies.v[i], t);
                            bodies.r[i] = r;
                            bodies.v[i] = v;
                        }
                    }
                    if (lenRotationBodies.Value > 0) {
                        for (int a = 0; a < lenRotationBodies.Value; a++) {
                            int i = rotationBodies[a];
                            (double3 r, double3 v) = CR3BP.FrameInertialToRotating(bodies.r[i], bodies.v[i], t);
                            bodies.r[i] = r;
                            bodies.v[i] = v;
                        }
                    }
                }


                if (ephemPropInfo.IsCreated && lenEphemBodies.Value > 0) {
                    EphemerisPropagator.Evolve(t, ref bodies, ref ephemPropInfo, ref ephemBodies, lenEphemBodies.Value, ref ephemerisData);
                }
                // check for collisions
                if (colliders.Length > 0)
                    CollisionChecks(t);

                return t;
            }

            /// <summary>
            /// Run a collision check between all objects that have registered a collider.
            /// </summary>
            /// <param name="t"></param>
            private void CollisionChecks(double t)
            {
                for (int i = 0; i < colliders.Length; i++) {
                    int cIndex1 = colliders[i];
                    if (collisionInfo[cIndex1].removed)
                        continue;
                    int id_i = collisionInfo[cIndex1].id;
                    double3 r_i = bodies.r[id_i];
                    for (int j = i + 1; j < colliders.Length; j++) {
                        int cIndex2 = colliders[j];
                        if (collisionInfo[cIndex2].removed)
                            continue;
                        int id_j = collisionInfo[cIndex2].id;
                        double dr = math.length(r_i - bodies.r[id_j]);
                        // special case: if both have zero radius don't check with each other
                        if (collisionInfo[i].radius == 0.0 && collisionInfo[j].radius == 0.0)
                            continue;
                        if (dr < (collisionInfo[i].radius + collisionInfo[j].radius)) {
                            // handle collision. More massive object type is used
                            int primary = cIndex1;
                            int secondary = cIndex2;
                            if (bodies.mu[id_i] < bodies.mu[id_j]) {
                                primary = cIndex2;
                                secondary = cIndex1;
                            }
                            if (uniqueCollisionEvents) {
                                // scan the current events and see if this has been added already
                                foreach (PhysEvent pe in gepcEvents) {
                                    if ((primary == pe.bodyId) && (secondary == pe.bodyId_secondary)) {
                                        continue;
                                    }
                                }
                            }
                            PhysEvent record = new PhysEvent {
                                auxIndex = primary,
                                bodyId = collisionInfo[primary].id,
                                bodyId_secondary = collisionInfo[secondary].id
                            };
                            record.r = bodies.r[record.bodyId];
                            record.v = bodies.v[record.bodyId];
                            record.r_secondary = bodies.r[record.bodyId_secondary];
                            record.v_secondary = bodies.v[record.bodyId_secondary];
                            record.t = t;
                            record.collisionType = collisionInfo[primary].cType;
                            switch (collisionInfo[primary].cType) {
                                case CollisionType.BOUNCE:
                                    // can only do momtm conservation if both have mass.
                                    BounceCollisionStateUpdate(id_i, id_j);
                                    break;
                                case CollisionType.ABSORB:
                                    // need to set the ligher inactive for rest of evolution
                                    int toAbsorb = i;
                                    if (collisionInfo[i].massInertial < collisionInfo[j].massInertial)
                                        toAbsorb = j;
                                    BodyRemoveImmediate(toAbsorb);
                                    // Deactivate collision record for j so it will not get added again
                                    ColliderInfo cInfo = new ColliderInfo(collisionInfo[toAbsorb]) {
                                        removed = true
                                    };
                                    collisionInfo[toAbsorb] = cInfo;
                                    break;

                                case CollisionType.TRIGGER:
                                    break;
                            }
                            gepcEvents.Add(record);
                        }
                    }
                }
            }

            /// <summary>
			/// A bounce collision between collider i and collider j has been detected. This will be treated as a point-point
			/// spherical collision. The post-collision velocities are determined and updated in the bodies state arrays.
			///
			/// Approach follows Coutinho "Dynanics simulations of multi-body systems". Springer. 2001.
			///
			/// Note this is very simplisitic collision handling and there is no multi-body collision detection/resolution
			/// or management of contact settling etc.
			/// 
			/// </summary>
			/// <param name="c_i">collider index of first body</param>
			/// <param name="c_j">collider index of second body</param>
            private void BounceCollisionStateUpdate(int c_i, int c_j)
            {
                int i = collisionInfo[c_i].id;
                int j = collisionInfo[c_j].id;
                double3 v1 = bodies.v[i];
                double3 v2 = bodies.v[j];
                double m1 = collisionInfo[c_i].massInertial;
                double m2 = collisionInfo[c_j].massInertial;
                double m12 = m1 * m2 / (m1 + m2);
                double3 vcm = (m1 * v1 + m2 * v2) / (m1 + m2);
                v1 -= vcm;
                v2 -= vcm;
                // e: coefficient of restitution
                double e = 0.5 * collisionInfo[c_i].bounceFactor + collisionInfo[c_j].bounceFactor;
                // resolve into normal (n) and tangent plane vectors (t, k)
                double3 n = math.normalize(bodies.r[j] - bodies.r[i]);
                // we want the rel. velocity along normal to be negative  (3.27)
                if (math.dot(v1 - v2, n) > 0)
                    n *= -1.0;
                // find a tangent vector, seed with a permuted version of normal
                double3 seed = new double3(n.y, n.z, n.x);
                double3 t = math.normalize(math.cross(n, seed));
                double3 k = math.normalize(math.cross(n, t));
                // components
                double v1n = math.dot(v1, n);
                double v1t = math.dot(v1, t);
                double v1k = math.dot(v1, k);
                double v2n = math.dot(v2, n);
                double v2t = math.dot(v2, t);
                double v2k = math.dot(v2, k);
                double mu_f = 0.1;
                double mu_dt = mu_f * math.sign((v2t - v1t) / (v2n - v1n)); // friction
                double mu_dk = mu_f * math.sign((v2k - v1k) / (v2n - v1n)); // friction
                // main equations (3.50)-(3.55)
                double dve = (1 + e) * (v2n - v1n);
                double U1n = m12 * dve;
                double U1t = mu_dt * m12 * dve;
                double U1k = mu_dk * m12 * dve;
                double U2n = -m12 * dve;
                double U2t = -mu_dt * m12 * dve;
                double U2k = -mu_dk * m12 * dve;
                // assemble result
                double V1n = (U1n + m1 * v1n) / m1;
                double V1t = (U1t + m1 * v1t) / m1;
                double V1k = (U1k + m1 * v1k) / m1;
                double V2n = (U2n + m2 * v2n) / m2;
                double V2t = (U2t + m2 * v2t) / m2;
                double V2k = (U2k + m2 * v2k) / m2;
                double3 V1 = V1n * n + V1t * t + V1k * k + vcm;
                double3 V2 = V2n * n + V2t * t + V2k * k + vcm;
                bodies.v[i] = V1;
                bodies.v[j] = V2;
            }

            /// <summary>
			/// Remove a body from immediate evolution so that subsequent RunStep() calls will
			/// not change it's state or use it's state.
			/// 
			/// </summary>
			/// <param name="index"></param>
            private void BodyRemoveImmediate(int index)
            {
                switch (bodies.propType[index]) {
                    case Propagator.GRAVITY:
                        if (bodies.mu[index] > 0) {
                            RemoveIndexByValue(ref massiveBodies, lenMassiveBodies.Value, index);
                            lenMassiveBodies.Value--;
                        } else {
                            RemoveIndexByValue(ref masslessBodies, lenMasslessBodies.Value, index);
                            lenMasslessBodies.Value--;
                        }
                        break;

                    case Propagator.KEPLER:
                        if (bodies.mu[index] > 0) {
                            RemoveIndexByValue(ref keplerMassive, lenKeplerMassive.Value, index);
                            lenKeplerMassive.Value--;
                        } else {
                            RemoveIndexByValue(ref keplerMassless, lenKeplerMassless.Value, index);
                            lenKeplerMassless.Value--;
                        }
                        break;

                    case Propagator.PKEPLER:
                        RemoveIndexByValue(ref pkeplerBodies, lenPkeplerBodies.Value, index);
                        lenPkeplerBodies.Value--;
                        break;

                    case Propagator.SGP4_RAILS:
                        RemoveIndexByValue(ref sgp4Bodies, lenSgp4Bodies.Value, index);
                        lenSgp4Bodies.Value--;
                        break;

                    case Propagator.FIXED:
                        RemoveIndexByValue(ref fixedBodies, lenFixedBodies.Value, index);
                        lenFixedBodies.Value--;
                        break;

                    case Propagator.EPHEMERIS:
                        RemoveIndexByValue(ref ephemBodies, lenEphemBodies.Value, index);
                        lenEphemBodies.Value--;
                        break;

                    case Propagator.KEPLER_COE:
                        RemoveIndexByValue(ref keplerCoeBodies, lenKeplerCoeBodies.Value, index);
                        lenKeplerCoeBodies.Value--;
                        break;
                }
                bodies.propType[index] = Propagator.UNASSIGNED;
            }


            /// <summary>
            /// Run to the indicated time or the next dT slice beyond the indicated time. 
            /// If all are on rails, then we can advance to the exact end time.
            /// </summary>
            /// <param name="t_end"></param>
            private void RunToTime(double t_end)
            {
                bool onRails = (lenMassiveBodies.Value + lenMasslessBodies.Value) == 0;

                double t = parms[GECore.T_PARAM];
                int loopCnt = 0;
                while (t < t_end) {
                    // Check for Maneuver
                    double t_next = t + parms[GECore.DT_PARAM];
                    if (onRails) {
                        t_next = t_end;
                    }
                    if (maneuvers.Length > 0) {
                        int m = 0;
                        while ((m < maneuvers.Length) && (maneuvers[m].t <= t_next)) {
                            // if onRails, then we can advance to the maneuver time exactly. Don't evolve
                            // if maneuver is at t. 
                            if (onRails && (maneuvers[m].t > t)) {
                                t = RunStep(t_end: maneuvers[m].t);
                                parms[GECore.T_PARAM] = t;
                            }
                            // do the maneuver
                            int index = maneuvers[m].id;
                            double3 v_center = double3.zero;
                            int cId = maneuvers[m].centerIndex;
                            if (cId >= 0) {
                                v_center = bodies.v[cId];
                            }
                            // prep an event to record the maneuver
                            PhysEvent mEvent = new() {
                                statusCode = 0,
                                type = EventType.MANEUVER,
                                r = bodies.r[index],
                                v = bodies.v[index],
                                t = t,
                                bodyId = index,
                                auxIndex = maneuvers[m].uniqueId
                            };
                            UnityEngine.Debug.LogFormat("Maneuver {0} at t={1} vs m.t={2}", maneuvers[m].LogString(), t, maneuvers[m].t);
                            if (bodies.patchIndex[index] >= 0) {
                                // odd ball case. Applying a maneuver to the current patch (since maneuver needs current state)
                                // Something went wrong. ManeuverAdd should have added a patch and absorbed the maneuver
                                mEvent.type = EventType.MANEUVER_ERROR;
                                mEvent.statusCode = MANEUVER_ON_PATCH;
                            } else {
                                bodies.v[index] = maneuvers[m].ApplyManeuver(bodies.v[index], v_center);
                                switch (bodies.propType[index]) {
                                    case Propagator.GRAVITY:
                                        break;

                                    case Propagator.KEPLER:
                                        // TODO: Could evolve to the exact maneuver time, then apply the maneuver
                                        // update the propagator to the new RVT
                                        KeplerPropagator.ManeuverPropagator(index, t, ref bodies, ref keplerPropInfo);
                                        break;

                                    case Propagator.PKEPLER:
                                        // update the propagator to the new RVT
                                        PKeplerPropagator.ManeuverPropagator(index, t, ref bodies, ref pKeplerPropInfo, scaleLtoKm, scaleKmsecToV);
                                        break;

                                    case Propagator.KEPLER_COE:
                                        // update the propagator to the new RVT by recomputing COE init from current RV
                                        KeplerCOEPropagator.ManeuverPropagator(index, t, ref bodies, ref keplerCoePropInfo);
                                        break;

                                    case Propagator.SGP4_RAILS:
                                        // update the propagator to the new RVT
                                        double tSec = t * scaleTtoSec;
                                        double timeJD = parms[GECore.START_TIME_JD] + TimeUtils.SecToJD(tSec);
                                        SGP4Propagator.ManeuverPropagator(index, tSec, timeJD, ref bodies, ref sgp4PropInfo, scaleLtoKm, scaleKmsecToV);
                                        break;

                                    default:
                                        mEvent.statusCode = BODY_TYPE_UNKNOWN;
                                        mEvent.type = EventType.MANEUVER_ERROR;
                                        break;
                                }
                            }
                            // record post-maneuver velocity
                            mEvent.v_secondary = bodies.v[index];
                            gepcEvents.Add(mEvent);
                            // a bit sneaky, leave index the same and redo loop with shuffled down list
                            maneuvers.RemoveAt(m);
                        } // while maneuvers
                    } // end of maneuvers loop (all maneuvers at new time)
                    t = RunStep(t_next);
                    loopCnt++;
                    parms[GECore.T_PARAM] = t;
                    //UnityEngine.Debug.LogFormat("Run to {0} loops={1} dt={2}", t_end, loopCnt, parms[GravityEngine.DT_PARAM]);
                } // while(t < t_end)

            }

            private void RunAndRecord()
            {
                // do run in bursts from timePoint to timePoint
                int tp = 0;
                while (tp < recordTimes.Length) {
                    RunToTime(recordTimes[tp]);
                    int baseIndex = tp * recordBodies.Length;
                    double t = parms[GECore.T_PARAM];
                    recordTimes[tp] = t;
                    // have a point to record
                    int bodyIndex;
                    for (int i = 0; i < recordBodies.Length; i++) {
                        bodyIndex = recordBodies[i];
                        recordedState[baseIndex + i] = new GEBodyState(bodies.r[bodyIndex], bodies.v[bodyIndex], t);
                    }
                    tp++;
                }
            }

            private void RunTrajectory()
            {
                // run to each record time point that is in the interval [t_now, t_until]
                // no need to run any further than that.
                // Intervals are at timeInterval boundaries
                double tInterval = parms[GECore.TRAJ_INTERVAL_PARAM];
                int currentInterval = (int)(parms[GECore.T_PARAM] / tInterval);
                int lastInterval = (int)(parms[GECore.T_END_PARAM] / tInterval);
                int i = currentInterval + 1;
                // current is last one written to
                int index = (int)parms[GECore.TRAJ_INDEX_PARAM];
                int size = (int)parms[GECore.TRAJ_NUMSTEPS_PARM];
                int numBodies = recordBodies.Length;
                while (i <= lastInterval) {
                    RunToTime(i * tInterval);
                    index = (index + 1) % size;
                    double t = parms[GECore.T_PARAM];
                    // have a point to record
                    for (int j = 0; j < numBodies; j++) {
                        recordedState[index * numBodies + j] =
                            new GEBodyState(bodies.r[recordBodies[j]], bodies.v[recordBodies[j]], t);
                    }
                    i++;
                }
                // index points to last entry written to
                parms[GECore.TRAJ_INDEX_PARAM] = index;
                parms[GECore.TRAJ_NUM_UPDATED_PARAM] = lastInterval - currentInterval;
            }

            public void Evolve()
            {
                switch (execMode) {
                    case ExecuteMode.NORMAL:
                        RunToTime(parms[GECore.T_END_PARAM]);
                        break;
                    case ExecuteMode.RECORD:
                        RunAndRecord();
                        break;
                    case ExecuteMode.TRAJECTORY:
                        RunTrajectory();
                        break;
                }

            }

            void IJob.Execute()
            {
                Evolve();
            }

            public void Dispose()
            {
                maneuvers.Dispose();
                parms.Dispose();
                keplerPropInfo.Dispose();
                pKeplerPropInfo.Dispose();
                keplerCoePropInfo.Dispose();
                sgp4PropInfo.Dispose();
                ephemPropInfo.Dispose();
                rotationPropInfo.Dispose();

                leapfrogData.Dispose();
                rk4Data.Dispose();

                recordBodies.Dispose();
                recordTimes.Dispose();
                recordedState.Dispose();

                extADesc.Dispose();
                extAccelIndices.Dispose();
                extAData.Dispose();

                massiveBodies.Dispose();
                masslessBodies.Dispose();
                fixedBodies.Dispose();
                keplerMassive.Dispose();
                keplerMassless.Dispose();
                pkeplerBodies.Dispose();
                keplerCoeBodies.Dispose();
                sgp4Bodies.Dispose();
                ephemBodies.Dispose();
                rotationBodies.Dispose();

                gepcEvents.Dispose();
                colliders.Dispose();
                collisionInfo.Dispose();

                lenMassiveBodies.Dispose();
                lenMasslessBodies.Dispose();
                lenKeplerMassive.Dispose();
                lenKeplerMassless.Dispose();
                lenFixedBodies.Dispose();
                lenPkeplerBodies.Dispose();
                lenKeplerCoeBodies.Dispose();
                lenSgp4Bodies.Dispose();
                lenEphemBodies.Dispose();
                lenPatchedBodies.Dispose();
                lenRotationBodies.Dispose();
                lenExtAccelIndices.Dispose();

                patchedBodies.Dispose();
                patches.Dispose();

                if (ephemerisData.IsCreated)
                    ephemerisData.Dispose();
            }
        }
    }
}
