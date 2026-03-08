using UnityEngine;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Burst;
using System.Collections.Generic;   // Dictionary
using Unity.Collections;

namespace GravityEngine2 {
    /// <summary>
    /// Evolve particles in the gravitation field computed by the GravityEngine. 
    ///
    /// This script is attached to a Unity ParticleSystem.
    ///
    /// Initialization of the particles may be handled by a script implementing IGravityParticleInit attached
    /// to the same game object that holds this component. Two common example are DustBall and DustRing.
    /// If there is no init delegate the Unity particle creation will be applied. 
    ///
    /// Each instance of GravityParticles manages the gravitational evolution of it's particle system.
    /// Particles are massless (they do not affect other particles) and are evolved with a built-in Leapfrog
    /// integrator. Particles which move within the particle capture radius of an NBody are made inactive
    /// and will not be further evolved. Particles that move outside the GravityEngine horizon are also
    /// made inactive. 
    /// </summary>
    [RequireComponent(typeof(ParticleSystem))]
    public class GSParticles : MonoBehaviour {
        private const bool debugLogs = false;

        //! If the particle system has an NBody or NBody parent, add the velocity of the NBody to particles that are created.
        public bool addGSBodyVelocity = false;
        //! Initial velocity to be added to particles (may be over-riden by an IGravityParticlesInit script, if present)
        public double3 initialVelocity = double3.zero;

        private ParticleSystem gravityParticles;
        private ParticleSystem.Particle[] particles;
        private int particleCount;
        // Useful to know if all particles created at start - removes need to do lifecycle handling
        private bool oneTimeBurst = false;

        // summary info (remove readonly when implement this fully)
        private const int inactiveCount = 0;

        // eject count unused - not sure a generic Out of Bounds is a good idea. Disable for now.
#pragma warning disable 0649
        private int ejectCount;
#pragma warning restore 0649

        // per-particle activity
        private bool allInactive;
        private bool playing;

        private double dt = 0;
        private double time = 0;

        //	private float outOfBoundsSquared; 
        private IGSParticlesInit particlesInit;
        private bool burstDone;

        private GSBody gsBodySource;

        private const double EPSILON = 1E-6;    // minimum distance for gravitatonal force

        /*
		Some Notes about particles:
		If all are created as a burst at start and live forever - things are simple. 

		When emission and extinction are happening, things get complicated. 

		If particle lifetime expires a particle from end of the active range is shuffled down. 
		As particles fade and number decreases get more shuffling down until get to 0 active and isStopped is true.

		When burst of long-lived particles are inactive, keep them in particle system but move them out of view
		This enures r,v,a data stays in correspondence to the particles array and these
		arrays do not need to be shuffled when particles are inactivated.

		Particle Lifecycle Handling:
		Code needs to detect when shuffling has happened and shuffle the physical variables as well. 
		- Shuffling detection is done by keeping a copy of each particles random seed and detecting when it has changed. 
		- new seed might be because a new particle overwrote, or because an existing particle was shuffled down
		- maintain a hashtable of seeds to physical array indices to determine if this was a shuffle (and copy physics info)
		*/

        private uint[] seed;    // tracking array for random seed
        private Dictionary<uint, int> particleBySeed;
        private int lastParticleCount;

        private const float OUT_OF_VIEW = 10000f;

        // useful in debugging to stop particle removal 
        private const bool allowRemoval = true;

        private GBUnits.GEScaler geScaler;

        public ParticleJob pJob;

        public void ParticleSetup(double dt, double time, GBUnits.GEScaler geScaler)
        {
            this.dt = dt;
            this.time = time;
            this.geScaler = geScaler;

            //		outOfBoundsSquared = gravityEngine.outOfBounds * gravityEngine.outOfBounds; 
            gravityParticles = GetComponent<ParticleSystem>();

            // Unity Jank?!
            // For IJobParallel mode MUST have prewarm set or else particle system does not start
            if (gravityParticles.main.prewarm || gravityParticles.main.loop)
                Debug.LogWarning("Turn off loop/pre-warm for job based particles");

            InitParticleData();

            // Had Play() here - but after Unity 5.3 this broke. Now moved to the update loop. 

            // Get the Init Delegate (if there is one)
            particlesInit = GetComponent<IGSParticlesInit>();

            if (particlesInit == null && addGSBodyVelocity) {
                // Find the associated NBody 
                // Can either be directly attached (particleSystem is attached directly to object with NBody) OR
                // can be the parent (if ParticleSystem has its own Game Object and is a child)
                gsBodySource = GetComponent<GSBody>();
                if (gsBodySource == null) {
                    gsBodySource = transform.parent.gameObject.GetComponent<GSBody>();
                }
            }

            particleBySeed = new Dictionary<uint, int>();

            // determine if this is a one-time burst scenario
            int burstCount = 0;
            ParticleSystem.Burst[] bursts = new ParticleSystem.Burst[gravityParticles.emission.burstCount];
            gravityParticles.emission.GetBursts(bursts);
            foreach (ParticleSystem.Burst burst in bursts) {
                burstCount += burst.maxCount;
            }
            if (burstCount == gravityParticles.main.maxParticles) {
                oneTimeBurst = true;
                burstDone = false;
            }
#pragma warning disable 162        // disable unreachable code warning
            if (debugLogs) {
                Debug.Log(this.name + " start: oneTimeBurst=" + oneTimeBurst + " burstCount=" +
                             burstCount + " pc=" + gravityParticles.main.maxParticles);
            }
#pragma warning restore 162
        }


        private void InitParticleData()
        {
            if (gravityParticles == null) {
                Debug.LogError("Must be attached to a particle system object");
                return;
            }
            if (gravityParticles.main.simulationSpace == ParticleSystemSimulationSpace.Local) {
                Debug.LogError("Particle simulation space must be set to World.");
            }
            if (gravityParticles.main.duration > 9999) {
                Debug.LogWarning("Duration > 9999 has resulted in no particles visible in the past.");
            }
            // create array to hold particles
            particles = new ParticleSystem.Particle[gravityParticles.main.maxParticles];
            // get particles from the system (this fills in particles[])
            gravityParticles.GetParticles(particles);
            int maxParticles = gravityParticles.main.maxParticles;
#pragma warning disable 162       // disable unreachable code warning
            if (debugLogs) {
                Debug.Log("Init numParticles=" + maxParticles);
            }
#pragma warning restore 162
            pJob = new ParticleJob();
            pJob.Init(maxParticles, time, dt);
            seed = new uint[maxParticles];
            allInactive = false;
        }


        // Compute the initial value of a[]. It will be used in the first part of Evolve[] 
        private void PreEvolve(int fromP, int toP, List<int> massiveIndices, ref GEPhysicsCore.GEBodies bodies)
        {
            // Precalc initial acceleration
            double3 rji;
            double r2;
            double r3;
            for (int i = fromP; i < toP; i++) {
                for (int k = 0; k < 3; k++) {
                    pJob.pData.a[i] = double3.zero;
                }
            }
            // evolve over all massive objects
            foreach (int i in massiveIndices) {
                if (bodies.mu[i] > 0) {
                    for (int j = fromP; j < toP; j++) {
                        rji = pJob.pData.r[j] - bodies.r[i];
                        r2 = 0;
                        for (int k = 0; k < 3; k++) {
                            r2 += rji[k] * rji[k];
                        }
                        r2 = rji.x * rji.x + rji.y * rji.y + rji.z * rji.z;
                        r3 = r2 * System.Math.Sqrt(r2) + EPSILON;
                        pJob.pData.a[j] += bodies.mu[i] * rji / r3;
#pragma warning disable 162, 429       // disable unreachable code warning
                        if (debugLogs && j == 0) {
                            Debug.Log(string.Format("PreEvolve : Initial a={0} {1} {2} rji={3} {4} {5} body={6} m={7}",
                                pJob.pData.a[0].x, pJob.pData.a[0].y, pJob.pData.a[0].z, rji.x, rji.y, rji.z, i, bodies.mu[i]));
                        }
#pragma warning restore 162, 429
                    }
                }
            }
        }

        // If there is no init delegate use the initial particle position (scaled to physics space) and the
        // initial particle velocity to seed the particle physics data. 
        private void NoInitDelegateSetup(int fromP, int toP)
        {
            Debug.LogWarning("TODO");
            //double3 sourceVelocity = double3.zero;
            //if (addGSBodyVelocity && gsBodySource != null) {
            //	GEBodyState state = new GEBodyState();
            //	if (ge.StateById(gsBodySource.Id(), ref state))
            //		sourceVelocity = state.v;
            //}
            //sourceVelocity += initialVelocity;
            //for (int i = fromP; i < toP; i++) {
            //	// particles are in world space - just use their position
            //	pJob.pData.r[i] = GravityMath.Vector3ToDouble3(particles[i].position) * scalePhysToScene;
            //	pJob.pData.v[i] = sourceVelocity + particles[i].velocity.x;
            //	pJob.pData.inactive[i] = false;
            //	particles[i].velocity = Vector3.zero;
            //}
        }


        //
        // Emmisive particle management
        // - per cycle look for new particles or cases where particles expired and were shuffled
        //

        void ParticleLifeCycleHandler(List<int> massiveIndices, ref GEPhysicsCore.GEBodies bodies)
        {
            // Particle life cycle management
            // - need GetParticles() call to get the correct number of active particle (p.particleCount did not work)
            // - IIUC this is a re-copy and it would be better to avoid this if possible
            particleCount = gravityParticles.GetParticles(particles);
            pJob.SetParticleCount(particleCount);
            if (lastParticleCount < particleCount) {
                //Debug.LogFormat("New particles: last={0} now={1}", lastParticleCount, particleCount);
                // there are new particles
                if (particlesInit != null) {
                    // Init will scale as necessary
                    particlesInit.InitNewParticles(lastParticleCount, gravityParticles.particleCount, geScaler,
                                                   ref pJob.pData.r, ref pJob.pData.v);
                } else {
                    NoInitDelegateSetup(lastParticleCount, gravityParticles.particleCount);
                }
                PreEvolve(lastParticleCount, gravityParticles.particleCount, massiveIndices, ref bodies);
                for (int i = lastParticleCount; i < gravityParticles.particleCount; i++) {
                    pJob.pData.inactive[i] = false;
                    seed[i] = particles[i].randomSeed;
                    particleBySeed[particles[i].randomSeed] = i;
                }
                lastParticleCount = gravityParticles.particleCount;
            }
            if (oneTimeBurst) {
                // not doing life cycle for this particle system
                return;
            }
            // Check if any existing particles were replaced. 
            // As particles expire, Unity will move particles from the end down into their slot and reduce
            // the number of active particles. Need to detect this and move their physics data.
            // This makes emmisive particle systems more CPU intensive. 
            for (int i = 0; i < particleCount; i++) {
                if (seed[i] != particles[i].randomSeed) {
                    // particle has been replaced
                    particleBySeed.Remove(seed[i]);
                    // remove old seed from hash
                    if (particleBySeed.ContainsKey(particles[i].randomSeed)) {
                        // particle was moved - copy physical data down
                        int oldIndex = particleBySeed[particles[i].randomSeed];
                        pJob.pData.r[i] = pJob.pData.r[oldIndex];
                        pJob.pData.v[i] = pJob.pData.v[oldIndex];
                        pJob.pData.a[i] = pJob.pData.a[oldIndex];
                        particleBySeed[particles[i].randomSeed] = i;
                    } else {
                        if (particlesInit != null) {
                            particlesInit.InitNewParticles(i, i + 1, geScaler, ref pJob.pData.r, ref pJob.pData.v);
                        } else {
                            NoInitDelegateSetup(i, i + 1);
                        }
                        PreEvolve(i, i + 1, massiveIndices, ref bodies);
                        particleBySeed[particles[i].randomSeed] = i;
                    }
                    seed[i] = particles[i].randomSeed;
                    pJob.pData.inactive[i] = false;
                }
            }
        }


        /// <summary>
        /// Direct evolution of particles without using the Job system. To maintain common code this
        /// is done by calling a method on the particle job, but we are not scheduling a job (yet). 
        /// </summary>
        /// <param name="evolveTime"></param>
        /// <param name="massiveIndices"></param>
        /// <param name="bodies"></param>
        /// <returns></returns>
        public double Evolve(double evolveTime,
                             List<int> massiveIndices, // TODO make an array ref for IJob
                             ref GEPhysicsCore.GEPhysicsJob geJob)
        {
            // do nothing if all inactive
            if (pJob.pData.inactive == null || allInactive) {
                return evolveTime;  // Particle system has not init-ed yet or is done
            }
            //  (did not work in Start() -> Unity bug? Init sequencing?)
            if (!playing) {
                gravityParticles.Play();
                playing = true;
            }
            ParticleLifeCycleHandler(massiveIndices, ref geJob.bodies);
            pJob.EvolveParticles(evolveTime, massiveIndices, ref geJob);
            return time;
        }

        public void ParallelJobSetup(List<int> massiveIndices, // TODO make an array ref for IJob
                     ref GEPhysicsCore.GEBodies bodies)
        {
            // do nothing if all inactive
            if (pJob.pData.inactive == null || allInactive) {
                return;  // Particle system has not init-ed yet or is done
            }
            //  (did not work in Start() -> Unity bug? Init sequencing?)
            if (!playing) {
                gravityParticles.Play();
                playing = true;
            }
            ParticleLifeCycleHandler(massiveIndices, ref bodies);
        }



#pragma warning disable 414 // unused if debug const is false		
        private int debugCnt = 0;
#pragma warning restore 414

        /// <summary>
        /// Updates the particles positions in world space.
        ///
        /// UpdateParticles is called from GSDisplay. Do not call from other scripts. 
        /// </summary>
        /// <param name="physicalScale">Physical scale.</param>
        public void UpdateParticles(GSDisplay gsDisplay, double lenPhysToWorld)
        {
            if (allInactive) {
                return;
            }
            // double itime = ge.GetTime();
            Vector3 pos;
            for (int i = 0; i < lastParticleCount; i++) {
                // double3 position = r[i] * scalePhysToScene;
                // always interpolate
                // position += itime * v[i];
                if (pJob.pData.inactive[i]) {
                    particles[i].remainingLifetime = 0;
                } else {
                    pos = GravityMath.Double3ToVector3(lenPhysToWorld * pJob.pData.r[i]);
                    // need to do physics to world
                    particles[i].position = gsDisplay.MapToScene(GravityMath.Vector3ToDouble3(pos));
                }
            }
            gravityParticles.SetParticles(particles, particleCount);
            // must be after display - so final inactivated particles are removed
            if (oneTimeBurst && burstDone && ((ejectCount + inactiveCount) >= particleCount)) {
                allInactive = true;
#pragma warning disable 162      // disable unreachable code warning
                if (debugLogs)
                    Debug.Log("All particles inactive! time = " + Time.time + " ejected=" + ejectCount + " inactive=" + inactiveCount +
                        " remaining=" + (particleCount - inactiveCount - ejectCount));
#pragma warning restore 162
            }
#pragma warning disable 162, 429       // disable unreachable code warning
            if (debugLogs && debugCnt++ > 30) {
                debugCnt = 0;
                string log = "time = " + Time.time + " ejected=" + ejectCount + " inactive=" + inactiveCount +
                        " particles=" + particleCount;
                log += " is Stopped " + gravityParticles.isStopped + " num=" + gravityParticles.particleCount + " pcount=" + particleCount + "\n";
                int logTo = (gravityParticles.main.maxParticles < 10) ? gravityParticles.main.maxParticles : 10;
                for (int i = 0; i < logTo; i++) {
                    log += string.Format("{0}  rand={1} life={2} inactive={3} ", i, particles[i].randomSeed, particles[i].remainingLifetime, pJob.pData.inactive[i]);
                    log += " pos=" + particles[i].position;
                    log += " phyPos= " + pJob.pData.r[i];
                    log += "\n";
                }
                Debug.Log(log);
            }
#pragma warning restore 162, 429
        }

        public void MoveAll(ref double3 moveBy)
        {
            for (int i = 0; i < lastParticleCount; i++) {
                pJob.pData.r[i] += moveBy;
            }
        }

        public void AdjustVelocity(double3 velAdjust)
        {
            for (int i = 0; i < lastParticleCount; i++) {
                pJob.pData.v[i] += velAdjust;
            }
        }

        private void OnDestroy()
        {
            pJob.Dispose();
        }

        //================================JOB CODE===================================

        /// <summary>
        /// Particle evolution.
        ///
        /// This is structured as a job, but a direct call to the execute will also work. 
        /// </summary>
        ///
        public struct ParticleData {
            public NativeArray<double3> r;
            public NativeArray<double3> v;
            public NativeArray<double3> a;
            public NativeArray<bool> inactive;
        }
#if PARTICLE_JOB
		public struct MassiveBodyData
        {
			public double3 r;
			public double3 v;
			public double mu;

			public MassiveBodyData(double3 r,
								   double3 v,
								   double mu)
            {
				this.r = r;
				this.v = v;
				this.mu = mu;
            }
        }

		public static NativeArray<MassiveBodyData> MassiveBodyDataInit(List<int> massiveIndices, ref GEPhysicsCore.GEBodies bodies)
        {
			NativeArray<MassiveBodyData> massiveData = new NativeArray<MassiveBodyData>(massiveIndices.Count, Allocator.Persistent);
			int n = 0; 
			foreach(int i in massiveIndices) {
				massiveData[n++] = new MassiveBodyData(bodies.r[i], bodies.v[i], bodies.mu[i]);
            }
			return massiveData;
        }
#endif
        public struct ParticleJob : IJob {
            public ParticleData pData;

            public double time;
            public double dt;
            public int particleCount;

            [ReadOnly]
            // public NativeArray<MassiveBodyData> massiveBodies;

            // times for particle evolution
            private double fromTime;
            private double toTime;
            private double endTime;

            public void Init(int numParticles, double time, double dt)
            {
                pData.r = new NativeArray<double3>(numParticles, Allocator.Persistent);
                pData.v = new NativeArray<double3>(numParticles, Allocator.Persistent);
                pData.a = new NativeArray<double3>(numParticles, Allocator.Persistent);
                pData.inactive = new NativeArray<bool>(numParticles, Allocator.Persistent);
                this.time = time;
                this.dt = dt;
                endTime = time;
                fromTime = time;
            }

            public void SetParticleCount(int n)
            {
                particleCount = n;
            }

            public void SetParticleTimes(double fromTime, double toTime)
            {
                this.fromTime = fromTime;
                this.toTime = toTime;
            }

            public double GetParticleEndTime()
            {
                return endTime;
            }

            /// <summary>
            /// Core physics loop for particle evolution.
            ///
            /// Because particle life cycles are closely tied to a specific GSDisplay instance the
            /// particle physics is done on th emain thread for now. (I do want to job-ify it, but that
            /// needs more debugging). 
            /// </summary>
            /// <param name="evolveTime"></param>
            /// <param name="massiveIndices"></param>
            /// <param name="bodies"></param>
            public void EvolveParticles(double evolveTime,
                     List<int> massiveIndices, // TODO make an array ref for IJob
                     ref GEPhysicsCore.GEPhysicsJob geJob)
            {
                while (time < evolveTime) {
                    time += dt;
                    // Update v and r
                    for (int i = 0; i < particleCount; i++) {
                        if (!pData.inactive[i]) {
                            pData.v[i] += pData.a[i] * 0.5 * dt;
                            pData.r[i] += pData.v[i] * dt;
                        }
                    }
                    // advance acceleration
                    double3 rji;
                    double r2;  // r squared
                    double r3;  // r cubed

                    // a = 0 
                    for (int i = 0; i < particleCount; i++) {
                        pData.a[i] = double3.zero;
                    }
                    // calc a
                    foreach (int i in massiveIndices) {
                        // check mass has inactive clear
                        if (geJob.bodies.mu[i] > 0) {
                            for (int j = 0; j < particleCount; j++) {
                                // only evolve active particles
                                if (!pData.inactive[j]) {
                                    rji = pData.r[j] - geJob.bodies.r[i];
                                    r2 = rji[0] * rji[0] + rji[1] * rji[1] + rji[2] * rji[2];
                                    // Check for incursion on massive bodies and inactivate particles that have collided
                                    // (Do not want to incur collider overhead per particle)
                                    //if (allowRemoval && r2 < nbodyState[i].size2) {
                                    //	inactive[j] = true;
                                    //	inactiveCount++;
                                    //	if (oneTimeBurst) {
                                    //		r[j] = OUT_OF_VIEW;
                                    //	}
                                    //	else {
                                    //		particles[j].remainingLifetime = 0;
                                    //	}
                                    //	continue;
                                    //}
                                    r3 = r2 * System.Math.Sqrt(r2);
                                    pData.a[j] -= geJob.bodies.mu[i] * rji / r3;
                                } // for j
                            } // info
                        } // for i
                    }
                    // External acceleration (C&P from Integrator)
                    // Only ON_OTHER applies in this scenario
                    ExternalAccel.EAStateData extData = new ExternalAccel.EAStateData();
                    foreach (int k in geJob.extAccelIndices) {
                        ExternalAccel.EADesc eaDesc = geJob.extADesc[k];
                        int extId = geJob.extADesc[k].bodyId;
                        extData.Init(ref geJob.bodies, extId);
                        double3 a_out = double3.zero;
                        int status;
                        for (int j = 0; j < particleCount; j++) {
                            if (geJob.extADesc[k].type == ExternalAccel.ExtAccelType.ON_OTHER) {
                                // apply accel to particle
                                extData.r_to = pData.r[j];
                                extData.v_to = pData.v[j];
                                double3 a_in = pData.a[j];
                                status = ExternalAccel.ExtAccelFactory(ref eaDesc, ref extData, time, dt, a_in, geJob.extAData, out a_out, ref geJob);
                                pData.a[j] += a_out;
                            }
                        }
                    }
                    // update velocity
                    for (int i = 0; i < particleCount; i++) {
                        if (!pData.inactive[i]) {
                            pData.v[i] += pData.a[i] * 0.5 * dt;
                        }
                    }
                } // while
            }
            void IJob.Execute()
            {
#if PARTICLE_JOB

                double t = fromTime;
				while (t < toTime) {
                    t += dt;
					foreach (MassiveBodyData mbd in massiveBodies) {
                        // check mass has inactive clear
                        if (mbd.mu > 0) {
							Debug.LogWarning("TODO ext ACCEL");
							for (int i = 0; i < particleCount; i++) {
                                if (!pData.inactive[i]) {

                                    // Update v and r
                                    pData.v[i] += pData.a[i] * 0.5 * dt;
                                    pData.r[i] += pData.v[i] * dt;
                                    // advance acceleration
                                    double3 rji;
                                    double r2;  // r squared
                                    double r3;  // r cubed

                                    // a = 0 
                                    pData.a[i] = double3.zero;
                                    // calc a
                                     // only evolve active particles
                                    rji = pData.r[i] - mbd.r;
                                    r2 = rji[0] * rji[0] + rji[1] * rji[1] + rji[2] * rji[2];
                                    // Check for incursion on massive bodies and inactivate particles that have collided
                                    // (Do not want to incur collider overhead per particle)
                                    //if (allowRemoval && r2 < nbodyState[i].size2) {
                                    //	inactive[j] = true;
                                    //	inactiveCount++;
                                    //	if (oneTimeBurst) {
                                    //		r[j] = OUT_OF_VIEW;
                                    //	}
                                    //	else {
                                    //		particles[j].remainingLifetime = 0;
                                    //	}
                                    //	continue;
                                    //}
                                    r3 = r2 * System.Math.Sqrt(r2);
                                    pData.a[i] -= mbd.mu * rji / r3;
                                } // info
                                // update velocity
                                pData.v[i] += pData.a[i] * 0.5 * dt;
                            }
                        } // for mbd

                    }
                } // while
#endif

            }
            public void Dispose()
            {
                pData.r.Dispose();
                pData.v.Dispose();
                pData.a.Dispose();
                pData.inactive.Dispose();

                // if (massiveBodies.IsCreated)
                // 	massiveBodies.Dispose();
            }

        }

    }

}
