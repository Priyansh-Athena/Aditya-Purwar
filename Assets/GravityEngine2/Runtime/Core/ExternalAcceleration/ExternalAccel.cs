using Unity.Mathematics;
using Unity.Burst;
using Unity.Collections;
using static GravityEngine2.GEPhysicsCore;  // GEBodies

namespace GravityEngine2 {
    /// <summary>
    /// The integrators in the GEPhysicsCore can be configured to include additional forces
    /// (beyond gravity) to their force computations. Typical examples include a continuous thrust
    /// rocket, atmospheric drag or changes to the gravitation force itself (e.g. 1/R)
    /// 
    /// See https://nbodyphysics.com/ge2/html/majorFeatures.html#external-forces-gsexternalacceleration
    /// 
    /// This class holds the common code to configure the properties of an external acceleration.
    /// 
    /// </summary>
    [BurstCompile(CompileSynchronously = true)]
    public class ExternalAccel {

        /// There are two types of external acceleration:
        /// SELF: Force invocation only affects the body the force is registered to. e.g. ion drive, rocket engine
        /// ON_OTHER: Force from body only affects other body e.g. model of an atmosphere
        public enum ExtAccelType { SELF, ON_OTHER, SELF_INTEGRATED, INACTIVE };

        /// <summary>
		/// Struct to hold arbitrary parameters for an external acceleration computation and body.
		/// </summary>
        public struct EADesc {
            public int bodyId;      // id number of the body that is linked to this force
            public ExtAccelType type;   // how does this force act

            public AccelType forceId;
            // function to call to compute acceleration
            public int paramBase;
            public int paramLen;
            public int myId;    // ID assigned when allocated to give a handle for deletion later

            public EADesc(EADesc from)
            {
                bodyId = from.bodyId;
                type = from.type;
                myId = from.myId;
                forceId = from.forceId;
                paramBase = from.paramBase;
                paramLen = from.paramLen;
                // eaInteractFn = from.eaInteractFn;
            }

            public string LogString()
            {
                return string.Format("bodyId={0} type={1} force={2}", bodyId, type, forceId);
            }
        }

        /// <summary>
		/// Data passed to an external accel function call. The state information for each of the two bodies and
		/// their effective masses are provided.
		///
		/// All values in world units.
		///
		/// The notion of "from" and "to" is indicate who the resulting acceleration will be applied to. 
		/// </summary>
        public struct EAStateData {
            public double3 r_from;
            public double3 v_from;
            public double mu_from;

            public double3 r_to;
            public double3 v_to;
            public double3 mu_to;

            public void Init(ref GEBodies bodies, int i, int j)
            {
                r_from = bodies.r[i];
                v_from = bodies.v[i];
                mu_from = bodies.mu[i];

                r_to = bodies.r[j];
                v_to = bodies.v[j];
                mu_to = bodies.mu[j];
            }

            public void Init(ref GEBodies bodies, int i)
            {
                r_from = bodies.r[i];
                v_from = bodies.v[i];
                mu_from = bodies.mu[i];

                r_to = double3.zero;
                v_to = double3.zero;
                mu_to = 0.0;
            }
#if TODO_DONE
            public void Init(GSParticles.MassiveBodyData mbd)
            {
                r_from = mbd.r;
                v_from = mbd.v;
                mu_from = mbd.mu;

                r_to = double3.zero;
                v_to = double3.zero;
                mu_to = 0.0;
            }
#endif
        }

        public static void ExtAccelDataAlloc(double3[] data,
                                             ref GEPhysicsJob gePhysJob,
                                             ref EADesc eaDesc)
        {
            int numPoints = data.Length;
            int size = gePhysJob.extAData.Length;
            int newSize = size + numPoints;
            NativeArray<double3> oldData = gePhysJob.extAData;
            gePhysJob.extAData = new NativeArray<double3>(newSize, Allocator.Persistent);
            NativeArray<double3>.Copy(oldData, 0, gePhysJob.extAData, 0, size);
            oldData.Dispose();
            // add new data
            for (int i = 0; i < numPoints; i++) {
                gePhysJob.extAData[size + i] = data[i];
            }
            eaDesc.paramBase = size;
            eaDesc.paramLen = numPoints;
        }

        public static void ExtAccelDataFree(ref GEPhysicsJob gePhysicsJob, int eaIndex)
        {
            NativeArray<double3> oldData = gePhysicsJob.extAData;
            int n = gePhysicsJob.extADesc[eaIndex].paramLen;
            int newSize = gePhysicsJob.extAData.Length - n;
            int baseIndex = gePhysicsJob.extADesc[eaIndex].paramBase;
            gePhysicsJob.extAData = new NativeArray<double3>(newSize, Allocator.Persistent);
            if (newSize > 0) {
                // copy old data up to the base index
                NativeArray<double3>.Copy(oldData, 0, gePhysicsJob.extAData, 0, baseIndex);
                // copy from end of removed data to the end
                if (baseIndex + n < oldData.Length) {
                    NativeArray<double3>.Copy(oldData, baseIndex + n, gePhysicsJob.extAData, baseIndex, oldData.Length - baseIndex - n);
                }
                oldData.Dispose();
                // need to adjust the paramBase for all subsequent external accelerations
                for (int i = 0; i < gePhysicsJob.extADesc.Length; i++) {
                    if (gePhysicsJob.extADesc[i].paramBase > baseIndex) {
                        int newBase = gePhysicsJob.extADesc[i].paramBase - n;
                        EADesc eaDescr = gePhysicsJob.extADesc[i];
                        eaDescr.paramBase = newBase;
                        gePhysicsJob.extADesc[i] = eaDescr;
                    }
                }
            }
            // null out the removed entry
            EADesc eaDescr2 = new EADesc();
            eaDescr2.myId = -1;
            eaDescr2.bodyId = -1;
            eaDescr2.type = ExtAccelType.INACTIVE;
            eaDescr2.paramBase = -1;
            eaDescr2.paramLen = -1;
            gePhysicsJob.extADesc[eaIndex] = eaDescr2;
        }

        public static double3[] ExtAccelData(ref GEPhysicsJob gePhysJob, int eaIndex)
        {
            if (eaIndex < 0) {
                return null;
            }
            if (gePhysJob.extADesc[eaIndex].myId < 0) {
                return null;
            }
            double3[] data = new double3[gePhysJob.extADesc[eaIndex].paramLen];
            int b = gePhysJob.extADesc[eaIndex].paramBase;
            for (int i = 0; i < data.Length; i++) {
                data[i] = gePhysJob.extAData[b + i];
            }
            return data;
        }

        public static void ExtAccelDataUpdate(ref GEPhysicsJob gePhysJob, int eaIndex, double3[] newData)
        {
            int b = gePhysJob.extADesc[eaIndex].paramBase;
            int len = gePhysJob.extADesc[eaIndex].paramLen;
            if (newData.Length != len) {
                UnityEngine.Debug.LogError("Data size mismatch");
                UnityEngine.Debug.Break();
                //throw new System.Exception("Data size mismatch");
            }
            for (int i = 0; i < len; i++) {
                gePhysJob.extAData[b + i] = newData[i];
            }
        }

        /// <summary>
        /// Extract a value from the data array given a look-up index. 
        /// a LUIndex is encoded as 2 bits to designate x, y, z entry
        /// and then the upper bits to encode the entry number in the 
        /// eaData array. 
        /// 
        /// Typical use is e.g. to allow the Earth atmosphere model to 
        /// retrive the inertial mass of a rocket booster and the booster
        /// mass is being decremented timestep by timestep during numerical 
        /// integration. 
        /// </summary>
        /// <param name="luIndex"></param>
        /// <param name="data"></param>
        /// <returns></returns>
        public static double LUValue(uint luIndex, NativeArray<double3> data)
        {
            double value = 0;
            uint field = luIndex & 0x3;
            int entry = (int)luIndex >> 2;
            switch (field) {
                case 1:
                    value = data[entry].x;
                    break;
                case 2:
                    value = data[entry].y;
                    break;
                case 3:
                    value = data[entry].z;
                    break;
            }
            return value;
        }

        public enum AccelType {
            INVERSE_R = 0,
            GRAVITY_J2 = 1,
            ION_DRIVE = 2,
            BOOSTER = 3,
            EARTH_ATMOSPHERE = 4,
            EARTH_ATMOSPHERE_REENTRY = 5,
            // Add custom External Forces here
            CUSTOM = 100,
        };

        public const int STATUS_NO_FORCE = 1;

        public const int STATUS_INACTIVE = 2;

        public static int ExtAccelFactory(ref EADesc eaDesc, ref EAStateData eaState, double t, double dt, double3 a_in, NativeArray<double3> eaData, out double3 a_out, ref GEPhysicsJob gePhysJob)
        {
            int status = 0;
            a_out = double3.zero;
            switch (eaDesc.forceId) {
                case AccelType.INVERSE_R:
                    status = InverseR.InvRAccel(ref eaState, ref eaDesc, t: t, dt: dt, ref a_in, eaData, out a_out);
                    break;
                case AccelType.GRAVITY_J2:
                    (status, a_out) = GravityJ2.J2Accel(ref eaState, ref eaDesc, t: t, dt: dt, a_in, eaData);
                    break;
                case AccelType.ION_DRIVE:
                    (status, a_out) = IonDrive.IonDriveAccel(ref eaState, ref eaDesc, t: t, dt: dt, a_in, eaData);
                    break;
                case AccelType.EARTH_ATMOSPHERE:
                    (status, a_out) = EarthAtmosphere.Accel(ref eaState, ref eaDesc, eaData, gePhysJob.ScaleLToKm(), gePhysJob.ScaleTtoSec(), gePhysJob.ScaleAccelSItoGE());
                    break;
                // Add user forces here
                default:
                    // Add an error event
                    status = STATUS_NO_FORCE;
                    break;
            }
            return status;
        }

        public static (int status, double3 r, double3 v) ExtAccelFactorySelfIntegrated(ref EAStateData eaState,
                                                                                        int extAccelId,
                                                                                        double tGE,
                                                                                        double dtGE,
                                                                                        ref GEPhysicsJob gePhysJob)
        {
            int status = 0;
            double3 r = double3.zero;
            double3 v = double3.zero;
            NativeArray<ExternalAccel.EADesc> eaDescs = gePhysJob.extADesc;
            switch (eaDescs[extAccelId].forceId) {

                case AccelType.BOOSTER:
                    (status, r, v) = Booster.EvolveSelfIntegrated(ref eaState, extAccelId, tGE: tGE, dt: dtGE, ref gePhysJob);
                    break;

                case AccelType.EARTH_ATMOSPHERE_REENTRY:
                    (status, r, v) = EarthAtmosphereReentry.EvolveSelfIntegrated(ref eaState, extAccelId, tGE: tGE, dt: dtGE, ref gePhysJob);
                    break;

                // Add user forces here
                default:
                    // Add an error event
                    status = STATUS_NO_FORCE;
                    break;
            }
            return (status, r, v);
        }

        public static void UpdateShadow(double3 r, double3 v, double tSec, AccelType accelType, int paramBase, NativeArray<double3> data)
        {
            switch (accelType) {
                case AccelType.EARTH_ATMOSPHERE_REENTRY:
                    EarthAtmosphereReentry.UpdateShadow(r, v, tSec, paramBase, data);
                    break;
            }
        }

        public static (bool remove, bool stopSelfIntegrating) SelfIntegratingStatusActions(int status, AccelType accelType)
        {
            bool remove = false;
            bool stopSelfIntegrating = false;
            switch (accelType) {
                case AccelType.BOOSTER:
                    (remove, stopSelfIntegrating) = Booster.SelfIntegratingStatusActions(status);
                    break;
                case AccelType.EARTH_ATMOSPHERE_REENTRY:
                    (remove, stopSelfIntegrating) = EarthAtmosphereReentry.SelfIntegratingStatusActions(status);
                    break;
            }
            return (remove, stopSelfIntegrating);
        }

        /// <summary>
		/// Optional delegate to handle interactions between External Forces. Triggered by a returning
		/// value from an EAFn call. 
		///
		/// One example is staging. Stage 0 can hold a reference to the ExtId of stage 1
		/// and can signal that it has finished and it's time to activate the next stage.
		///
		/// This needs to be down in the core, so that a job-mode full launch sim can run
		/// through without needing intervention from GECore/GSController.
		/// 
		/// </summary>
		/// <param name="eaDescs"></param>
		/// <param name="eaData"></param>
		/// <returns></returns>
        public delegate bool EAInteraction(int index, ref EADesc eaDescs, NativeArray<double3> eaData);

    }
}
