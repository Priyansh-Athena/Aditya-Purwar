
using Unity.Mathematics;

namespace GravityEngine2 {
    // SET_VELOCITY: Use velocityParam as the new velocity vector
    // APPLY_DV: add velocityParam as dV vector to the current velocity
    // SCALAR_DV: Change the magnitude of the velocity to the new value, keep direction the same
    //            (use velocityParam.x as the magnitude of the new velocity)
    //   e.g. to increase by 1% set velocityParm.x to 1.01
    public enum ManeuverType { SET_VELOCITY, APPLY_DV, SCALAR_DV, /* SCALAR_DV_RELATIVE */ };

    public delegate void DoneCallback(GEManeuver gem, GEPhysicsCore.PhysEvent physEvent);

    /// <summary>
    /// GEmaneuver is the CLASS that represents a maneuver while it is being created and planned. 
    /// As a class it is easy to adjust member variables during creation. It5 also holds a reference to the 
    /// body to be applied (and optionally center it is relative to) by id
    ///
    /// GE expects the quantities in GEManeuver to be in the default world units it was configured for. 
    /// 
    /// Once added to GE it needs to be a type that can be handled in an IJob so GE converts it into 
    /// a GEManueverStruct. This struct also refences the body and center by type and index. 
    /// 
    /// </summary>
    public class GEManeuver {
        public int uniqueId; // assigned by GE when added. This is not the body ID

        public ManeuverType type;

        //! relative time of maneuver at time of planning. Used during maneuver sequence construction.
        public double t_relative;

        //! velocity vector. Usage depnds on the type of the manuever. In some case only the first entry in the vector is used.
        public double3 velocityParam;

        // velocity change in world units for the maneuver. This is typically used when the maneuvers are generated
        // by TransferShip (Hohmann/Lambert) to indicate the size and direction of each maneuver required, for estimation
        // of the feasability of the maneuver given the ships dV allowance.
        public double3 dV;
        //! descriptive enum to indicate how maneuver was generated (strings are not allowed in IJob code)
        public ManeuverInfo info;

        // CenterId (optional)
        //
        // -1 if not in use (manuever is not relative to a center)
        public int centerId;

        // Maneuvers (esp. transfers) can optionally provide the actual post maneuver R, V relative to a center body
        // this allows GE to interpret these maneuvers as a new patch when the body is configured to prop as a series
        // of propagator patches. This allows time reversability & drag forward in time.
        //
        // If not provided, then at the time of the maneuver the prop will be updated with the
        // new post-maneuver state. 
        public bool hasRelativeRV;
        public double3 r_relative;
        public double3 v_relative;
        public GEPhysicsCore.Propagator prop = GEPhysicsCore.Propagator.UNASSIGNED;

        //! called after the timeslice in which the maneuver was executed. NOT *at* the exact time!
        public DoneCallback doneCallback;
        //! OPaque data to be used by callback
        public object opaqueData;

        public GEManeuver()
        {
            centerId = -1;
            prop = GEPhysicsCore.Propagator.UNASSIGNED;
        }

        public GEManeuver(double t, double3 v, ManeuverType type)
        {
            this.t_relative = t;
            velocityParam = v;
            this.type = type;
            dV = double3.zero;
            info = ManeuverInfo.USER;
            centerId = -1;
            prop = GEPhysicsCore.Propagator.UNASSIGNED;
        }

        public string LogString()
        {
            return string.Format("{0} t_rel={1} mode={2} vel={3} dV={4} centerId={5} ",
                info, t_relative, type, velocityParam, dV, centerId);
        }

        public double DvMagnitude()
        {
            return math.length(dV);
        }


    }

    /// <summary>
    /// It's useful to add info to a maneuver for debug that shows how the maneuver was generated (transfers etc.)
    /// Strings cannot be put into NativeArrays, so resort to tokens that can be decoded. 
    /// </summary>
    public enum ManeuverInfo {
        USER, // entry 0
        HG_IO_PHASE,  // Hohmann General Inner to Outer, Phase burn
        HG_IO_X1,
        HG_IO_X2,
        HG_OI_PHASE,  // Hohmann General Inner to Outer, Phase burn
        HG_OI_X1,
        HG_OI_X2,
        HG_CHV_1,
        HG_CHV_2,
        HG_SOP_1,
        HG_SOP_2,
        HG_CIAN_1,
        // Lambert
        LAM_1,
        LAM_2,
        LBAT_1,
        LBAT_2,
        INTERCEPT
    };

    /// <summary>
    /// Struct of types that are Burst/Job compatible to describe maneuvers for the IJob propagators.
    /// 
    /// </summary>
    public struct GEManeuverStruct {
        public int uniqueId; // assigned by GE to map back to GEManeuver object
        public ManeuverType type;

        //! the index into the internal integrator reference (coupled to bodyType)
        public int id;

        //! relative time of maneuver at time of planning. Used during maneuver sequence construction.
        public double t;

        //! velocity vector. Usage depnds on the type of the manuever. In some case only the first entry in the vector is used.
        public double3 velocityParam;

        //! descriptive enum to indicate how maneuver was generated (strings are not allowed in IJob code)
        public ManeuverInfo info;

        //! RelativeTo information. Vel change can be relative to some center body (which may be in motion)
        //! This is used to avoid keeping track of the overall motion of the center when e.g. doing an orbit
        //! transfer around it. 
        public int centerIndex;

        // status info: record the actual dV value (handy for unit tests on transfers)
        public double3 dVActual;

        // GEManeuver is in world units, need to convert to GE units in this struct
        public GEManeuverStruct(GEManeuver m,
                               int id,
                               GBUnits.GEScaler geScaler,
                               double timeOffsetGE = 0)
        {
            uniqueId = m.uniqueId;
            t = geScaler.ScaleTimeWorldToGE(m.t_relative) + timeOffsetGE;
            velocityParam = m.velocityParam * geScaler.ScaleVelocityWorldToGE(1.0);
            type = m.type;
            this.id = id;
            info = m.info;
            dVActual = double3.zero;
            this.centerIndex = m.centerId;
        }

        /// <summary>
        /// Generate a maneuver with a new body index. Used when there is a shuffle in an integrator array
        /// because cannot assign to a field in IJob struct.
        /// </summary>
        /// <param name="m"></param>
        /// <param name="newIndex"></param>
        public GEManeuverStruct(GEManeuverStruct m, int newIndex, int newCenterIndex)
        {
            uniqueId = m.uniqueId;
            t = m.t;
            velocityParam = m.velocityParam;
            type = m.type;
            id = newIndex;
            info = m.info;
            centerIndex = newCenterIndex;
            dVActual = m.dVActual;
        }


        public double3 ApplyManeuver(double3 v, double3 centerV)
        {
            double3 v_new = v - centerV;
            switch (type) {
                case ManeuverType.SET_VELOCITY:
                    v_new = velocityParam;
                    break;
                case ManeuverType.APPLY_DV:
                    v_new += velocityParam;
                    break;
                case ManeuverType.SCALAR_DV:
                    v_new = velocityParam.x * math.normalize(v) + v;
                    break;
                    //case ManeuverType.SCALAR_DV_RELATIVE:
                    //    v_new = velocityParam.x * v;
                    //    break;
            }
            return v_new + centerV;
        }


        public string LogString()
        {
            return string.Format("{0} t_rel={1} mode={2} vel={3}  index={4}",
                info, t, type, velocityParam, id);
        }
    }

}
