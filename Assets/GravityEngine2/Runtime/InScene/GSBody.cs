using Unity.Mathematics;
using UnityEngine;


namespace GravityEngine2 {
    /// <summary>
    /// A component for the in-scene representation of a body to be evolved with GE via a GravitySceneController.
    ///
    /// The initial state of the body can be represented in a number of ways:
    /// 1) Absolute position/velocity in world space
    /// 2) Relative position/velocity with respect to another GSBody
    /// 3) In orbit relative to a center body. The orbit is specified via the "classical orbital elements (COE)"
    ///    or variations (where e.g. apoapsis and periapsis can be used instead of major axis and ecentricity.
    ///    Note apoapsis is the general form of apogee, which is for the Earth)
    /// 4) In orbit around the Earth via the Spacetrack two line element format. This is a commonly used general
    ///    format specifying the orbit parameters and epoch time. It can be retreived from e.g. Celestrak
    ///
    /// The GSBody component also allows the specification of how GE will propagate the body. The general case
    /// is to use N-body evolution (GRAVITY). Other choices are:
    ///   FIXED: don't move it at all (suitable for e.g. a central star)
    ///   KEPLER: Keep object "on-rails" around a center body and evolve with Kepler's equation
    ///   SGP4: Move according to a model that incorporates Earth oblateness, atmospheric drag, Moon, Sun.
    ///         This is only valid for a center body that is modelling the Earth.
    ///   PKEPLER: An Earth orbit model that incorporates non-spherical Earth only (e.g. for Sun synchronous
    ///      orbits. These rely on the non-spherical orbit perturbation. Only valid for Earth orbits.
    ///      
    /// </summary>
    public class GSBody : MonoBehaviour {
        public double mass;

        public bool optionalDataFoldout;
        // optional (in units specified in BID)
        public double radius;

        // JPL *might* provide an axis tilt and rotation rate. Unclear how to determine axis at time of ephem...
        public Vector3 rotationAxis = Vector3.up;
        public double rotationRate = 0.0;
        public double rotationPhi0 = 0.0;
        public GEPhysicsCore.Propagator propagator = GEPhysicsCore.Propagator.GRAVITY;

        public GEPhysicsCore.EarlyPropagation earlyPropagation = GEPhysicsCore.EarlyPropagation.ERROR_INACTIVATE;

        // Propagators that are deterministic/stateless can be be used in a time sequence
        // of patches. This facilitates jumping time forward/backwards IF all objects
        // in the GE are deterministic. 
        public bool patched;

        // Body around which this will orbit. 
        public GSBody centerBody;


        // initial data (R, V, coe, TLE etc.) are handled by this class. 
        public BodyInitData bodyInitData = new BodyInitData();

        public string ephemFilename;
        public bool ephemRelative;
        public EphemerisPropagator.Interpolate ephemInterpolate = EphemerisPropagator.Interpolate.LINEAR;
        public EphemerisData.FileFormat ephemFormat = EphemerisData.FileFormat.TRV;
        public GBUnits.Units ephemFileUnits = GBUnits.Units.SI_km;

        private Orbital.COE oe;

        // If part of a binary, the binary object will use BinarySet() to configure
        // a back-reference to the binary. The orbit configuration from GSBinary will
        // control the orbit initial data of this body and the mass configured in the
        // GSBinary will control the mass
        [SerializeField]
        private GSBinary gsBinary;

        //! GravityData id. Assigned when added to GE
        // MUST NOT BE IN INSPECTOR
        public static int NO_ID = -1;
        private int id = NO_ID;


        public int Id()
        {
            return id;
        }

        public void IdSet(int id)
        {
            this.id = id;
        }

        public Orbital.COE COEFromBID()
        {
            // Always provide a fresh copy, since GSController may scale units etc. 
            oe = new Orbital.COE();
            bodyInitData.FillInCOE(oe);
            return oe;
        }

        public GSBody CenterBody()
        {
            if (centerBody == null)
                centerBody = transform.parent.GetComponent<GSBody>();
            return centerBody;
        }

        public void BinarySet(GSBinary gsBinary)
        {
            this.gsBinary = gsBinary;
        }

        /// <summary>
        /// If this body is a member of a GSBinary then the GSBinary object will 
        /// have the details on the joint orbit and the mass fractions etc. 
        /// 
        /// </summary>
        public GSBinary Binary()
        {
            return gsBinary;
        }


#if UNITY_EDITOR
        /// <summary>
        /// Called from a GSBody if it is determined that body has an orbit
        /// </summary>
        /// <returns></returns>
        public Vector3 EditorPositionRelative()
        {
            Vector3 r_vec = GravityMath.Double3ToVector3(bodyInitData.r);
            (bool haveCOE, Orbital.COE coe) = EditorCOE();
            if (haveCOE) {
                (double3 r, double3 v) = Orbital.COEtoRVRelative(coe);
                r_vec = GravityMath.Double3ToVector3(r);
            } else if (bodyInitData.initData == BodyInitData.InitDataType.LATLONG_POS) {
                // if we have a physical radius for the center body we can set position
                if (centerBody != null && centerBody.radius > 0.0) {
                    r_vec = GravityMath.Double3ToVector3(
                                GravityMath.LatLongToR(bodyInitData.latitude,
                                                   bodyInitData.longitude,
                                                   centerBody.radius)
                            );
                }
            }
            return r_vec;
        }

        public bool EditorNeedsCenter()
        {
            // only case where do not need a center is RV_ABSOLUTE with GRAVITY PROP or
            // FIXED prop
            bool needCenter = true;
            if (propagator == GEPhysicsCore.Propagator.EPHEMERIS)
                needCenter = ephemRelative;
            else if (bodyInitData.initData == BodyInitData.InitDataType.RV_ABSOLUTE)
                needCenter = false;


            return needCenter;
        }

        /// <summary>
        /// Fill in a COE struct based on the editor parameter mode. Used by Gizmo orbit code.
        /// </summary>
        /// <returns></returns>
        public (bool haveCoe, Orbital.COE coe) EditorCOE()
        {
            BodyInitData.InitDataType initData = bodyInitData.initData;
            bool haveCoe = (initData == BodyInitData.InitDataType.COE) ||
                           (initData == BodyInitData.InitDataType.COE_ApoPeri) ||
                           (initData == BodyInitData.InitDataType.COE_HYPERBOLA) ||
                           (initData == BodyInitData.InitDataType.TWO_LINE_ELEMENT);
            Orbital.COE coe = new Orbital.COE();
            if (haveCoe) {
                bodyInitData.FillInCOE(coe);
            } else if (initData == BodyInitData.InitDataType.RV_RELATIVE && centerBody != null) {
                // can determine COE unless radial infall
                if (math.length(math.cross(bodyInitData.r, bodyInitData.v)) > 1E-6) {
                    double g = GBUnits.GForUnits(bodyInitData.units);
                    coe = Orbital.RVtoCOE(bodyInitData.r, bodyInitData.v, g * centerBody.mass);
                    haveCoe = true;
                }
            }
            return (haveCoe, coe);
        }

#endif
        override
        public string ToString()
        {
            return string.Format("id={0} r={1} v={2} name={3}", id, bodyInitData.r, bodyInitData.v, gameObject.name);
        }

    }
}
