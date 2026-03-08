using UnityEngine;
using Unity.Mathematics;

namespace GravityEngine2 {
    /// <summary>
    /// Component to specify that two GSBody objects are in orbit around their mutual 
    /// center of mass (CM). Use when the mass ratio of the two bodies m1/m2, m1 < m2 is
    /// non negligible. e.g. the Earth/Moon system. 
    /// 
    /// The center of mass can be optionally added as GSBody. This is intended for cases where
    /// the propagation of the binary bodies is KEPLER mode, in which case they need a center object that
    /// is fixed or is itslef in KEPLER mode.
    /// 
    /// More commonly the bodies will be given the necessary binary orbit (r, v) intial conditions
    /// added to the binary CM initial conditions (which could be as RV or an orbit COE). 
    /// 
    /// If the CM is added and a GRAVITY propagator is used, the position of the CM will be
    /// computed based on the positions of the binary bodies. This allows it's use as the center
    /// of a component to display the orbit wrt the CM.
    /// 
    /// The binary orbit initial conditions must be specified using a COE. The physical size of
    /// the orbit is between the bodies.
    /// 
    /// </summary>
    public class GSBinary : GSBody {

        // CM initial data (R, V, coe, TLE etc.) are handled by parent bodyInitData if addCM is true.
        // Cannot be TLE or LAT_LONG
        public bool addCM;

        // propagator *if* the CM is added to GECore. Can only be GRAVITY or KEPLER.
        public GEPhysicsCore.Propagator cMPropagator = GEPhysicsCore.Propagator.GRAVITY;

        // reference to the two bodies in the binary. Needed to get their masses.
        // When set in the editor will Set() a field in the GSBody so it has a reference
        // to this GSBinary for init purposes.
        public GSBody body1, body2;

        public double mass1, mass2;

        public GEPhysicsCore.Propagator binaryPropagator;

        // Body around which CM will orbit. 
        public GSBody cMcenterBody;

        // binary orbit initial data 
        // Can only be COE or COE_ApoPeri. 
        public BodyInitData binaryInitData = new BodyInitData();

        private Orbital.COE oe;



        public Orbital.COE BinaryCOE()
        {
            // Always provide a fresh copy, since GSController may scale units etc. 
            oe = new Orbital.COE();
            binaryInitData.FillInCOE(oe);
            return oe;
        }

        /// <summary>
        /// (Internal Use)
        /// Fill in the BodyInitData for a member of the binary system. Expecting to be called
        /// as part of the GSController.BodyAdd() logic. 
        /// 
        /// If the body is part of a binary it needs to have it's gsBinary field set (the editor
        /// script will do this, if un-edited need a script to do this).
        /// 
        /// If the CM is being added, then it can be the center object provided it has been added
        /// first and the BID can be a copy of the binary BID (as COE) suitably scaled. 
        /// 
        /// If CM not added then need to compute the absolute R, V including the initial info from 
        /// the CM BodyInitData.
        /// 
        /// </summary>
        /// <param name="gsbody"></param>
        public void InitMember(GSBody gsbody, GEBodyState cmState)
        {
            if ((binaryPropagator == GEPhysicsCore.Propagator.KEPLER) && !addCM) {
                // Kepler will need an object to be the center of the binary pair
                Debug.LogWarning("forcing add of CM for Kepler binary");
                addCM = true;
            }

            UnityEngine.Debug.Log("init binary for " + gsbody.gameObject.name);
            bool primary = gsbody == body1;
            double m_total = mass1 + mass2;
            double mu1 = mass1 / m_total;
            double mu2 = mass2 / m_total;
            // Need to set the mass to the effective mass
            double mu_orbit;
            if (primary) {
                mu_orbit = mu2 * mu2 * mass2; // ??? From GE but seems odd
                gsbody.bodyInitData.nu = (binaryInitData.nu + 180.0) % 360.0;
                gsbody.bodyInitData.a = binaryInitData.a * mu2;
                gsbody.bodyInitData.p_semi = binaryInitData.p_semi * mu2;
                gsbody.mass = mass1;
            } else {
                mu_orbit = mu1 * mu1 * mass1;
                gsbody.bodyInitData.nu = binaryInitData.nu;
                gsbody.bodyInitData.a = binaryInitData.a * mu1;
                gsbody.bodyInitData.p_semi = binaryInitData.p_semi * mu1;
                gsbody.mass = mass2;
            }
            gsbody.bodyInitData.eccentricity = binaryInitData.eccentricity;
            gsbody.bodyInitData.omega_lc = binaryInitData.omega_lc;
            gsbody.bodyInitData.omega_uc = binaryInitData.omega_uc;
            gsbody.bodyInitData.inclination = binaryInitData.inclination;
            gsbody.bodyInitData.units = binaryInitData.units;

            if (binaryPropagator == GEPhysicsCore.Propagator.GRAVITY) {
                // need to init in RV mode
                Orbital.COE coe = new Orbital.COE();
                gsbody.bodyInitData.initData = BodyInitData.InitDataType.COE; // FillIn needs this
                gsbody.bodyInitData.FillInCOE(coe);
                coe.mu = mu_orbit * GBUnits.GForUnits(bodyInitData.units);
                (double3 r, double3 v) = Orbital.COEtoRVRelative(coe);
                gsbody.bodyInitData.initData = BodyInitData.InitDataType.RV_ABSOLUTE;
                gsbody.bodyInitData.r = r + cmState.r;
                gsbody.bodyInitData.v = v + cmState.v;
            } else {
                // Kepler mode
                gsbody.centerBody = this;
                gsbody.bodyInitData.initData = BodyInitData.InitDataType.COE;
            }
        }

    }
}
