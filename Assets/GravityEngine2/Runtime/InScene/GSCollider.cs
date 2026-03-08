using UnityEngine;

namespace GravityEngine2 {
    /// <summary>
    /// In Scene representation of a collider attached to a GSBody.
	///
	/// The radius of the body is taken from the corresponding GSBody
	/// optional physical info. 
    ///
    /// Collision detection is done in the Gravity Engine. 
    /// 
    /// </summary>
    public class GSCollider : MonoBehaviour {
        [Header("Requires a GSBody on same Game Object with a radius specified.")]
        public GEPhysicsCore.CollisionType collisionType;
        // TODO: Need to show units or allow them to be chosen
        public double bounceFactor = 1.0;

        public bool useGsbodyMass = true;

        [Header("Inertial mass (used for massless body bounces)")]
        public double inertialMass = 1.0;
    }
}
