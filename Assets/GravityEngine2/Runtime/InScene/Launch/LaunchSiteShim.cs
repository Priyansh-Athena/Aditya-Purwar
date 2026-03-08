using UnityEngine;

namespace GravityEngine2 {

    /// <summary>
    /// A kinda kludgy class to position a transform at the true physical surface of a planet
    /// and orient the indicated display axis model to be normal to the planets surface. 
    /// 
    /// Generally the launch site models will be children of this and get their position
    /// and orientation from here.
    /// 
    /// This is used because a mesh sphere will (mostly) have locations where the triangle surfaces are
    /// not the correct radius away from the center.
    /// 
    /// Assumes that this object will be in the heirarchy of a GSDisplay
    /// 
    /// </summary>
    public class LaunchSiteShim : MonoBehaviour {

        public GSBody launchBody;

        public GameObject shipModel;

        public GSDisplay gsDisplay;

        public GravityMath.AxisV3 alignAxis;

        public float alignYawDeg = 0.0f;

        private float size = 0.0f;

        // Start is called before the first frame update
        void Awake()
        {
            TransformUpdate(gsDisplay);
        }

        private void TransformUpdate(GSDisplay gsd)
        {
            if (launchBody.bodyInitData.initData == BodyInitData.InitDataType.LATLONG_POS) {
                if (launchBody.centerBody != null && launchBody.centerBody.radius > 0.0) {
                    Vector3 r = GravityMath.Double3ToVector3(
                                    GravityMath.LatLongToR(launchBody.bodyInitData.latitude,
                                                   launchBody.bodyInitData.longitude,
                                                   launchBody.centerBody.radius)
                            );
                    // assume r is in world units
                    // TODO: center body position
                    r *= gsd.scale;
                    if (gsd.xzOrbitPlane)
                        GravityMath.Vector3ExchangeYZ(ref r);
                    // adjust shim down by the size of the payload (assumes gsbody is located at top of rocket)
                    if (shipModel != null && size <= 0) {
                        MeshRenderer[] renderers = shipModel.GetComponentsInChildren<MeshRenderer>();
                        size = 0.0f;
                        foreach (Renderer renderer in renderers)
                            size += renderer.bounds.size.magnitude;
                        Debug.Log("Size=" + size);
                    }
                    transform.position = r - r.normalized * size;
                    Vector3 alignAxisVec = GravityMath.Axis(alignAxis);
                    transform.rotation = Quaternion.AngleAxis(alignYawDeg, alignAxisVec) *
                        Quaternion.FromToRotation(alignAxisVec, r);
                }
            }
        }

#if UNITY_EDITOR
        public void EditorUpdate()
        {
            GSDisplay gsd = GSCommon.FindGSDisplayAbove(transform);
            TransformUpdate(gsd);
        }
#endif
    }
}
