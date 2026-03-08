using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace GravityEngine2 {
    public class DynamicAddBody : MonoBehaviour {
        public GSController gsController;

        public GSDisplay gsDisplay;

        public GSBody centerBody;

        public int prefabIndex;

        public GameObject[] prefabs;

        private GameObject prefabToAdd;

        [Header("A add, S to Remove")]
        public bool enableKeys;

        public bool autoAdd;
        public float addRatePerSec = 2.0f;

        private float timeToAdd = 0;

        private List<GSBody> bodiesAdded;

        private double SIZE_FROM = 10.0;
        private double SIZE_TO = 40.0;
        private double orbitSize = 10.0f;

        // Start is called before the first frame update
        void Start()
        {
            bodiesAdded = new List<GSBody>();
            prefabToAdd = prefabs[prefabIndex];
        }

        /// <summary>
        /// Overkill, but to illustrate how params are added to a GE callback pass the
        /// orbit shape and size through GE and back into the callback.
        /// </summary>
        public struct OrbitShapeSize {
            public double a;
            public double e;

            public OrbitShapeSize(double a, double e)
            {
                this.a = a;
                this.e = e;
            }
        }

        private void AddBody(GECore ge, object arg)
        {
            GameObject go = Instantiate(prefabToAdd);
            OrbitShapeSize oss = (OrbitShapeSize)arg;
            // want to put body in orbit around star
            GSBody gsBody = go.GetComponent<GSBody>();
            gsBody.centerBody = centerBody;
            gsBody.bodyInitData.initData = BodyInitData.InitDataType.COE;
            gsBody.bodyInitData.a = oss.a;
            gsBody.bodyInitData.eccentricity = oss.e;
            gsBody.centerBody = centerBody;
            gsBody.propagator = GEPhysicsCore.Propagator.GRAVITY;
            gsBody.mass = 0;
            gsController.BodyAdd(gsBody);

            if (gsDisplay != null) {
                // could have both a display object and a display orbit
                GSDisplayObject[] gdos = go.GetComponents<GSDisplayObject>();
                foreach (GSDisplayObject gdo in gdos) {
                    if (gdo is GSDisplayOrbit) {
                        GSDisplayOrbit gsdOrbit = (GSDisplayOrbit)gdo;
                        gsdOrbit.centerDisplayBody = centerBody.GetComponent<GSDisplayBody>();
                    }
                    gsDisplay.DisplayObjectAdd(gdo);
                    if (gdo is GSDisplayBody) {
                        GSDisplayBody gdb = (GSDisplayBody)gdo;
                        if (gdb.showTrajectory) {
                            gsController.TrajectoryAdd(gdb.gsBody.Id());
                        }
                    }
                }
            }
            bodiesAdded.Add(gsBody);
        }

        public void RemoveBody(GECore ge, object gsBodyObj)
        {
            GSBody gsBody = (GSBody)gsBodyObj;
            gsController.BodyRemove(gsBody, removeFromDisplays: true);
            Destroy(gsBody.gameObject);
        }

        private int n = 0;
        // Update is called once per frame
        void Update()
        {
            if (enableKeys) {
                if (Input.GetKeyDown(KeyCode.A)) {
                    Debug.Log("Add for keypress");
                    OrbitShapeSize oss = new OrbitShapeSize(orbitSize,
                                                             Random.Range(0.0f, 0.3f));
                    gsController.GECore().PhyLoopCompleteCallbackAdd(AddBody, oss);
                    orbitSize += 1.0;
                    if (orbitSize > SIZE_TO)
                        orbitSize = SIZE_FROM;
                }
                if (Input.GetKeyDown(KeyCode.S)) {
                    Debug.Log("Remove for keypress");
                    if (bodiesAdded.Count > 0) {
                        gsController.GECore().PhyLoopCompleteCallbackAdd(RemoveBody, bodiesAdded[0]);
                        bodiesAdded.RemoveAt(0);
                    }
                }
            }
            if (autoAdd) {
                if (Time.time > timeToAdd) {
                    OrbitShapeSize oss = new OrbitShapeSize(orbitSize,
                                                             Random.Range(0.0f, 0.3f));
                    gsController.GECore().PhyLoopCompleteCallbackAdd(AddBody, oss);
                    orbitSize += 1.0;
                    if (orbitSize > SIZE_TO)
                        orbitSize = SIZE_FROM;
                    timeToAdd = Time.time + 1.0f / addRatePerSec;
                    Debug.Log("Num=" + n++);
                }
            }
        }
    }
}
