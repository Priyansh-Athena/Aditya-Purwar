using UnityEngine.Networking;
using System.Collections;
using UnityEngine;
using System.Collections.Generic;

namespace GravityEngine2 {
    /// <summary>
    /// SolarSystemBuilder is an Editor feature used to create a Solar System prior to
    /// running a scene. This class is just a struct like anchor for the editor operations.
    ///
    /// It should not be run during execution of a scene (better to inactivate or remove it after
    /// creating the system). Running it should be harmless but has not been tested. 
    /// </summary>
    public class SolarSystemBuilder : MonoBehaviour {
        public GameObject sunPrefab;
        public GameObject planetPrefab;
        public GameObject satellitePrefab;
        public GameObject smallBodyPrefab;

        // Date for which starting position will be determined
        public int year = 2024;
        public int month = 1;   // 1-12
        public int day = 1;     // 1+

        public static int NUM_PLANETS = 8;  // breaks my heart....
        protected string[] planetNames = new string[]
            { "Mercury", "Venus", "Earth", "Mars", "Jupiter", "Saturn", "Uranus", "Neptune"};
        public bool[] planets = new bool[NUM_PLANETS];

        protected int[] maxSatellites = new int[] { 0, 0, 1, 2, 95, 146, 27, 14 };

        public int[] satellites = new int[NUM_PLANETS];

        public string[] smallBodies;

        private double startEpochJD;

        private double MIN_RADIUS_KM = 20.0;
        private float MAX_SCENE_DIM = 10000f;
        private float GAMESEC_PER_ORBIT = 120f;

        /// <summary>
        /// Container class to store the relevant info to make a Horizons Web API request.
        ///
        /// Also contains one or two GSBody references to be populated with the response.
        /// </summary>
        public class COERequest {
            public enum BodyType { PLANET, SATELLITE, SMALL };

            public int planet;
            public int satellite;
            public string smallBody;
            // if a small body gets an multiple bodies response, then it's assumed a comet with
            // a choice of ephem data based on epoch. 
            public bool comet;
            public BodyType bodyType;
            public GSBody body1;
            public GSBody body2;
        }

        /// <summary>
        /// Horizons will 503 if there are too many outstanding reqs at once, so limit to one at a time, since
        /// scene building is not a "game time" activity.
        ///
        /// </summary>
        /// <param name="requests"></param>
        ///
        private int reqNum;
        private Queue<COERequest> requests;

        // will be created
        private SolarMetaController solarMeta;

        public void DoRequests()
        {
            // each request response will trigger the next request.
            COERequest request = requests.Dequeue();
            StartCoroutine(JPLRequest(request));
        }

        public void RequestsClear()
        {
            if (requests != null)
                requests.Clear();
        }

        public int[] MaxSatellites()
        {
            return maxSatellites;
        }

        public string[] PlanetNames()
        {
            return planetNames;
        }

        private string JPLReqUri(COERequest coeRequest)
        {
            // TODO: Handle wrap if day+1 goes past end of month
            int bodyNum = (coeRequest.planet + 1) * 100 + 99;
            string center = "sun";
            if (coeRequest.bodyType == COERequest.BodyType.SATELLITE) {
                bodyNum = (coeRequest.planet + 1) * 100 + (coeRequest.satellite + 1);
                center = string.Format("{0}", (coeRequest.planet + 1) * 100 + 99);
            }
            string bodyString = bodyNum.ToString();
            if (coeRequest.bodyType == COERequest.BodyType.SMALL) {
                bodyString = "DES=" + coeRequest.smallBody;
                if (coeRequest.comet)
                    bodyString += "%3BCAP%3BNOFRAG"; // SPKID and use closest Ephem in time
            }
            return string.Format("https://ssd.jpl.nasa.gov/api/horizons.api?format=text&COMMAND='{0}'&OBJ_DATA='YES'&MAKE_EPHEM='YES'&EPHEM_TYPE='ELEMENTS'&CENTER='{1}'&START_TIME='{2}-{3}-{4}'&STOP_TIME='{2}-{3}-{5}'&STEP_SIZE='1%20d'&QUANTITIES='1,9,20,23,24,29'",
                bodyString, center, year, month, day, day + 1);

        }

        public string JPLSmallBodyQuery(string smallBodyName)
        {
            // %3B is a semi-colon, needed for small bodies
            return string.Format("https://ssd.jpl.nasa.gov/api/horizons.api?format=text&COMMAND='{0}%3B'&OBJ_DATA='YES'",
                smallBodyName);
        }

        private IEnumerator JPLRequest(COERequest coeReq)
        {
            string uri = JPLReqUri(coeReq);
            using (UnityWebRequest webRequest = UnityWebRequest.Get(uri)) {
                Debug.LogFormat("send request {0}/{1}", reqNum++, requests.Count);
                // Request and wait for the desired page.
                yield return webRequest.SendWebRequest();

                string[] pages = uri.Split('/');
                int page = pages.Length - 1;

                switch (webRequest.result) {
                    case UnityWebRequest.Result.ConnectionError:
                    case UnityWebRequest.Result.DataProcessingError:
                        Debug.LogError(pages[page] + ": Error: " + webRequest.error);
                        break;
                    case UnityWebRequest.Result.ProtocolError:
                        Debug.LogError(pages[page] + ": HTTP Error: " + webRequest.error);
                        break;
                    case UnityWebRequest.Result.Success:
                        Debug.Log(pages[page] + ":\nReceived: " + webRequest.downloadHandler.text);
                        JPLResponseHandler(coeReq, webRequest.downloadHandler.text);
                        break;
                }
            }
        }

        private void JPLRequestsComplete()
        {
            Debug.Log("ALL REQUESTS COMPLETE");
            // Ask the controllers to determine the right scales
            for (int i = 0; i < NUM_PLANETS + 1; i++) {
                if (solarMeta.controllers[i] != null) {
                    ScaleSystem(solarMeta.controllers[i]);
                }
            }
        }

        private void ScaleSystem(GSController gsc)
        {
#if UNITY_EDITOR
            gsc.gameSecPerOrbit = GAMESEC_PER_ORBIT;
            gsc.EditorAutoscale();
            GSDisplay gsd = gsc.GetComponentInChildren<GSDisplay>();
            gsd.maxSceneDimension = MAX_SCENE_DIM;
            gsd.scale = gsd.EditorAutoScale();
            // objects may need to update their positions
            gsd.EditorUpdateAllBodies();
            // set the radii of all bodies that have spheres
            GSBody[] bodies = gsd.GetComponentsInChildren<GSBody>();
            foreach (GSBody b in bodies) {
                RadiusSet(b, gsd);
            }
#endif
        }

        private void JPLResponseHandler(COERequest coeReq, string jplData)
        {
            bool skipReponse = false;
            if (coeReq.bodyType == COERequest.BodyType.SMALL) {
                // check to see we got a real response. For comets may get a list of
                // epochs. To disambiguate need to re-request with CAP;NOFRAG
                if (jplData.Contains("Matching small-bodies")) {
                    coeReq.comet = true;
                    requests.Enqueue(coeReq);
                    skipReponse = true;
                }
            }
            if (!skipReponse) {
                // Fill in the GSBody info
                FillInBodyInfo(coeReq.body1, jplData);
                if (coeReq.body2 != null) {
                    GSBody body2 = coeReq.body2;
                    FillInBodyInfo(body2, jplData);
                    // body 2 (planet at center of new system) wants to be at (0,0,0)
                    // world space and fixed
                    body2.bodyInitData.initData = BodyInitData.InitDataType.RV_ABSOLUTE;
                    body2.bodyInitData.r = Unity.Mathematics.double3.zero;
                    body2.propagator = GEPhysicsCore.Propagator.FIXED;
                    body2.gameObject.name += "System";
                }
            }
            // Start next query
            if (requests.Count > 0) {
                COERequest request = requests.Dequeue();
                StartCoroutine(JPLRequest(request));
            } else {
                JPLRequestsComplete();
            }
        }

        private void FillInBodyInfo(GSBody body, string jplData)
        {
            body.bodyInitData = JPLHorizonTools.HorizonResponseToBID(jplData);
            if (body.bodyInitData == null) {
                Debug.LogError("could not fulfill request");
            }
            // set body start epoch
            body.bodyInitData.startEpochType = BodyInitData.StartEpochType.DMY_UTC;
            body.bodyInitData.startEpochYear = year;
            body.bodyInitData.startEpochMonth = month;
            body.bodyInitData.startEpochDay = day;
            body.bodyInitData.startEpochUtc = 0.0;
            body.bodyInitData.startEpochWorldTime = 0.0;
            body.gameObject.name = JPLHorizonTools.BodyName(jplData);
            body.mass = JPLHorizonTools.Mass(jplData);
            body.radius = JPLHorizonTools.Radius(jplData);
            // give everything a default radius
            if (body.radius == 0)
                body.radius = MIN_RADIUS_KM;
            body.propagator = GEPhysicsCore.Propagator.KEPLER_COE;
            body.earlyPropagation = GEPhysicsCore.EarlyPropagation.PROPAGATE;
            TMPro.TextMeshPro text = body.GetComponentInChildren<TMPro.TextMeshPro>();
            if (text != null) {
                text.text = body.gameObject.name;
            }
            // if there is rotation info use it
            body.rotationRate = JPLHorizonTools.RotationRate(jplData);
            if (body.rotationRate > 0) {
                float angle = (float)JPLHorizonTools.AxisTiltDegrees(jplData);
                // rotate Z-axis around X by the tilt. AFAICT JPL does not specify how to ensure correct tilt
                // at a given time but I think I just have not found it yet...
                body.rotationAxis = Quaternion.AngleAxis(angle, new Vector3(1, 0, 0)) * Vector3.forward; // 
            }
            // Now we have the official name, so if there is a texture, apply it
            MaterialSet(body.gameObject);
            Debug.Log("Process response for " + body.gameObject.name + " mass=" + body.mass);
        }

        private GameObject SunCreate()
        {
            GameObject sun = Instantiate(sunPrefab);
            GSBody sunBody = sun.GetComponent<GSBody>();
            sunBody.mass = GBUnits.SUN_MASS;
            sunBody.bodyInitData.units = GBUnits.Units.SI_km;
            sunBody.radius = GBUnits.SUN_RADIUS_KM;
            sunBody.bodyInitData.initData = BodyInitData.InitDataType.RV_ABSOLUTE;
            sunBody.propagator = GEPhysicsCore.Propagator.FIXED;
            sunBody.mass = GBUnits.SUN_MASS;

            sun.name = "Sun";
            // DisplayBody
            sun.AddComponent<GSDisplayBody>();
            return sun;
        }

        /// <summary>
        /// Based on the input variables, create all the game objects and heirarchy
        /// required for the solar system as configured and generate the JPL info
        /// requests
        /// </summary>
        public void BodiesAddToScene()
        {
            if (requests == null)
                requests = new Queue<COERequest>();

            if (requests.Count > 0) {
                Debug.LogError("Requests still pending. Try again once completed.");
                return;
            }
            startEpochJD = TimeUtils.JulianDate(year, month, day, utime: 0.0);
            // Controller
            (GameObject solarGC, GameObject solarGD) =
                GSTools.ControllerDisplayPair("SolarSystem", GBUnits.Units.SI_km);
            // Add the Sun
            GameObject sun = SunCreate();
            sun.transform.parent = solarGD.transform; requests = new Queue<COERequest>();
            // Add a SolarSystemController top the GSController
            solarMeta = solarGC.AddComponent<SolarMetaController>();
            solarMeta.controllers[0] = solarGC.GetComponent<GSController>();
            solarMeta.targets[0] = sun;
            solarMeta.SetDateTime(year, month, day, utc: 0.0);
            GSBody gsbSun = sun.GetComponent<GSBody>();
            // Setup GECamera on Sun (Meta controller will jumpt to other planets)
            GESphereCamera geCam = FindAnyObjectByType<GESphereCamera>();
            geCam.gsDisplay = solarGD.GetComponent<GSDisplay>();
            solarMeta.geCamera = geCam;

            for (int i = 0; i < NUM_PLANETS; i++) {
                if (planets[i]) {
                    // create the planet
                    GameObject planet = Instantiate(planetPrefab);
                    planet.name = planetNames[i];   // this will be stomped by response, but useful if there is none
                    planet.transform.parent = sun.transform;
                    GSBody gsbPlanet = planet.GetComponent<GSBody>();
                    gsbPlanet.centerBody = gsbSun;
                    // If there is a GSDisplayOrbit, set center to planet
                    GSDisplayOrbit gdo = planet.GetComponent<GSDisplayOrbit>();
                    if (gdo != null) {
                        gdo.centerDisplayBody = sun.GetComponent<GSDisplayBody>();
                    }
                    // build request for COE info
                    COERequest req = new COERequest();
                    req.planet = i;
                    req.bodyType = COERequest.BodyType.PLANET;
                    req.body1 = planet.GetComponent<GSBody>();
                    if (satellites[i] > 0) {
                        // Create an independant GSC/GSD to evolve the satellites as a sub-system
                        (GameObject planetGC, GameObject planetGD) =
                            GSTools.ControllerDisplayPair(planetNames[i] + "System", GBUnits.Units.SI_km);
                        GSController planetGCController = planetGC.GetComponent<GSController>();
                        planetGCController.year = year;
                        planetGCController.month = month;
                        planetGCController.day = day;
                        planetGCController.utc = 0.0;
                        GameObject planet2 = Instantiate(planetPrefab);
                        planet2.name = planetNames[i] + "System";
                        planet2.transform.parent = planetGD.transform;
                        req.body2 = planet2.GetComponent<GSBody>();
                        requests.Enqueue(req);
                        // meta
                        solarMeta.controllers[i + 1] = planetGC.GetComponent<GSController>();
                        solarMeta.targets[i + 1] = planet2;
                        GSBody gsbPlanet2 = planet2.GetComponent<GSBody>();

                        // don't need display orbit on clone of planet
                        GSDisplayOrbit cloneGdo = planet2.GetComponent<GSDisplayOrbit>();
                        if (cloneGdo != null)
                            DestroyImmediate(cloneGdo);

                        for (int j = 0; j < satellites[i]; j++) {
                            GameObject satellite = Instantiate(satellitePrefab);
                            satellite.transform.parent = planet2.transform;
                            COERequest reqSat = new COERequest();
                            reqSat.planet = i;
                            reqSat.satellite = j;
                            reqSat.bodyType = COERequest.BodyType.SATELLITE;
                            reqSat.body1 = satellite.GetComponent<GSBody>();
                            requests.Enqueue(reqSat);
                            // set body center
                            GSBody gsbSat = satellite.GetComponent<GSBody>();
                            gsbSat.centerBody = gsbPlanet2;
                            // If there is a GSDisplayOrbit, set center to planet
                            GSDisplayOrbit gdo2 = satellite.GetComponent<GSDisplayOrbit>();
                            if (gdo2 != null) {
                                gdo2.centerDisplayBody =
                                    planet2.GetComponent<GSDisplayBody>();
                            }
                        }
                    } else {
                        requests.Enqueue(req);
                    }
                }
            }
            // Small bodies
            foreach (string sb in smallBodies) {
                GameObject smallBody = Instantiate(smallBodyPrefab);
                smallBody.transform.parent = sun.transform;
                COERequest reqSB = new COERequest();
                reqSB.smallBody = sb;
                reqSB.bodyType = COERequest.BodyType.SMALL;
                reqSB.body1 = smallBody.GetComponent<GSBody>();
                requests.Enqueue(reqSB);
                // set body center
                GSBody gsbSat = smallBody.GetComponent<GSBody>();
                gsbSat.centerBody = gsbSun;
                // If there is a GSDisplayOrbit, set center to planet
                GSDisplayOrbit gdo2 = smallBody.GetComponent<GSDisplayOrbit>();
                if (gdo2 != null) {
                    gdo2.centerDisplayBody =
                        sun.GetComponent<GSDisplayBody>();
                }

            }
            DoRequests();
        }

        /// <summary>
        /// Set the radius of the sphere that is a direct child of the given gameobject
        /// </summary>
        /// <param name="body"></param>
        private void RadiusSet(GSBody body, GSDisplay gsd)
        {
            GSDisplayBody gdb = body.GetComponent<GSDisplayBody>();
            if (gdb != null && gdb.displayGO != null) {
                // found the sphere that is a direct child of this body
                float diam = (float)(2.0 * body.radius * gsd.scale);
                gdb.displayGO.transform.localScale = new Vector3(diam, diam, diam);
                gdb.displayGO.gameObject.name = body.gameObject.name + "Sphere";
                Debug.LogFormat("Set {0} to diam={1}", gdb.displayGO.transform.parent.gameObject.name, diam);
                return;
            }
        }

        // If there is texture resource with the body name, load it
        private void MaterialSet(GameObject go)
        {
            GSDisplayBody gdb = go.GetComponent<GSDisplayBody>();
            if (gdb != null && gdb.displayGO != null) {
                string filename = "Materials/" + go.name + "Mat";
                // Material names have "Mat post-pended
                Material mat = Resources.Load<Material>(filename);
                if (mat != null) {
                    Debug.Log("Loaded texture for " + filename);
                    Renderer r = gdb.displayGO.GetComponent<Renderer>();
                    r.material = mat;    // Jank: SetTexture didn't work.
                } else {
                    Debug.LogFormat("No material for :{0}:", filename);
                }
            }
        }



    }
}
