using UnityEngine;

namespace GravityEngine2 {
    /// <summary>
    /// The Solar System has a GSController for the motion of planets around the sun and separate controllers
    /// for any planets with satellites. Due to the difference length and mass scales these controllers
    /// will have different internal integration times.
    ///
    /// At any given time one of these controllers will be the primary controller. It's time evolution
    /// will set the time evolution in all the other systems.
	///
	/// For brevity, this controller also handles the switching of the display when a new primary system is
	/// selected. This is done by:
	/// - deactivating the display of objects in the newly inactive system
	/// - enabling display of the objects in the active system
	/// - changing the GE camera config to a display scale suitable distance from the origin
    ///
    /// There may be cases where there is no need to update the evolution in all the secondary systems
    /// e.g. if they are not in the display and they are all "on-rails" and can be jumped to a new
    /// time easily. When this is not the case (off-rails) it is best to evolve all the secondary controllers
    /// as the primary progresses to avoid a big "catch-up" computation when a new primary is selected.
    ///
    /// On Awake the start time from the Solar System time will be used (and pushed to all the planet
    /// controllers).
	/// 
    /// </summary>
    public class SolarMetaController : MonoBehaviour {
        [Header("Start time for ALL Controller\n(Set on Awake)")]
        public int year = 2024;
        public int month = 1;   // 1-12
        public int day = 1;     // 1+

        [Header("Evolve Mode for all Controllers")]
        GSController.EvolveMode evolveMode;

        [Header("Switch Camera and Primary Controller\nPlanets F1-F8, Sun=F10")]
        public GESphereCamera geCamera;

        [Header("Text UI to Show Wolrd Time")]
        public TMPro.TextMeshProUGUI timeText;

        [Header("Enable debug keys on primary controller")]
        public bool debugKeys = true;


        // for now manually create 8 of these
        [Header("Controllers and Targets filled in by SSB")]
        public GSController[] controllers = new GSController[SolarSystemBuilder.NUM_PLANETS + 1];

        public GameObject[] targets = new GameObject[SolarSystemBuilder.NUM_PLANETS + 1];

        private GSDisplay[] displays = new GSDisplay[SolarSystemBuilder.NUM_PLANETS + 1];

        private GSController solarController;

        private int primary = 0;

        // Real radii results in planet/satellite sizes that are not visible on a scale where the
        // orbits are visible. When set, the sphere radii will be adjusted based on the initial
        // camera boom position.
        public bool zoomRadiiForVisibility = true;

        // run all controllers, setting world time from the primary
        public bool runAllControllers = true;

        // F1-F10 are planets 1-8 and F10=sun

        void Awake()
        {
            // set all times in Awake (controllers will use this in Start)
            for (int i = 0; i < controllers.Length; i++) {
                if (controllers[i] != null) {
                    controllers[i].evolveMode = evolveMode;
                }
            }
            SetDateTime(year, month, day, utc: 0.0);

            // Auto fill timeText if not set
            // if (timeText == null) {
            //     TMPro.TextMeshProUGUI text = FindFirstObjectByType<TMPro.TextMeshProUGUI>();
            //     timeText = text;
            // }
        }

        // Controllers may not have run their Start yet (and inited displays etc.) so need to do stuff as a callback
        void Start()
        {
            solarController = controllers[0];
            solarController.ControllerStartedCallbackAdd(SetupPrimary);
            // ensure solar is running the show to start with
            for (int i = 1; i < controllers.Length; i++) {
                if (controllers[i] != null) {
                    controllers[i].ControllerStartedCallbackAdd(SetupSecondary);
                }
            }

        }

        private void SetupPrimary(GSController gsc)
        {
            displays[0] = solarController.Displays()[0];
            displays[0].ActiveSet(true);
            gsc.debugKeys = debugKeys;
            gsc.PrimaryControllerSet(null);

            SwitchPrimary(primary);
        }


        private void SetupSecondary(GSController gsc)
        {
            gsc.debugKeys = false;
            gsc.PrimaryControllerSet(solarController);
            gsc.Displays()[0].ActiveSet(false);
            // awkward. Need to recover index for display
            for (int i = 1; i < controllers.Length; i++) {
                if (gsc == controllers[i]) {
                    displays[i] = gsc.Displays()[0];
                    break;
                }
            }
        }

        /// <summary>
        /// Switch the primary controller to the controller for the indicated bodyCode
        /// 0=sun, 1=mercury etc.
        /// </summary>
        /// <param name="bodyCode"></param>
        public void SwitchPrimary(int bodyCode)
        {
            int newPrimary = bodyCode;
            if (controllers[newPrimary] == null) {
                Debug.LogWarning("No controller for bodyCode " + bodyCode);
                return;
            }
            if (runAllControllers) {
                for (int i = 0; i < controllers.Length; i++) {
                    if ((i != newPrimary) && (controllers[i] != null)) {
                        controllers[i].PrimaryControllerSet(controllers[newPrimary]);
                        if (debugKeys)
                            controllers[i].debugKeys = false;
                    }
                }
                controllers[newPrimary].PrimaryControllerSet(null);
                if (debugKeys)
                    controllers[newPrimary].debugKeys = true;
            } else {
                // get time from the current primary and set it in new primary
                controllers[newPrimary].TimeWorldSet(controllers[primary].WorldTime());
                controllers[primary].PausedSet(true);
                controllers[newPrimary].PausedSet(false);
            }
            displays[primary].ActiveSet(false);
            displays[newPrimary].ActiveSet(true);
            primary = newPrimary;
            SetCameraForPrimaryDisplay();
            // scale satellites, but not the primary
            if (zoomRadiiForVisibility) {
                float scaleUp = 5f;
                if (primary == 0)
                    scaleUp = 1000f; // can't scale purely based on radius, need a min sphere size
                SphereCollider[] spheres = displays[primary].GetComponentsInChildren<SphereCollider>();
                foreach (SphereCollider s in spheres) {
                    GSBody body = s.transform.parent.GetComponent<GSBody>();
                    if (body != null && body.propagator != GEPhysicsCore.Propagator.FIXED) {
                        // found the sphere that is a direct child of this body
                        float diam = (float)(2.0 * body.radius * displays[primary].scale * scaleUp);
                        s.transform.localScale = new Vector3(diam, diam, diam);
                        s.gameObject.name = body.gameObject.name + "Sphere";
                    }
                }
            }
        }

        private void SetCameraForPrimaryDisplay()
        {
            Debug.Log("Set camera for " + primary);
            // set the boom length to the recommended maxDistance x 1.5 in scaled units
            geCamera.gsDisplay = displays[primary];
            geCamera.BoomLengthReset(displays[primary].maxSceneDimension * 1.5f);
        }

        public void SetDateTime(int year, int month, int day, double utc)
        {
            for (int i = 0; i < controllers.Length; i++) {
                if (controllers[i] != null) {
                    controllers[i].year = year;
                    controllers[i].month = month;
                    controllers[i].day = day;
                    controllers[i].utc = utc;
                }
            }
            this.year = year;
            this.month = month;
            this.day = day;
        }

        // Update is called once per frame
        void Update()
        {
            // User input to switch the controller
            if (debugKeys) {
                for (int i = 0; i < 10; i++) {
                    if (Input.GetKeyDown(KeyCode.F1 + i)) {
                        // F10 is the sun, code 0
                        int code = (i == 9) ? 0 : i + 1;
                        if (code != primary)
                            SwitchPrimary(code);
                        break;
                    }
                }
            }
            if (timeText != null) {
                timeText.SetText(TimeUtils.JDTimeFormattedYD(controllers[primary].JDTime()));
                //Debug.Log(timeText.text);
            }
        }
    }
}
