using UnityEngine;
using Unity.Mathematics;
using UnityEngine.UI;

namespace GravityEngine2 {
    /// <summary>
    /// Use sliders to allow runtime setting of orbit elements.
    /// 
    /// </summary>
    public class InteractiveOrbitController : MonoBehaviour {
        [Header("P: Pause G:Go")]
        [Header("Use sliders when paused to adjust orbit")]
        public GSController gsController;

        [Header("DisplayOrbit to show preview when paused")]
        public GSDisplayOrbit orbitPreview;

        // List of sliders in order a, e, i, oU, oL, nu
        public Slider[] paramSliders;

        //! Required component
        private GSDisplayBody displayBody;

        //! id for the GSBody of the ship
        private int bodyId;

        int centerId;

        double centerMu;

        private GECore ge;

        private GEBodyState pausedState;

        private bool paused;

        private float nuSliderValue;


        // Start is called before the first frame update
        void Start()
        {
            displayBody = GetComponent<GSDisplayBody>();
            if (displayBody == null) {
                Debug.LogError("Script must be attached to a GSDisplayBody: " + gameObject.name);
            }
            gsController.ControllerStartedCallbackAdd(GEStart);

            for (int i = 0; i < paramSliders.Length; i++) {
                paramSliders[i].onValueChanged.AddListener(SliderChanged);
            }
            nuSliderValue = paramSliders[5].value;

        }

        // common callback that reads all the sliders and sets a COE. Lazy, but simple. 
        private void SliderChanged(float value)
        {
            if (!paused) {
                Debug.LogWarning("Pause (press P) before setting orbit with sliders");
                return;
            }
            Orbital.COE coe = new Orbital.COE();
            coe.EllipseInitDegrees(centerMu,
                        a: paramSliders[0].value,
                        e: paramSliders[1].value,
                        i: paramSliders[2].value,
                        omegaL: paramSliders[3].value,
                        omegaU: paramSliders[4].value,
                        nu: paramSliders[5].value
                       );
            // unless phase slider was changed try to keep ship near last position
            if (nuSliderValue == paramSliders[5].value) {
                coe.nu = Orbital.PhaseAngleRadiansForDirection(pausedState.r, coe);
            }
            nuSliderValue = paramSliders[5].value;
            (double3 r, double3 v) = Orbital.COEtoRVRelative(coe);
            orbitPreview.RVRelativeSet(r, v);
            // move the ship to the new R, V since it is no longer on the orbit
            pausedState.r = r;
            pausedState.v = v;
            ge.StateSetById(bodyId, pausedState);
        }

        public void GEStart(GSController gsc)
        {
            ge = gsc.GECore();

            // turn off debug keys in GSC (conflict possible)
            if (gsController.debugKeys) {
                gsController.debugKeys = false;
                Debug.LogWarning("Turned off GSController debug keys to avoid conflict");
            }
            centerId = orbitPreview.centerDisplayBody.gsBody.Id();
            centerMu = ge.MuWorld(centerId);
            bodyId = displayBody.gsBody.Id();
        }

        // Update is called once per frame
        void Update()
        {
            if (Input.GetKeyDown(KeyCode.P)) {
                // pause and get ship state
                gsController.PausedSet(true);
                // turn on display orbit
                orbitPreview.gameObject.SetActive(true);
                System.Collections.Generic.List<GSDisplay> displays = gsController.Displays();
                foreach (GSDisplay disp in displays) {
                    disp.DisplayObjectAdd(orbitPreview);
                }
                paused = true;
                // Update sliders with current COE using SetValueWithoutNotify
                Orbital.COE coe = ge.COE(bodyId, centerId);
                paramSliders[0].SetValueWithoutNotify((float)coe.a);
                paramSliders[1].SetValueWithoutNotify((float)coe.e);
                paramSliders[2].SetValueWithoutNotify((float)(coe.i * Mathf.Rad2Deg));
                paramSliders[3].SetValueWithoutNotify((float)(coe.omegaL * Mathf.Rad2Deg));
                paramSliders[4].SetValueWithoutNotify((float)(coe.omegaU * Mathf.Rad2Deg));
                paramSliders[5].SetValueWithoutNotify((float)(coe.nu * Mathf.Rad2Deg));
                SliderChanged(0);
            } else if (Input.GetKeyDown(KeyCode.G)) {
                // go: commit r,v change and unpause
                gsController.PausedSet(false);
                orbitPreview.gameObject.SetActive(false);
                orbitPreview.enabled = false;
                ge.StateSetById(bodyId, pausedState);
            }
        }
    }
}
