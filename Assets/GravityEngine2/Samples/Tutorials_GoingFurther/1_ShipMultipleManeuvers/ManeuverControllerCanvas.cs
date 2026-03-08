using UnityEngine;
using UnityEngine.UI;
using Unity.Mathematics;
using TMPro;
using System.Collections.Generic;
using System.Globalization;

namespace GravityEngine2 {
    /// <summary>
    /// Controller to add and adjust manual maneuvers.
    /// 
    /// UI allows for sliders or input fields to specify the dV at the maneuver point. 
    /// The ManeuverController component will do the orbit segment display. 
    /// </summary>
    public class ManeuverControllerCanvas : MonoBehaviour {

        public Slider sliderR;
        public Slider sliderS;
        public Slider sliderW;
        public Slider orbitPosSlider;

        public TMP_InputField inputR;
        public TMP_InputField inputS;
        public TMP_InputField inputW;
        public TMP_InputField orbitPos;

        public TMP_Dropdown orbitPointDropdown;

        public Button addManeuverButton;
        public Button delManeuverButton;

        // modal buttons
        public TMP_Dropdown maneuverPointDropdown;

        public Button executeButton;

        // half width of the slider is percent of velocity of the ship at the initial point
        public double velocityScalePercent = 70.0;

        public ManeuverController maneuverController;

        private int numManeuvers;

        // Start is called before the first frame update
        void Start()
        {
            ActivateCanvas();
        }

        // Update is called once per frame
        void Update()
        {

        }

        private void TextInput(TMPro.TMP_InputField iField)
        {
            double3 dV = maneuverController.Dv();
            double3 v = maneuverController.VelocityAtPoint();
            double vscale = velocityScalePercent / 100.0 * math.length(v);
            double result;
            if (double.TryParse(iField.text, NumberStyles.Any, CultureInfo.InvariantCulture, out result)) {
                if (iField == inputR) {
                    dV.x = vscale * sliderR.value / 100.0;
                    sliderR.SetValueWithoutNotify(100f * (float)(result / vscale));
                } else if (iField == inputS) {
                    dV.y = vscale * sliderS.value / 100.0;
                    sliderS.SetValueWithoutNotify(100f * (float)(result / vscale));
                } else if (iField == inputW) {
                    dV.z = vscale * sliderW.value / 100.0;
                    sliderW.SetValueWithoutNotify(100f * (float)(result / vscale));
                }
            }
            maneuverController.DvSet(dV);
            maneuverController.ManeuverChainUpdate();
        }

        /// <summary>
        /// Update the sliders and text fields that display the dV
        /// </summary>
        /// <param name="dV"></param>
        private void UpdateDvInfo(double3 dV)
        {
            // scale using ship velocity at start
            double3 v = maneuverController.VelocityAtPoint();
            double maxDv = velocityScalePercent / 100.0 * math.length(v);
            Vector3 dVScaled = new Vector3(100f * (float)(dV.x / maxDv),
                                        100f * (float)(dV.y / maxDv),
                                        100f * (float)(dV.z / maxDv));
            sliderR.value = dVScaled.x;
            sliderS.value = dVScaled.y;
            sliderW.value = dVScaled.z;
            inputR.SetTextWithoutNotify(string.Format("{0}", dV.x));
            inputS.SetTextWithoutNotify(string.Format("{0}", dV.y));
            inputW.SetTextWithoutNotify(string.Format("{0}", dV.z));
        }
        private void RSWSliderChanged(Slider slider)
        {
            if (numManeuvers == 0)
                return;

            double3 v = maneuverController.VelocityAtPoint();
            double maxDv = velocityScalePercent / 100.0 * math.length(v);

            double3 dV = maneuverController.Dv();
            if (slider == sliderR) {
                dV.x = maxDv * sliderR.value / 100.0;
                inputR.SetTextWithoutNotify(string.Format("{0:0.000}", dV.x));
            } else if (slider == sliderS) {
                dV.y = maxDv * sliderS.value / 100.0;
                inputS.SetTextWithoutNotify(string.Format("{0:0.000}", dV.y));
            } else if (slider == sliderW) {
                dV.z = maxDv * sliderW.value / 100.0;
                inputW.SetTextWithoutNotify(string.Format("{0:0.000}", dV.z));
            }
            maneuverController.DvSet(dV);
            maneuverController.ManeuverChainUpdate();
        }

        private void OrbitPositionSlider()
        {
            maneuverController.PositionSet(ManeuverController.PositionMode.PERIOD_PERCENT, orbitPosSlider.value / 100f);
            maneuverController.ManeuverChainUpdate();
            orbitPos.SetTextWithoutNotify(string.Format("{0}", orbitPosSlider.value));
            // reset any orbit point selected
            orbitPointDropdown.SetValueWithoutNotify(0);
        }

        private void OrbitPosInput()
        {
            float value = 0f;
            if (float.TryParse(orbitPos.text, NumberStyles.Any, CultureInfo.InvariantCulture, out value)) {
                maneuverController.PositionSet(ManeuverController.PositionMode.PERIOD_PERCENT, value / 100f);
                maneuverController.ManeuverChainUpdate();
                orbitPosSlider.SetValueWithoutNotify(value);
                // reset any orbit point selected
                orbitPointDropdown.SetValueWithoutNotify(0);
            }
        }

        private void OrbitPointDropdownChanged()
        {
            // options are in 1:1 correspondance with the enum in ManeuverControl!!
            ManeuverController.PositionMode mode = (ManeuverController.PositionMode)orbitPointDropdown.value;
            maneuverController.PositionSet(mode, orbitPosSlider.value);
        }

        private void SwitchManeuverPoint()
        {
            maneuverController.ActiveManeuverPointSet(maneuverPointDropdown.value);
            // get dV and update sliders and input fields
            double3 dV = maneuverController.Dv();
            UpdateDvInfo(dV);
        }

        private void ButtonPressed(Button button)
        {
            numManeuvers = maneuverController.MarkerCount();
            if (button == addManeuverButton) {
                maneuverController.PausedSet(true);
                maneuverController.ManeuverPointAdd();
                // by default set the point 10% ahead, so it is visually separated from the ship
                float offset = 0.1f;
                maneuverController.PositionSet(ManeuverController.PositionMode.PERIOD_PERCENT, offset);
                orbitPointDropdown.SetValueWithoutNotify((int)ManeuverController.PositionMode.PERIOD_PERCENT);
                orbitPosSlider.SetValueWithoutNotify(100f * offset);
                orbitPos.text = "" + 100f * offset;
                numManeuvers = maneuverController.MarkerCount();
                if (numManeuvers > 1) {
                    SetManeuverDropdown();
                }
                executeButton.gameObject.SetActive(true);
                UpdateDvInfo(double3.zero);
            } else if (button == delManeuverButton) {
                maneuverController.RemoveManeuverPoint(maneuverPointDropdown.value);
                maneuverController.ManeuverChainUpdate();
                // need to re-list the points in the dropdown
                SetManeuverDropdown();
            } else if (button == executeButton) {
                maneuverController.ExecuteManeuvers(removeMarkers: false);
                maneuverController.PausedSet(false);
            }
        }

        private void SetManeuverDropdown()
        {
            maneuverPointDropdown.gameObject.SetActive(true);
            maneuverPointDropdown.ClearOptions();
            List<string> options = new List<string>();
            for (int i = 0; i < maneuverController.MarkerCount(); i++)
                options.Add((i + 1).ToString());
            maneuverPointDropdown.AddOptions(options);
            maneuverPointDropdown.SetValueWithoutNotify(options.Count - 1);
            maneuverPointDropdown.gameObject.SetActive(true);
        }



        private void ActivateCanvas()
        {
            sliderR.onValueChanged.AddListener(delegate { RSWSliderChanged(sliderR); });
            sliderS.onValueChanged.AddListener(delegate { RSWSliderChanged(sliderS); });
            sliderW.onValueChanged.AddListener(delegate { RSWSliderChanged(sliderW); });
            // keep sliders in a reasonable range
            sliderR.minValue = -100.0f;
            sliderS.minValue = -100.0f;
            sliderW.minValue = -100.0f;
            sliderR.maxValue = 100.0f;
            sliderS.maxValue = 100.0f;
            sliderW.maxValue = 100.0f;

            // phase slider controls orbit position of maneuver point as a percenytage of a full
            // orbit *relative to ship*
            orbitPosSlider.onValueChanged.AddListener(delegate { OrbitPositionSlider(); });
            orbitPosSlider.minValue = 0;
            orbitPosSlider.maxValue = 100;
            orbitPosSlider.value = 0f;

            orbitPointDropdown.ClearOptions();
            orbitPointDropdown.AddOptions(new List<string>(ManeuverController.PositionStrings));
            orbitPointDropdown.onValueChanged.AddListener(delegate { OrbitPointDropdownChanged(); });

            inputR.onValueChanged.AddListener(delegate { TextInput(inputR); });
            inputS.onValueChanged.AddListener(delegate { TextInput(inputS); });
            inputW.onValueChanged.AddListener(delegate { TextInput(inputW); });

            orbitPos.onValueChanged.AddListener(delegate { OrbitPosInput(); });

            addManeuverButton.onClick.AddListener(delegate { ButtonPressed(addManeuverButton); });

            delManeuverButton.onClick.AddListener(delegate { ButtonPressed(delManeuverButton); });

            // don't enable others until there is a maneuver in play
            maneuverPointDropdown.onValueChanged.AddListener(delegate { SwitchManeuverPoint(); });
            maneuverPointDropdown.gameObject.SetActive(false);

            executeButton.onClick.AddListener(delegate { ButtonPressed(executeButton); });
            executeButton.gameObject.SetActive(false);
        }


    }
}
