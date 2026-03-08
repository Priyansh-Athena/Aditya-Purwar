using UnityEngine;

namespace GravityEngine2 {
    /// <summary>
    /// Manages the display of stages for a multi-stage booster.
    /// 
    /// Registers with the GSBoosterMultiStage component to get notifications of stage changes and
    /// manages the display of the stages and payload models. 
    /// 
    /// As staging occurs a different model for each view of "the stack" can be displayed. 
    /// 
    /// Staging also results in a new GSBody being created for each stage and this code links the
    /// the per stage display game objects to the GSBody so they can be displayed. 
    /// </summary>
    public class GSStageDisplayMgr : MonoBehaviour {
        public GSBoosterMultiStage booster;

        [Header("Stage Display Bodies. Last stage is the payload.")]
        public GSDisplayBody[] stageDisplayBodies;

        private GSDisplayBody payloadDisplayBody;

        [Header("Stack Model Per Stage. Last stage is the payload.")]
        public GameObject[] stackModelPerStage;
        private void Awake()
        {
            payloadDisplayBody = stageDisplayBodies[stageDisplayBodies.Length - 1];
            if (booster == null) {
                Debug.LogError("GSStageDisplayMgr: Booster reference not set. Please assign a GSBoosterMultiStage in the inspector. " + gameObject.name);
                return;
            }

            // Register this manager with the booster
            booster.RegisterStageDisplayManager(this);
        }

        public void OnLaunch()
        {
            TrailRenderer[] trails = payloadDisplayBody.GetComponentsInChildren<TrailRenderer>();
            foreach (TrailRenderer trail in trails) {
                trail.emitting = true;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="stageNumber">number of stage being dropped</param>
        /// <param name="stageBodyId">body id for dropped stage</param>
        public void OnStageChange(int stageNumber, int stageBodyId)
        {
            // Update payload model
            if (stackModelPerStage != null && stackModelPerStage.Length > stageNumber) {
                int nextStage = stageNumber + 1;
                if (nextStage < stackModelPerStage.Length) {
                    // update display object so aligned with velocity
                    payloadDisplayBody.displayGO = stackModelPerStage[nextStage];
                    stackModelPerStage[nextStage].SetActive(true);
                    stackModelPerStage[stageNumber].SetActive(false);
                }
            }

            // Enable the new stage display body
            if (stageDisplayBodies != null) {
                GSDisplayBody currentStageDisplay = stageDisplayBodies[stageNumber];
                if (currentStageDisplay != null) {
                    // Get the GSDisplay for this stage
                    GSDisplay display = GSCommon.FindGSDisplayAbove(currentStageDisplay.transform);
                    if (display != null) {
                        if (!currentStageDisplay.gameObject.activeInHierarchy) {
                            currentStageDisplay.gameObject.SetActive(true);
                            display.DisplayObjectAdd(currentStageDisplay);
                        }
                        display.UpdateDisplayObjectBody(currentStageDisplay.DisplayId(), stageBodyId);
                        currentStageDisplay.DisplayEnabledSet(true);
                        // enable DisplayOrbit if it exists
                        GSDisplayOrbit displayOrbit = currentStageDisplay.GetComponent<GSDisplayOrbit>();
                        if (displayOrbit != null) {
                            displayOrbit.DisplayEnabledSet(true);
                        }
                        TrailRenderer[] trails = payloadDisplayBody.GetComponentsInChildren<TrailRenderer>();
                        foreach (TrailRenderer trail in trails) {
                            trail.emitting = true;
                        }
                    }
                }
            }
        }

        public void DisplayOrbitSet(bool display)
        {
            GSDisplayOrbit displayOrbit = payloadDisplayBody.GetComponent<GSDisplayOrbit>();
            if (displayOrbit != null) {
                displayOrbit.DisplayEnabledSet(display);
            }
        }

    }
}
