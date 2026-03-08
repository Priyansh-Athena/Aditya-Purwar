using UnityEngine.UI;
using UnityEngine;

namespace GravityEngine2 {
    /// <summary>
	/// Very basic time set. Assumes slider scale maps directly to world time.
	/// </summary>
    public class GETimeSlider : MonoBehaviour {
        public Slider slider;
        public GSController gsController;

        private float time;
        private bool setTime;

        // Start is called before the first frame update
        void Start()
        {
            slider.onValueChanged.AddListener(TimeSlide);
        }

        private void TimeSlide(float t)
        {
            time = t;
            setTime = true;
        }

        private void TimeSet(GECore ge, object arg)
        {
            gsController.TimeWorldSet(time);
        }

        private void TimeUpdate(GECore ge, object arg)
        {
            slider.SetValueWithoutNotify((float)gsController.GECore().TimeWorld());
        }

        void Update()
        {
            if (setTime) {
                gsController.GECore().PhyLoopCompleteCallbackAdd(TimeSet);
                setTime = false;
            } else {
                gsController.GECore().PhyLoopCompleteCallbackAdd(TimeUpdate);
            }

        }
    }
}
