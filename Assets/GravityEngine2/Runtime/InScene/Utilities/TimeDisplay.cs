using UnityEngine;

namespace GravityEngine2 {
    public class TimeDisplay : MonoBehaviour {
        public GSController gsController;
        public TMPro.TMP_Text text;
        // Start is called before the first frame update


        // Update is called once per frame
        void Update()
        {
            text.text = gsController.GECore().TimeWorld().ToString("F3");
        }
    }
}
