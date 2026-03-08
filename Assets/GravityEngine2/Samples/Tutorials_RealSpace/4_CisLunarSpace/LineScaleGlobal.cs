using UnityEngine;

namespace GravityEngine2 {
    public class LineScaleGlobal : MonoBehaviour {
        public float width = 1.0f;
        // Start is called before the first frame update
        void Start()
        {
            LineRenderer[] lr = FindObjectsByType<LineRenderer>(FindObjectsSortMode.None);
            foreach (LineRenderer l in lr) {
                l.startWidth = width;
                l.endWidth = width;
            }
        }

        // Update is called once per frame
        void Update()
        {

        }
    }
}
