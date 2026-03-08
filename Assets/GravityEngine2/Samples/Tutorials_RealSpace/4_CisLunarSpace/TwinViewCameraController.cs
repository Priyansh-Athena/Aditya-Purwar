
using UnityEngine;

namespace GravityEngine2 {

    /// <summary>
    /// Simple controller to flip which camera is in the main view and which is in the mini-view.
    /// </summary>
    public class TwinViewCameraController : MonoBehaviour {

        [Header("Camera Settings for Mini-View")]
        public float x = 0.0f;
        public float y = 0.0f;

        public float width = 0.4f;
        public float height = 0.4f;

        public Camera camera1;
        public Camera camera2;

        private GESphereCamera sphericalCamera1;
        private GESphereCamera sphericalCamera2;

        [Header("Keyboard Control: V to toggle view")]
        public bool enableKeyboardControl = true;

        private Rect fullScreen = new Rect(0.0f, 0.0f, 1.0f, 1.0f);
        private Rect miniView;
        void Start()
        {
            miniView = new Rect(x, y, width, height);
            camera1.depth = 0;
            camera1.rect = fullScreen;
            sphericalCamera1 = camera1.transform.parent.GetComponent<GESphereCamera>();
            sphericalCamera1.interactive = true;
            camera2.depth = 1;
            camera2.rect = miniView;
            sphericalCamera2 = camera2.transform.parent.GetComponent<GESphereCamera>();
            sphericalCamera2.interactive = false;
        }

        // Update is called once per frame
        void Update()
        {
            if (enableKeyboardControl) {
                if (Input.GetKeyDown(KeyCode.V)) {
                    SwapViews();
                }
            }
        }

        public void SwapViews()
        {
            // mini-view is on top (depth 1)
            if (camera1.depth == 0) {
                camera1.depth = 1;
                camera2.depth = 0;
                camera1.rect = miniView;
                camera2.rect = fullScreen;
                sphericalCamera1.interactive = false;
                sphericalCamera2.interactive = true;
            } else {
                camera1.depth = 0;
                camera2.depth = 1;
                camera1.rect = fullScreen;
                camera2.rect = miniView;
                sphericalCamera1.interactive = true;
                sphericalCamera2.interactive = false;
            }
        }
    }
}
