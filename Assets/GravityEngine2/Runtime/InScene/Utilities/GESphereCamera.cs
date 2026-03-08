namespace GravityEngine2 {
    using System.Collections.Generic;
    using UnityEngine;

    /// <summary>
    /// Key control to rotate camera boom using Arrow keys for rotation and < > keys for zoom.
    ///
    /// Assumes the Main Camara is a child of the object holding this script with a local position offset
    /// (the boom length) and oriented to point at this object. Then pressing the keys will spin the camera
    /// around the object this script is attached to.
    /// 
    /// If a LineScalar is present, zoom change call will be made to the LineScalar. 
    /// 
    /// </summary>
    public class GESphereCamera : MonoBehaviour {

        public GSDisplay gsDisplay;

        public bool interactive = true;

        //! Rate of spin (degrees per Update)
        [Header("Mouse middle button to move and zoom")]
        public bool enableMouse = true;
        public float spinRate = 1f;
        public float mouseSpinRate = 3f;
        public float zoomSize = 1f;
        public float mouseWheelZoom = 0.5f;

        [Header("Key Options for rotation")]
        public bool arrowKeys = true;
        public bool ijklKeys = true;
        public bool macArrowKeys = true;

        private Vector3 initialBoom;
        // factor by which zoom is changed 
        private float zoomStep = 0.02f;
        private Camera boomCamera;

        //! Optional Line Scalar (auto-detected)
        private LineScaler lineScaler;

        private const float minZoomSize = 0.01f;

        private Vector3 leftRightAxis;
        private Vector3 upDnAxis;

        // Macbook arrow keys added empirically. Someday find named types.
        private List<KeyCode> leftCodes = new List<KeyCode>();
        private List<KeyCode> rightCodes = new List<KeyCode>();
        private List<KeyCode> upCodes = new List<KeyCode>();
        private List<KeyCode> dnCodes = new List<KeyCode>();


        // Use this for initialization
        void Start()
        {
            // allow lazy scene building. If no GSDisplay, use first one we find
            if (gsDisplay == null) {
                gsDisplay = FindAnyObjectByType<GSDisplay>();
                Debug.Log("** GSDisplay was not set, using first one we find.");
            }


            boomCamera = GetComponentInChildren<Camera>();
            if (boomCamera != null) {
                initialBoom = boomCamera.transform.localPosition;
            }

            lineScaler = GetComponent<LineScaler>();
            if (gsDisplay.xzOrbitPlane) {
                leftRightAxis = Vector3.up;
                upDnAxis = Vector3.right;
            } else {
                leftRightAxis = Vector3.forward;
                upDnAxis = Vector3.up;
            }
            if (arrowKeys) {
                leftCodes.Add(KeyCode.LeftArrow);
                rightCodes.Add(KeyCode.RightArrow);
                upCodes.Add(KeyCode.UpArrow);
                dnCodes.Add(KeyCode.DownArrow);
            }
            if (ijklKeys) {
                leftCodes.Add(KeyCode.J);
                rightCodes.Add(KeyCode.L);
                upCodes.Add(KeyCode.I);
                dnCodes.Add(KeyCode.K);
            }
            if (macArrowKeys) {
                leftCodes.Add((KeyCode)276);
                rightCodes.Add((KeyCode)275);
                upCodes.Add((KeyCode)273);
                dnCodes.Add((KeyCode)274);
            }
        }

        /// <summary>
        /// The "boom" is the local position of the transform holding the Camera.
        ///
        /// By convention this is set to look down on the XZ orbital plane from Y, so
        /// the length is all in the y component of the local position.
        /// </summary>
        /// <param name="len"></param>
        public void BoomLengthReset(float len)
        {
            Camera cam = GetComponentInChildren<Camera>();
            cam.transform.localPosition = new Vector3(0, len, 0);
        }

        private bool KeyPressed(List<KeyCode> codes)
        {
            bool pressed = false;
            foreach (KeyCode k in codes)
                if (Input.GetKey(k)) {
                    pressed = true;
                    break;
                }
            return pressed;
        }

        // Update is called once per frame
        void Update()
        {
            if (!interactive) {
                return;
            }
            float lastZoom = zoomSize;

            if (KeyPressed(leftCodes)) {
                transform.rotation *= Quaternion.AngleAxis(spinRate, leftRightAxis);
            } else if (KeyPressed(rightCodes)) {
                transform.rotation *= Quaternion.AngleAxis(-spinRate, leftRightAxis);
            } else if (KeyPressed(upCodes)) {
                transform.rotation *= Quaternion.AngleAxis(spinRate, upDnAxis);
            } else if (KeyPressed(dnCodes)) {
                transform.rotation *= Quaternion.AngleAxis(-spinRate, upDnAxis);
            } else if (Input.GetKey(KeyCode.Comma)) {
                // change boom length
                zoomSize += zoomStep;
                boomCamera.transform.localPosition = zoomSize * initialBoom;
            } else if (Input.GetKey(KeyCode.Period)) {
                // change boom lenght
                // change boom length
                zoomSize -= zoomStep;
                zoomSize = Mathf.Max(zoomSize, minZoomSize);
                boomCamera.transform.localPosition = zoomSize * initialBoom;
            }

            if (enableMouse) {
                // Mouse commands - middle mouse button to spin
                if (Input.GetMouseButton(2) || Input.GetMouseButton(1)) {
                    float h = mouseSpinRate * Input.GetAxis("Mouse X");
                    float v = mouseSpinRate * Input.GetAxis("Mouse Y");
                    transform.Rotate(v, h, 0);
                }
                // scroll speed typically +/- 0.1
                float scrollSpeed = Input.GetAxis("Mouse ScrollWheel");
                if (scrollSpeed != 0) {
                    zoomSize += mouseWheelZoom * scrollSpeed;
                    zoomSize = Mathf.Max(zoomSize, minZoomSize);
                    boomCamera.transform.localPosition = zoomSize * initialBoom;
                }
            }

            // if LineScalar is around, tell it about the new zoom setting
            if (lastZoom != zoomSize) {
                if (lineScaler != null) {
                    lineScaler.SetZoom(zoomSize);
                }
            }
        }
    }

}
