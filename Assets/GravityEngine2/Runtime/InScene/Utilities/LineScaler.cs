using System.IO;
using UnityEngine;

namespace GravityEngine2 {
    public class LineScaler : MonoBehaviour {

        //! scale of width w.r.t. zoom value
        public float zoomSlope = 0.2f;

        private LineRenderer[] lineRenderers;

        private float[] initialWidth;

        void Start()
        {
            FindAll();
        }

        /// <summary>
        /// Re-detect all line renderers in a scene
        /// </summary>
        public void FindAll()
        {
            lineRenderers = (LineRenderer[])FindObjectsByType(typeof(LineRenderer), FindObjectsSortMode.None);
            initialWidth = new float[lineRenderers.Length];
            for (int i = 0; i < lineRenderers.Length; i++) {
                initialWidth[i] = lineRenderers[i].startWidth;
            }
        }

        // Update is called once per frame
        public void SetZoom(float zoom)
        {
            if (lineRenderers == null)
                return;
            // turn zoom into a start/end width
            // this will be scene dependent

            for (int i = 0; i < lineRenderers.Length; i++) {
                lineRenderers[i].widthMultiplier = zoom * zoomSlope;
            }
        }
    }
}
