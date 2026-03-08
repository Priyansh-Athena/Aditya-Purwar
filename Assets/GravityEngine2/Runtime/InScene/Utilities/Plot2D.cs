
using TMPro;
using UnityEngine;

namespace GravityEngine2 {

    /// <summary>
    /// Plot 2D data.
    /// 
    /// Very simple plot routine that plots data in XY space. Since Unity is
    /// left-handed this means that the camera is located at -Z looking towards 
    /// the origin.
    /// </summary>
    public class Plot2D : MonoBehaviour {
        public float horizontalSize = 100.0f;
        public float verticalSize = 100.0f;

        public LineRenderer lineRenderer;

        public LineRenderer axisRenderer;

        public TextMeshPro xMinText;
        public TextMeshPro xMaxText;
        public TextMeshPro yMinText;
        public TextMeshPro yMaxText;

        public TextMeshPro xLabel;
        public TextMeshPro yLabel;

        public string xLabelText = "X";
        public string yLabelText = "Y";

        public TextMeshPro markerText;

        private float minX, maxX, minY, maxY;

        private float scaleX, scaleY, hOrigin, vOrigin;

        // Start is called before the first frame update
        void Start()
        {
            lineRenderer.positionCount = 0;
            axisRenderer.positionCount = 0;
        }

        public void PlotData(Vector2[] points)
        {
            minX = float.MaxValue;
            maxX = float.MinValue;
            minY = float.MaxValue;
            maxY = float.MinValue;

            // find the min and max of the data
            foreach (Vector2 point in points) {
                if (point.x < minX) minX = point.x;
                if (point.x > maxX) maxX = point.x;
                if (point.y < minY) minY = point.y;
                if (point.y > maxY) maxY = point.y;
            }

            scaleX = horizontalSize / (maxX - minX);
            scaleY = verticalSize / (maxY - minY);
            hOrigin = -horizontalSize / 2.0f;
            vOrigin = -verticalSize / 2.0f;

            float yValue;
            // plot the data
            lineRenderer.positionCount = points.Length;
            for (int i = 0; i < points.Length; i++) {
                yValue = points[i].y;
                if (float.IsNaN(yValue) || float.IsInfinity(yValue)) {
                    yValue = maxY;
                }
                lineRenderer.SetPosition(i, new Vector3(hOrigin + (points[i].x - minX) * scaleX, vOrigin + (yValue - minY) * scaleY, 0.0f));
            }

            // plot the axes
            axisRenderer.positionCount = 3;
            float hSizeDiv2 = horizontalSize / 2.0f;
            float vSizeDiv2 = verticalSize / 2.0f;
            // x axis
            axisRenderer.SetPosition(0, new Vector3(hSizeDiv2, -vSizeDiv2, 0.0f));
            axisRenderer.SetPosition(1, new Vector3(-hSizeDiv2, -vSizeDiv2, 0.0f));
            // y axis
            axisRenderer.SetPosition(2, new Vector3(-hSizeDiv2, vSizeDiv2, 0.0f));

            float textOffset = 5.0f;
            // position the labels
            xMinText.transform.localPosition = new Vector3(-hSizeDiv2, -vSizeDiv2 - textOffset, 0.0f);
            xMaxText.transform.localPosition = new Vector3(hSizeDiv2, -vSizeDiv2 - textOffset, 0.0f);
            yMinText.transform.localPosition = new Vector3(-hSizeDiv2 - textOffset, -vSizeDiv2, 0.0f);
            yMaxText.transform.localPosition = new Vector3(-hSizeDiv2 - textOffset, vSizeDiv2, 0.0f);
            xLabel.transform.localPosition = new Vector3(0f, -vSizeDiv2 - 2 * textOffset, 0.0f);
            yLabel.transform.localPosition = new Vector3(-hSizeDiv2 - 2 * textOffset, 0, 0.0f);

            xMinText.text = minX.ToString("F2");
            xMaxText.text = maxX.ToString("F2");
            yMinText.text = minY.ToString("F2");
            yMaxText.text = maxY.ToString("F2");

            xLabel.text = xLabelText;
            yLabel.text = yLabelText;
        }

        public void MarkerUpdate(Vector2 marker, GameObject markerObject)
        {
            markerObject.transform.localPosition = new Vector3(hOrigin + (marker.x - minX) * scaleX, vOrigin + (marker.y - minY) * scaleY, 0.0f);
            markerText.transform.localPosition = new Vector3(0, 0.3f * verticalSize, 0.0f);
            markerText.text = "marker dV: " + marker.y.ToString("F2");
        }
    }
}
