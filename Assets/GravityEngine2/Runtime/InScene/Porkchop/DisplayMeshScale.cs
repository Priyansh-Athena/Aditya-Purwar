using TMPro;
using Unity.Mathematics;
using UnityEngine;

namespace GravityEngine2 {
    /// <summary>
    /// Started life with a snippet from https://catlikecoding.com/unity/tutorials/procedural-grid/
    /// </summary>
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
    public class DisplayMeshScale : MonoBehaviour {
        [SerializeField]
        private float xWidth = 10;

        [SerializeField]
        private float yWidth = 100;

        [SerializeField]
        private TMP_Text minText;

        [SerializeField]
        private TMP_Text maxText;

        private int numRows = 100;
        private int numCols = 10;

        private Mesh mesh;
        private Vector3[] vertices;
        private Vector2[] uv;

        private float xStep;
        private float yStep;
        private float zStep;

        /// <summary>
        /// Create a mesh and set color based on data values. The data can be sparse but must be aligned on 
        /// the grid. 
        /// </summary>
        /// <param name="data"></param>
        /// <param name="numRows"></param>
        /// <param name="numCols"></param>
        /// <param name="gridClicked"></param>
        public void GenerateFromMinMax(float minValue, float maxValue, DisplayFunctionAsMesh.DataMorph dataMorph)
        {
            Debug.LogFormat("Scale mesh min={0} max={1}", minValue, maxValue);
            if (dataMorph == DisplayFunctionAsMesh.DataMorph.LOG10) {
                minText.text = "log10\n(" + math.pow(10, minValue).ToString("F3") + ")";
                maxText.text = "log10\n(" + math.pow(10, maxValue).ToString("F3") + ")";
            } else if (dataMorph == DisplayFunctionAsMesh.DataMorph.LOGE) {
                minText.text = "ln\n(" + math.exp(minValue).ToString("F3") + ")";
                maxText.text = "ln\n(" + math.exp(maxValue).ToString("F3") + ")";
            } else {
                minText.text = minValue.ToString("F3");
                maxText.text = maxValue.ToString("F3");
            }
            minText.transform.localPosition = new Vector3(5, -yWidth / 2.0f - 10.0f, 0);
            maxText.transform.localPosition = new Vector3(5, yWidth / 2.0f + 10.0f, 0);
            GetComponent<MeshFilter>().mesh = mesh = new Mesh();
            mesh.name = "Procedural Grid";

            int[] triangles;
            MeshBuilder meshBuilder = new MeshBuilder();
            (vertices, triangles) = meshBuilder.RectMesh(numCols, xWidth, numRows, yWidth);
            uv = new Vector2[numRows * numCols];

            xStep = xWidth / numCols;
            yStep = yWidth / numRows;
            // just step z from 0..1
            zStep = 1.0f / numRows;

            // create new colors array where the colors will be created.

            for (int r = 0; r < numRows; r++) {
                for (int c = 0; c < numCols; c++) {
                    vertices[r * numCols + c] = new Vector3(c * xStep - xWidth / 2.0f, r * yStep - yWidth / 2.0f, 0.0f);
                    uv[r * numCols + c] = new Vector2(r * zStep, 0.1f);
                }
            }

            // assign the array of colors to the Mesh.

            mesh.vertices = vertices;
            mesh.uv = uv;
            mesh.triangles = triangles;
            mesh.RecalculateNormals();

            // MUST assign mesh once it has been filled in and not before!
            GetComponent<MeshCollider>().sharedMesh = mesh;
        }

    }
}
