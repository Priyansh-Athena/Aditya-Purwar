using TMPro;
using Unity.Mathematics;
using UnityEngine;

namespace GravityEngine2 {
    /// <summary>
    /// Started life with a snippet from https://catlikecoding.com/unity/tutorials/procedural-grid/
    /// </summary>
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
    public class DisplayFunctionAsMesh : MonoBehaviour {
        [SerializeField]
        private float xWidth = 10;

        [SerializeField]
        private float yWidth = 10;

        [SerializeField]
        private bool useZ = true;

        private int numRows;
        private int numCols;


        public enum DataMorph { LINEAR, LOG10, LOGE };

        [SerializeField]
        private DataMorph dataMorph = DataMorph.LINEAR;

        [SerializeField]
        [Header("Optional (required if mouse clicks on grid are desired)")]
        private Camera viewCamera = null;

        [SerializeField]
        private DisplayMeshScale displayMeshScale;

        public enum DisplayTimeScale { SECONDS, DAYS };

        public DisplayTimeScale displayTimeScale = DisplayTimeScale.DAYS;
        public TextMeshPro xMinText;
        public TextMeshPro xMaxText;
        public TextMeshPro yMinText;
        public TextMeshPro yMaxText;
        public TextMeshPro xLabel;
        public TextMeshPro yLabel;

        public string xLabelText = "Departure Time (days)";
        public string yLabelText = "Arrival Time (days)";

        private Mesh mesh;
        private Vector3[] vertices;
        private Vector2[] uv;

        public delegate void GridClicked(float x, float y);
        private GridClicked gridClicked;

        public GameObject clickedMarker;

        public Material myMaterial;

        public Renderer myRenderer;

        // Start is called before the first frame update
        void Start()
        {
            // Create a gradient texture to color code the function values.
            int width = 64;
            int height = 64;
            int texelCount = width * height;

            // Rent a temporary array to hold the colours
            // so we don't allocate large structures unnecessarily.
            // gradient in x coordinate. y > 0.5 is all black
            // "colors must contain the pixels row by row, starting at the bottom left of the texture"
            var colors = System.Buffers.ArrayPool<Color32>.Shared.Rent(texelCount);
            for (int r = 0; r < height; r++) {
                for (int c = 0; c < width; c++) {
                    //if (c < width / 2) {
                    colors[r * width + c] = Color.Lerp(Color.blue, Color.red, (float)(c / (float)(width - 1)));
                    // } else {
                    //     colors[r * width + c] = Color.black;
                    // }
                }
            }
            Texture2D myTexture = new Texture2D(width, height);
            // Using 32-bit colours uses 1/4 the memory, for efficiency.
            myTexture.SetPixels32(colors);
            myTexture.wrapMode = TextureWrapMode.Clamp;

            // Recycle the temporary array we rented.
            System.Buffers.ArrayPool<Color32>.Shared.Return(colors);

            // Update the GPU copy of the texture, including mipmaps.
            myTexture.Apply(true, false);

            myMaterial.mainTexture = myTexture;
            Debug.Log("Texture created");

            myRenderer.material.mainTexture = myTexture;

            if (displayMeshScale != null) {
                displayMeshScale.GetComponent<MeshRenderer>().material.mainTexture = myTexture;
            }
            float hSizeDiv2 = xWidth / 2;
            float ySizeDiv2 = yWidth / 2;
            float textOffset = 5.0f;

            xMinText.transform.localPosition = new Vector3(-hSizeDiv2, -ySizeDiv2 - textOffset, 0.0f);
            xMaxText.transform.localPosition = new Vector3(hSizeDiv2, -ySizeDiv2 - textOffset, 0.0f);
            yMinText.transform.localPosition = new Vector3(-hSizeDiv2 - textOffset, -ySizeDiv2, 0.0f);
            yMaxText.transform.localPosition = new Vector3(-hSizeDiv2 - textOffset, ySizeDiv2, 0.0f);

            xLabel.transform.localPosition = new Vector3(0f, -ySizeDiv2 - textOffset, 0.0f);
            yLabel.transform.localPosition = new Vector3(-hSizeDiv2 - 2 * textOffset, 0, 0.0f);
            xLabel.text = xLabelText;
            yLabel.text = yLabelText;
        }

        /// <summary>
        /// Create a mesh and set color based on adat values. The data can be sparse but must be aligned on 
        /// the grid. 
        /// </summary>
        /// <param name="data"></param>
        /// <param name="numRows"></param>
        /// <param name="numCols"></param>
        /// <param name="gridClicked"></param>
        public void GenerateFromSparseData(float3[] data, int numCols, int numRows, GridClicked gridClicked, float zLimit = float.NaN)
        {
            this.gridClicked = gridClicked;
            Debug.Log("Mesh generated");
            GetComponent<MeshFilter>().mesh = mesh = new Mesh();
            mesh.name = "Procedural Grid";
            this.numRows = numRows;
            this.numCols = numCols;

            int[] triangles;
            MeshBuilder meshBuilder = new MeshBuilder();
            (vertices, triangles) = meshBuilder.RectMesh(numCols, xWidth, numRows, yWidth);
            uv = new Vector2[numRows * numCols];

            ComputeScale(data, numCols, numRows);

            if (!float.IsNaN(zLimit)) {
                if (dataMorph == DataMorph.LOG10) {
                    zRange = math.log10(zLimit) - zMin;
                } else if (dataMorph == DataMorph.LOGE) {
                    zRange = math.log(zLimit) - zMin;
                } else {
                    zRange = zLimit - zMin;
                }
            }
            if (displayMeshScale != null) {
                displayMeshScale.GenerateFromMinMax(zMin, zMin + zRange, dataMorph);
            }
            int index;
            int minIndex = int.MaxValue;
            float zMinValue = float.MaxValue;
            for (int i = 0; i < data.Length; i++) {
                index = IndexForData(data[i], numCols, numRows);
                if (index >= vertices.Length) {
                    Debug.LogWarningFormat("IndexForData data[{0}]={1} index={2}", i, data[i], index);
                    index = vertices.Length - 1;
                }
                float z = data[i].z;
                if (z < zMinValue) {
                    zMinValue = z;
                    minIndex = index;
                }
                switch (dataMorph) {
                    case DataMorph.LOG10:
                        z = Mathf.Log10(z);
                        break;

                    case DataMorph.LOGE:
                        z = Mathf.Log(z);
                        break;

                    default:
                        break;
                }
                if (float.IsNaN(z)) {
                    uv[index] = new Vector2(0.1f, 0.9f);
                } else {
                    z = Mathf.Clamp01((z - zMin) / zRange);
                    uv[index] = new Vector2(z, 0.1f);
                }
                if (!useZ)
                    z = 0;
                vertices[index] = new Vector3(vertices[index].x, vertices[index].y, z);
                //Debug.LogFormat("i={0} data[{0}]={1} index={2} uv={3}", i, data[i], index, uv[index]);
            }
            if (clickedMarker != null) {
                clickedMarker.transform.localPosition = vertices[minIndex];
            }

            mesh.vertices = vertices;
            mesh.uv = uv;
            mesh.triangles = triangles;
            mesh.RecalculateNormals();

            // MUST assign mesh once it has been filled in and not before!
            GetComponent<MeshCollider>().sharedMesh = mesh;
        }

        private float xMin;
        private float xStep;
        private float yMin;
        private float yStep;

        private float zMin;

        private float zMax;
        private float zRange;
        public void ComputeScale(float3[] data, int numX, int numY)
        {
            xMin = float.MaxValue;
            double xMax = float.MinValue;
            yMin = float.MaxValue;
            double yMax = float.MinValue;
            zMin = float.MaxValue;
            double zMax = float.MinValue;
            foreach (float3 pt in data) {
                xMin = math.min(xMin, pt.x);
                xMax = math.max(xMax, pt.x);
                yMin = math.min(yMin, pt.y);
                yMax = math.max(yMax, pt.y);
                zMin = math.min(zMin, pt.z);
                zMax = math.max(zMax, pt.z);
            }
            xStep = (float)(xMax - xMin) / numX;
            yStep = (float)(yMax - yMin) / numY;
            Debug.Log($"zRange={zRange} zMin={zMin} zMax={zMax}");
            if (dataMorph == DataMorph.LOG10) {
                zMin = (float)math.log10(zMin);
                zMax = (float)math.log10(zMax);
            } else if (dataMorph == DataMorph.LOGE) {
                zMin = (float)math.log(zMin);
                zMax = (float)math.log(zMax);
            }
            zRange = (float)(zMax - zMin);

            float scale = 1.0f;
            if (displayTimeScale == DisplayTimeScale.DAYS) {
                scale = (float)GBUnits.SECS_PER_SOLAR_DAY;
            }
            xMinText.text = $"{xMin / scale:F2}";
            xMaxText.text = $"{xMax / scale:F2}";
            yMinText.text = $"{yMin / scale:F2}";
            yMaxText.text = $"{yMax / scale:F2}";
        }

        public int IndexForData(float3 data, int numCols, int numRows)
        {
            int col = (int)((data.x - xMin) / xStep);
            col = math.min(col, numCols - 1);
            int row = (int)((data.y - yMin) / yStep);
            row = math.min(row, numRows - 1);
            int index = row * numCols + col;
            //            Debug.LogFormat("IndexForData data={0} row={1} col={2} index={3}", data, row, col, index);
            return index;
        }


        void OnMouseDown()
        {
            Debug.Log("Clicked " + gameObject.name);
            // find the triangle that was hit 
            // https://docs.unity3d.com/ScriptReference/RaycastHit-triangleIndex.html
            RaycastHit hit;
            if (!Physics.Raycast(viewCamera.ScreenPointToRay(Input.mousePosition), out hit))
                return;

            MeshCollider meshCollider = hit.collider as MeshCollider;
            if (meshCollider == null || meshCollider.sharedMesh == null)
                return;
            Debug.Log("Hit triangle = " + hit.triangleIndex);
            Vector3[] vertices = mesh.vertices;
            int[] triangles = mesh.triangles;
            Vector3 p0 = vertices[triangles[hit.triangleIndex * 3 + 0]];
            Vector3 p1 = vertices[triangles[hit.triangleIndex * 3 + 1]];
            Vector3 p2 = vertices[triangles[hit.triangleIndex * 3 + 2]];
            Vector2 uv = mesh.uv[triangles[hit.triangleIndex * 3 + 0]];

            Debug.LogFormat("Triangle is p0={0} p1={1} p2={2} uv={3}", p0, p1, p2, uv);
            float x = (p0.x + xWidth / 2) / xWidth;
            float y = (p0.y + yWidth / 2) / yWidth;
            x = xMin + x * numCols * xStep;
            y = yMin + y * numRows * yStep;
            gridClicked(x, y);
            if (clickedMarker != null) {
                clickedMarker.transform.position = new Vector3(hit.point.x, hit.point.y, 0);
            }
        }
    }
}
