using Unity.Mathematics;
using UnityEngine;

namespace GravityEngine2 {

    /// Started life with a snippet from https://catlikecoding.com/unity/tutorials/procedural-grid/

    public class MeshBuilder {

        private float xOrigin;
        private float yOrigin;
        private float xStep;
        private float yStep;


        public (Vector3[] vertices, int[] triangles) RectMesh(int numX, float xSize, int numY, float ySize)
        {
            Vector3[] vertices = new Vector3[numX * numY];
            xOrigin = -0.5f * xSize;
            yOrigin = -0.5f * ySize;
            xStep = xSize / (numX - 1);
            yStep = ySize / (numY - 1);
            for (int row = 0; row < numY - 1; row++) {
                for (int col = 0; col < numX; col++) {
                    vertices[row * numX + col] = new Vector3(xOrigin + col * xStep, yOrigin + row * yStep, 0);
                }
            }
            int[] triangles = new int[(numX - 1) * (numY - 1) * 6];
            for (int ti = 0, vi = 0, yi = 0; yi < numY - 1; yi++, vi++) {
                for (int xi = 0; xi < numX - 1; xi++, ti += 6, vi++) {
                    triangles[ti] = vi;
                    triangles[ti + 3] = triangles[ti + 2] = vi + 1;
                    triangles[ti + 4] = triangles[ti + 1] = vi + numX;
                    triangles[ti + 5] = vi + numX + 1;
                }
            }
            return (vertices, triangles);
        }


    }
}
