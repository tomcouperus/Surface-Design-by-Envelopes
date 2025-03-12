using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;

public class MeshData
{
    Vector3[] vertices;
    Vector2[] uvs;

    int[] triangles;
    int triangleIndex;

    public MeshData(int xVertices, int yVertices)
    {
        vertices = new Vector3[xVertices * yVertices];
        uvs = new Vector2[vertices.Length];

        triangles = new int[(xVertices - 1) * (yVertices - 1) * 6];
        triangleIndex = 0;
    }

    public void AddVertex(Vector3 vertex, Vector2 uv, int vertexIndex)
    {
        vertices[vertexIndex] = vertex;
        uvs[vertexIndex] = uv;
    }

    public void AddTriangle(int a, int b, int c)
    {
        triangles[triangleIndex] = a;
        triangles[triangleIndex + 1] = b;
        triangles[triangleIndex + 2] = c;
        triangleIndex += 3;
    }

    public Mesh CreateMesh()
    {
        Mesh mesh = new Mesh();
        CreateMesh(mesh);
        return mesh;
    }

    public void CreateMesh(Mesh mesh)
    {
        mesh.SetVertices(vertices);
        mesh.SetTriangles(triangles, 0);
        mesh.SetUVs(0, uvs);
        mesh.RecalculateNormals();
    }

    public void MakeDoubleSided()
    {
        List<Vector3> doubleVertices = new(vertices);
        doubleVertices.AddRange(vertices);

        List<Vector2> doubleUVs = new(uvs);
        doubleUVs.AddRange(uvs);

        List<int> doubleTriangles = new(triangles);
        int[] ccwTriangles = new int[triangles.Length];
        int triangleCount = triangles.Length / 3;
        for (int i = 0; i < triangleCount; i++)
        {
            ccwTriangles[i * 3] = triangles[i * 3];
            ccwTriangles[i * 3 + 1] = triangles[i * 3 + 2];
            ccwTriangles[i * 3 + 2] = triangles[i * 3 + 1];
        }
        doubleTriangles.AddRange(ccwTriangles);

        vertices = doubleVertices.ToArray();
        uvs = doubleUVs.ToArray();
        triangles = doubleTriangles.ToArray();
    }
}
