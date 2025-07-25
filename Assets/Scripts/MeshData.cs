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

    public MeshData(int xVertices, int yVertices, bool xWrap = false)
    {
        vertices = new Vector3[xVertices * yVertices];
        uvs = new Vector2[vertices.Length];

        int numTriangles;
        if (!xWrap) numTriangles = (xVertices - 1) * (yVertices - 1);
        else numTriangles = xVertices * (yVertices - 1);
        triangles = new int[numTriangles * 6];
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
}
