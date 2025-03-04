using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(MeshRenderer))]
[RequireComponent(typeof(MeshFilter))]
public class Envelope : MonoBehaviour
{
    [Header("Tool")]
    [SerializeField]
    private Vector3 toolAxis = Vector3.up;
    private BezierCurve toolPath;
    [SerializeField]
    private float toolRadius = 1.0f;
    [SerializeField]
    private float toolHeight = 2;

    [Header("Render")]
    [SerializeField]
    private int tSectors = 50;
    [SerializeField]
    private int aSectors = 20;

    private void Awake()
    {
        toolPath = GetComponentInChildren<BezierCurve>();
    }

    private void Start()
    {
        MeshFilter meshFilter = GetComponent<MeshFilter>();
        Mesh mesh = GenerateMeshData().CreateMesh();
        meshFilter.sharedMesh = mesh;
    }

    private MeshData GenerateMeshData()
    {
        int vertexIndex = 0;
        MeshData data = new MeshData(tSectors + 1, aSectors + 1);
        for (int tIdx = 0; tIdx <= tSectors; tIdx++)
        {
            for (int aIdx = 0; aIdx <= aSectors; aIdx++)
            {
                float t = 1.0f / tSectors * tIdx;
                float a = 1.0f / aSectors * aIdx;
                Debug.Log("T: " + t + " a:" + a);

                Vector3 vertex = toolPath.Evaluate(t * toolPath.NumSegments) + a * toolHeight * toolAxis;
                Debug.Log(vertex);
                Vector2 uv = new Vector2(t, a);

                data.AddVertex(vertex, uv, vertexIndex);
                bool makeTriangle = tIdx < tSectors && aIdx < aSectors;
                if (makeTriangle)
                {
                    int v1 = vertexIndex;
                    int v2 = vertexIndex + 1;
                    int v3 = vertexIndex + aSectors + 1;
                    int v4 = vertexIndex + aSectors + 2;
                    data.AddTriangle(v1, v2, v3);
                    data.AddTriangle(v2, v4, v3);
                }
                vertexIndex++;
            }
        }
        return data;
    }
}
