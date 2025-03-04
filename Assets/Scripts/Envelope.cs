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

                Vector3 normal = CalculateNormal(t, a);

                Vector3 vertex = toolPath.Evaluate(t) + a * toolHeight * toolAxis + toolRadius * normal;
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

    // Calculate normal of envelope according to Bassegoda's paper
    private Vector3 CalculateNormal(float t, float a)
    {
        // tool surface derivative wrt a
        Vector3 sa = toolAxis;
        // tool surface derivative wrt t
        Vector3 st = toolPath.EvaluateTangent(t).normalized + a * Vector3.zero; // Todo replace with derivative of tool axis, but since that's fixed it's just zero for now
        Vector3 sNormal = Vector3.Cross(sa, st).normalized;

        // tool radius derivate wrt a
        float ra = 0; // Todo replace with actual derivate. Is fixed for now
        float alpha, beta, gamma;
        // Determinant of partial derivative matrix
        float determinant = (Vector3.Dot(sa, sa) * Vector3.Dot(st, st)) - (Vector3.Dot(sa, st) * Vector3.Dot(st, sa));
        float m11 = Vector3.Dot(st, st) / determinant;
        alpha = m11 * -ra;
        float m21 = Vector3.Dot(st, sa) / determinant;
        beta = m21 * -ra;
        gamma = (determinant > 0 ? -1 : 1) * Mathf.Sqrt(1 - ra * ra * m11);

        Vector3 envelopeNormal = alpha * sa + beta * st + gamma * sNormal;
        return envelopeNormal.normalized;
    }
}
