using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(MeshRenderer))]
[RequireComponent(typeof(MeshFilter))]
public class Envelope : MonoBehaviour
{
    [SerializeField]
    private Mesh mesh;
    [Header("Tool")]
    [SerializeField]
    private Vector3 toolAxis = Vector3.up;
    public BezierCurve toolPath;
    [SerializeField]
    private float toolRadius = 1.0f;
    [SerializeField]
    public float toolHeight = 2;

    [Header("Constraints")]
    [SerializeField]
    private Envelope adjacentEnvelope;

    [Header("Render")]
    [SerializeField]
    private int tSectors = 50;
    [SerializeField]
    private int aSectors = 20;

    private void Awake()
    {
        toolPath = GetComponentInChildren<BezierCurve>();
        MeshFilter meshFilter = GetComponent<MeshFilter>();
        mesh = new();
        meshFilter.mesh = mesh;
    }

    private void Start()
    {
        UpdateEnvelope();
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

                Vector3 vertex = GetToolPathAt(t) + a * toolHeight * GetToolAxisAt(t) + GetToolRadiusAt(a) * normal;
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

    public Vector3 GetToolPathAt(float t)
    {
        return toolPath.Evaluate(t);
    }

    public Vector3 GetToolPathTangentAt(float t)
    {
        return toolPath.EvaluateTangent(t);
    }

    public float GetToolRadiusAt(float a)
    {
        return toolRadius;
    }

    public float GetRadiusDerivateAt(float a)
    {
        // Todo replace with actual derivate. Is fixed for now
        return 0;
    }

    public Vector3 GetToolAxisAt(float t)
    {
        return toolAxis;
    }

    public Vector3 GetToolAxisDerivativeAt(float t)
    {
        // Todo replace with derivative of tool axis, but since that's fixed it's just zero for now
        return Vector3.zero;
    }

    // Calculate normal of envelope according to Bassegoda's paper
    // Expects t in [0, 1] and a in [0, 1]
    private Vector3 CalculateNormal(float t, float a)
    {
        // tool surface derivative wrt a
        Vector3 sa = GetToolAxisAt(t);
        // tool surface derivative wrt t
        Vector3 st = GetToolPathTangentAt(t).normalized + a * toolHeight * GetToolAxisDerivativeAt(t);
        Vector3 sNormal = Vector3.Cross(sa, st).normalized;

        // tool radius derivate wrt a
        float ra = GetRadiusDerivateAt(a);
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

    public void UpdateEnvelope()
    {
        if (adjacentEnvelope != null)
        {
            // The path of an envelope A adjacent to envelope B is the same as B's, but just translated a bit.
            toolPath.points.Clear();
            toolPath.points.AddRange(adjacentEnvelope.toolPath.points);
            // Place the path points in the right position
            for (int i = 0; i < toolPath.points.Count; i++)
            {
                float t = (float)i / toolPath.points.Count;
                toolPath.points[i] += adjacentEnvelope.GetToolAxisAt(t) * adjacentEnvelope.toolHeight +
                                      adjacentEnvelope.GetToolRadiusAt(1.0f) * adjacentEnvelope.CalculateNormal(t, 1.0f) -
                                      GetToolRadiusAt(0.0f) * CalculateNormal(t, 0.0f);
            }
        }

        toolPath.UpdateLineRenderer();
        // Recreate the envelope's mesh
        GenerateMeshData().CreateMesh(mesh);
    }
}
