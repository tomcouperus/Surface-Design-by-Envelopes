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
    private Vector3 toolAxisT0 = Vector3.up;
    [SerializeField]
    private Vector3 toolAxisT1 = Vector3.up + Vector3.forward / 2;
    private BezierCurve toolPath;
    [SerializeField]
    private float toolRadius = 1.0f;
    [SerializeField]
    private float toolHeight = 2;

    [Header("Constraints")]
    [SerializeField]
    private Envelope adjacentEnvelopeA0;
    [SerializeField]
    private Envelope adjacentEnvelopeA1;

    [Header("Render")]
    [SerializeField]
    private int tSectors = 50;
    [SerializeField]
    private int aSectors = 20;

    public bool IsPositionContinuous { get { return adjacentEnvelopeA0 != null; } }
    public bool IsAxisConstrained { get { return IsPositionContinuous && adjacentEnvelopeA1 != null; } }

    private void Awake()
    {
        toolPath = GetComponentInChildren<BezierCurve>();
        MeshFilter meshFilter = GetComponent<MeshFilter>();
        mesh = new();
        meshFilter.mesh = mesh;
    }

    private void Start()
    {
        transform.localPosition -= transform.localPosition;

        UpdatePath();
        UpdateEnvelope();
    }

    private MeshData GenerateMeshData()
    {
        Debug.Log("Generating mesh for " + gameObject.name);
        int vertexIndex = 0;
        MeshData data = new MeshData(tSectors + 1, aSectors + 1);
        for (int tIdx = 0; tIdx <= tSectors; tIdx++)
        {
            for (int aIdx = 0; aIdx <= aSectors; aIdx++)
            {
                float t = (float)tIdx / tSectors;
                float a = (float)aIdx / aSectors;

                Vector3 vertex = GetEnvelopeAt(t, a);
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
        // data.MakeDoubleSided();
        return data;
    }

    public Vector3 GetEnvelopeAt(float t, float a)
    {
        return GetToolPathAt(t) + a * toolHeight * GetToolAxisAt(t) + GetToolRadiusAt(a) * CalculateNormal(t, a);
    }

    public Vector3 GetToolPathAt(float t)
    {
        if (IsPositionContinuous)
        {
            return adjacentEnvelopeA0.GetToolPathAt(t) + adjacentEnvelopeA0.GetToolAxisAt(t) * adjacentEnvelopeA0.toolHeight +
                   adjacentEnvelopeA0.GetToolRadiusAt(1) * adjacentEnvelopeA0.CalculateNormal(t, 1) -
                   GetToolRadiusAt(0) * CalculateNormal(t, 0);
        }
        else
        {
            return toolPath.Evaluate(t);
        }
    }

    public Vector3 GetToolPathTangentAt(float t)
    {
        if (IsPositionContinuous)
        {
            return adjacentEnvelopeA0.toolPath.EvaluateTangent(t);
        }
        else
        {
            return toolPath.EvaluateTangent(t);
        }
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
        Vector3 axis = Vector3.Lerp(toolAxisT0, toolAxisT1, t).normalized;
        if (IsAxisConstrained)
        {
            axis = adjacentEnvelopeA1.GetEnvelopeAt(t, 0) - adjacentEnvelopeA0.GetEnvelopeAt(t, 1);
        }
        return axis.normalized;
    }

    public Vector3 GetToolAxisDerivativeAt(float t)
    {
        // Todo replace with derivative of tool axis, but since that's fixed it's just zero for now
        return Vector3.zero;
    }

    // Calculate normal of envelope according to Bassegoda's paper
    // Expects t in [0, 1] and a in [0, 1]
    public Vector3 CalculateNormal(float t, float a)
    {
        // tool surface derivative wrt a
        Vector3 sa = toolHeight * GetToolAxisAt(t);
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
        float m21 = -Vector3.Dot(st, sa) / determinant;
        beta = m21 * -ra;
        gamma = -1 * (determinant > 0 ? 1 : -1) * Mathf.Sqrt(1 - ra * ra * m11);

        Vector3 envelopeNormal = alpha * sa + beta * st + gamma * sNormal;
        return envelopeNormal.normalized;
    }

    public void UpdateEnvelope()
    {
        CheckConstraints();
        GenerateMeshData().CreateMesh(mesh);
    }

    public void UpdatePath()
    {
        if (IsPositionContinuous)
        {
            List<Vector3> pathPoints = new();
            for (int tIdx = 0; tIdx <= tSectors; tIdx++)
            {
                float t = (float)tIdx / tSectors;
                pathPoints.Add(GetToolPathAt(t));
            }
            toolPath.UpdateLineRenderer(pathPoints);
        }
        else
        {
            toolPath.resolution = tSectors;
            toolPath.UpdateLineRenderer();
        }
    }

    public void CheckConstraints()
    {
        // Check distance between envelopes
        if (IsAxisConstrained)
        {
            for (int tIdx = 0; tIdx <= tSectors; tIdx++)
            {
                float t = (float)tIdx / tSectors;
                Vector3 axis = adjacentEnvelopeA1.GetEnvelopeAt(t, 0) - adjacentEnvelopeA0.GetEnvelopeAt(t, 1);
                if (axis.sqrMagnitude < 0 || axis.sqrMagnitude > toolHeight * toolHeight)
                {
                    Debug.LogWarning(gameObject.name + ": Constraining envelopes too far apart.");
                    break;
                }
            }
        }
    }

    private void OnValidate()
    {
        if (adjacentEnvelopeA0 == this) adjacentEnvelopeA0 = null;
        if (adjacentEnvelopeA1 == this || adjacentEnvelopeA1 == adjacentEnvelopeA0) adjacentEnvelopeA1 = null;
    }
}
