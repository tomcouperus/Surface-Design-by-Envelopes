using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(MeshRenderer))]
[RequireComponent(typeof(MeshFilter))]
public class Envelope : MonoBehaviour
{

    [SerializeField]
    private Mesh mesh;
    [SerializeField]
    private GameObject sphere;
    [Header("Tool")]
    [SerializeField]
    private Vector3 toolAxisT0 = Vector3.up;
    [SerializeField]
    private Vector3 toolAxisT1 = Vector3.up + Vector3.forward / 2;
    private BezierCurve toolPath;
    [SerializeField]
    private float toolRadius = 0.5f;
    private Tool tool;

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
    [SerializeField]
    [Range(0, 1)]
    private float t = 0;
    [SerializeField]
    [Range(0, 1)]
    private float a = 0;
    [SerializeField]
    private bool showTool = true;
    [SerializeField]
    private bool showSphere = false;

    public bool IsPositionContinuous { get { return adjacentEnvelopeA0 != null; } }
    public bool IsAxisConstrained { get { return IsPositionContinuous && adjacentEnvelopeA1 != null; } }

    private void Awake()
    {
        toolPath = GetComponentInChildren<BezierCurve>();
        tool = GetComponentInChildren<Tool>();

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

    void Update()
    {
        UpdateTool();
        UpdateSphere();
    }

    private void UpdateTool()
    {
        tool.gameObject.SetActive(showTool);
        if (!showTool) return;
        tool.transform.localScale = new Vector3(toolRadius, 1, toolRadius);
        tool.transform.localPosition = GetToolPathAt(t);
        tool.transform.rotation = Quaternion.LookRotation(GetToolAxisAt(t)) * Quaternion.FromToRotation(Vector3.up, Vector3.forward);
    }

    private void UpdateSphere()
    {
        if (sphere == null) return;
        sphere.SetActive(showSphere);
        sphere.transform.localScale = new Vector3(toolRadius, toolRadius, toolRadius);
        sphere.transform.localPosition = GetToolPathAt(t) + a * GetToolAxisAt(t);
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
        return GetToolPathAt(t) + a * GetToolAxisAt(t) + GetToolRadiusAt(a) * CalculateNormal(t, a);
    }

    public Vector3 GetEnvelopeDtAt(float t, float a)
    {
        return Vector3.zero;
    }

    public Vector3 GetToolPathAt(float t)
    {
        if (IsPositionContinuous)
        {
            return adjacentEnvelopeA0.GetEnvelopeAt(t, 1) +
                   GetToolRadiusAt(0) * CalculateNormal(t, 0);
        }
        else
        {
            return toolPath.Evaluate(t);
        }
    }

    public Vector3 GetToolPathDtAt(float t)
    {
        if (IsPositionContinuous)
        {
            return adjacentEnvelopeA0.GetToolPathDtAt(t);
        }
        else
        {
            return toolPath.EvaluateTangent(t);
        }
    }

    public Vector3 GetToolPathDt2At(float t)
    {
        if (IsPositionContinuous)
        {
            return adjacentEnvelopeA0.GetToolPathDt2At(t);
        }
        else
        {
            return toolPath.EvaluateSecondDerivative(t);
        }
    }

    public float GetToolRadiusAt(float a)
    {
        return toolRadius;
    }

    public float GetToolRadiusDaAt(float a)
    {
        // Todo replace with actual derivate. Is fixed for now
        return 0;
    }

    public Vector3 GetToolAxisAt(float t)
    {
        Vector3 axis = Vector3.Lerp(toolAxisT0, toolAxisT1, t);
        return axis.normalized;
    }

    public Vector3 GetToolAxisDtAt(float t)
    {
        Vector3 axis = Vector3.Lerp(toolAxisT0, toolAxisT1, t);
        Vector3 axis_t = toolAxisT1 - toolAxisT0;
        return MathUtility.NormalVectorDerivative(axis, axis_t);
    }

    public Vector3 GetToolAxisDt2At(float t)
    {
        Vector3 axis = Vector3.Lerp(toolAxisT0, toolAxisT1, t);
        Vector3 axis_t = toolAxisT1 - toolAxisT0;
        Vector3 axis_tt = Vector3.zero;
        return MathUtility.NormalVectorDerivative2(axis, axis_t, axis_tt);
    }

    // Calculate normal of envelope according to Bassegoda's paper
    // Expects t in [0, 1] and a in [0, 1]
    public Vector3 CalculateNormal(float t, float a)
    {
        Vector3 sa = GetToolAxisAt(t);
        Vector3 st = GetToolPathDtAt(t) + a * GetToolAxisDtAt(t);
        Vector3 sNormal = Vector3.Cross(sa, st).normalized;

        float ra = GetToolRadiusDaAt(a);

        float E = Vector3.Dot(sa, sa);
        float F = Vector3.Dot(sa, st);
        float G = Vector3.Dot(st, st);
        float EG_FF = E * G - F * F;

        float m11 = G / EG_FF;
        float m21 = -F / EG_FF;

        float alpha = -m11 * ra;
        float beta = -m21 * ra;
        float gamma = (EG_FF > 0 ? 1 : -1) * Mathf.Sqrt(1 - ra * ra * m11);

        Vector3 n = alpha * sa + beta * st + gamma * sNormal;
        return n.normalized;
    }

    public Vector3 CalculateNormalDtAt(float t, float a)
    {
        return Vector3.zero;
    }

    public void UpdateEnvelope()
    {
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

    private void OnDrawGizmosSelected()
    {
        if (Application.isPlaying)
        {
            // Vector3 p = GetToolPathAt(t);
            // Vector3 axis = GetToolAxisAt(t);
            // Vector3 axis_t = GetToolAxisDtAt(t);
            // Vector3 s = p + a * axis;
            // Vector3 x = GetEnvelopeAt(t, a);
            // Vector3 n = CalculateNormal(t, a);
            // Vector3 nt = CalculateNormalDtAt(t, a);
            // // Axis
            // Gizmos.color = Color.blue;
            // Gizmos.DrawLine(p, p + axis);
            // // Axis derivative
            // Gizmos.DrawLine(p, p + axis_t);
            // // Normal
            // Gizmos.color = Color.green;
            // Gizmos.DrawLine(s, s - n);
            // // Normal derivative
            // Gizmos.DrawLine(s, s - nt);

        }
    }

    private void OnValidate()
    {
        if (adjacentEnvelopeA0 == this) adjacentEnvelopeA0 = null;
        if (adjacentEnvelopeA1 == this || adjacentEnvelopeA1 == adjacentEnvelopeA0) adjacentEnvelopeA1 = null;
        if (Application.isPlaying)
        {
            // Debug.Log("axis . axis_t: " + Vector3.Dot(GetToolAxisAt(t), GetToolAxisDtAt(t)));
            // Debug.Log("normal . normal_t: " + Vector3.Dot(CalculateNormal(t, a), CalculateNormalDtAt(t, a)));
        }
    }
}
