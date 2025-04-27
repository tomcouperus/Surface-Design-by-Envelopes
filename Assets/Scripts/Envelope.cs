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
    private float toolRadius = 1.0f;
    // [SerializeField]
    // private float toolHeight = 2;
    private Tool tool;

    [Header("Constraints")]
    [SerializeField]
    private Envelope adjacentEnvelopeA0;
    [SerializeField]
    private Envelope adjacentEnvelopeA1;
    [SerializeField]
    public bool perfectFit = false;

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
        if (perfectFit && perfectFitToolAxes == null) return;
        tool.transform.localPosition = GetToolPathAt(t);
        tool.transform.rotation = Quaternion.LookRotation(GetToolAxisAt(t)) * Quaternion.FromToRotation(Vector3.up, Vector3.forward);
    }

    private void UpdateSphere()
    {
        if (sphere == null) return;
        sphere.SetActive(showSphere);
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
        return GetToolPathAt(t) + a * GetToolAxisAt(t) - GetToolRadiusAt(a) * CalculateNormal(t, a);
    }

    public Vector3 GetEnvelopeDtAt(float t, float a)
    {
        return GetToolPathDtAt(t) + a * GetToolAxisDtAt(t) - GetToolRadiusAt(a) * CalculateNormalDtAt(t, a);
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
        if (IsAxisConstrained)
        {
            if (perfectFit)
            {
                axis = perfectFitToolAxes[(int)(t * tSectors)];
            }
            else
            {
                axis = adjacentEnvelopeA1.GetEnvelopeAt(t, 0) - adjacentEnvelopeA0.GetEnvelopeAt(t, 1);
            }
        }
        return axis.normalized;
    }

    public Vector3 GetToolAxisDtAt(float t)
    {
        if (!IsAxisConstrained)
        {
            Vector3 axisLerp = Vector3.Lerp(toolAxisT0, toolAxisT1, t);
            Vector3 axisLerpDeriv = toolAxisT1 - toolAxisT0;
            return MathUtility.NormalVectorDerivative(axisLerp, axisLerpDeriv);
        }
        else
        {
            Vector3 axis = adjacentEnvelopeA1.GetEnvelopeAt(t, 0) - adjacentEnvelopeA0.GetEnvelopeAt(t, 1);
            Vector3 axis_t = adjacentEnvelopeA1.GetEnvelopeDtAt(t, 0) - adjacentEnvelopeA0.GetEnvelopeDtAt(t, 1);
            return MathUtility.NormalVectorDerivative(axis, axis_t);
        }
    }

    public Vector3 GetToolAxisDt2At(float t)
    {
        if (!IsAxisConstrained)
        {
            Vector3 axisLerp = Vector3.Lerp(toolAxisT0, toolAxisT1, t);
            Vector3 axisLerpDeriv = toolAxisT1 - toolAxisT0;
            return MathUtility.NormalVectorDerivative2(axisLerp, axisLerpDeriv, Vector3.zero);
        }
        else
        {
            Debug.LogError("2nd derivative of constrained axis");
            return Vector3.zero;
        }
    }

    private Vector3[] perfectFitToolAxes;
    public void CalculatePerfectFitToolAxes()
    {
        perfectFitToolAxes = new Vector3[tSectors + 1];
        float sPrevious = 0;
        for (int tIdx = 0; tIdx <= tSectors; tIdx++)
        {
            if (sPrevious > 1)
            {
                Debug.LogError("Unable to find perfectly fitting axes");
                break;
            }
            float s = sPrevious;
            float t = (float)tIdx / tSectors;
            Vector3 x1 = adjacentEnvelopeA0.GetEnvelopeAt(t, 1);
            Vector3 x3 = adjacentEnvelopeA1.GetEnvelopeAt(s, 0);
            float d = 1;
            float sqrDiff = (x3 - x1).sqrMagnitude - d * d;
            float deltaS = 1.0f / (tSectors * 50);
            while (s <= 1)
            {
                Debug.Log(sqrDiff);
                if (sqrDiff < 1e-5)
                {
                    perfectFitToolAxes[tIdx] = x3 - x1;
                    sPrevious = s;
                    break;
                }
                x3 = adjacentEnvelopeA1.GetEnvelopeAt(s + deltaS, 0);
                sqrDiff = (x3 - x1).sqrMagnitude - d * d;
                s += deltaS;
            }
        }
    }

    // Calculate normal of envelope according to Bassegoda's paper
    // Expects t in [0, 1] and a in [0, 1]
    public Vector3 CalculateNormal(float t, float a)
    {
        // tool surface derivative wrt a
        Vector3 sa = /* toolHeight * */ GetToolAxisAt(t);
        // tool surface derivative wrt t
        Vector3 st = GetToolPathDtAt(t) + a /* * toolHeight */ * GetToolAxisDtAt(t);
        Vector3 sNormal = Vector3.Cross(sa, st).normalized;

        // tool radius derivate wrt a
        float ra = GetToolRadiusDaAt(a);
        float alpha, beta, gamma;
        // First fundamental form for the various dot products
        float E = Vector3.Dot(sa, sa);
        float F = Vector3.Dot(sa, st);
        float G = Vector3.Dot(st, st);
        // Determinant of partial derivative matrix
        float EG_FF = E * G - F * F;
        float m11 = G / EG_FF;
        alpha = m11 * -ra;
        float m21 = -F / EG_FF;
        beta = m21 * -ra;
        gamma = (EG_FF > 0 ? 1 : -1) * Mathf.Sqrt(1 - ra * ra * m11);

        Vector3 normal = alpha * sa + beta * st + gamma * sNormal;
        return normal.normalized;
    }

    public Vector3 CalculateNormalDtAt(float t, float a)
    {
        Vector3 sa = GetToolAxisAt(t);
        Vector3 sat = GetToolAxisDtAt(t);
        Vector3 st = GetToolPathDtAt(t) + a * GetToolAxisDtAt(t);
        Vector3 stt = GetToolPathDt2At(t) + a * GetToolAxisDt2At(t);

        Vector3 sNormal = Vector3.Cross(sa, st).normalized;
        Vector3 sNormalt = MathUtility.NormalVectorDerivative(Vector3.Cross(sa, st), Vector3.Cross(sat, st) + Vector3.Cross(sa, stt));

        float ra = GetToolRadiusDaAt(a);

        float E = Vector3.Dot(sa, sa);
        float Et = 2 * Vector3.Dot(sa, sat);
        float F = Vector3.Dot(sa, st);
        float Ft = Vector3.Dot(sat, st) + Vector3.Dot(sa, stt);
        float G = Vector3.Dot(st, st);
        float Gt = 2 * Vector3.Dot(st, stt);

        float EG_FF = E * G - F * F;
        float EG_FF_t = Et * G + E * Gt - 2 * F * Ft;

        float alpha, alpha_t;
        float m11 = G / EG_FF;
        float m11_t = (EG_FF * Gt - G * EG_FF_t) / (EG_FF * EG_FF);
        alpha = m11 * -ra;
        alpha_t = m11_t * -ra;

        float beta, beta_t;
        float m21 = -F / EG_FF;
        float m21_t = -(EG_FF * Ft - F * EG_FF_t) / (EG_FF * EG_FF);
        beta = m21 * -ra;
        beta_t = m21_t * -ra;

        float gamma, gamma_t;
        gamma = (EG_FF > 0 ? 1 : -1) * Mathf.Sqrt(1 - ra * ra * m11);
        gamma_t = (EG_FF > 0 ? 1 : -1) * -ra * ra * m11_t / (2 * Mathf.Sqrt(1 - ra * ra * m11));

        Vector3 normal = alpha * sa + beta * st + gamma * sNormal;
        Vector3 normalDeriv = alpha * sat + alpha_t * sa + beta * stt + beta_t * st + gamma * sNormalt + gamma_t * sNormal;
        return MathUtility.NormalVectorDerivative(normal, normalDeriv);
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
            float distance = 0;
            for (int tIdx = 0; tIdx <= tSectors; tIdx++)
            {
                float t = (float)tIdx / tSectors;
                Vector3 axis = adjacentEnvelopeA1.GetEnvelopeAt(t, 0) - adjacentEnvelopeA0.GetEnvelopeAt(t, 1);
                if (axis.sqrMagnitude > 1 + 1e-5/* toolHeight * toolHeight */)
                {
                    distance = Mathf.Max(distance, axis.magnitude - 1);
                }
            }
            if (distance > 0)
            {
                Debug.LogWarning(gameObject.name + ": Constraining envelopes too far apart by " + distance);
            }
        }
    }

    private void OnDrawGizmosSelected()
    {
        if (Application.isPlaying)
        {
            Vector3 p = GetToolPathAt(t);
            Vector3 axis = GetToolAxisAt(t);
            Vector3 axis_t = GetToolAxisDtAt(t);
            Vector3 s = p + a * axis;
            Vector3 x = GetEnvelopeAt(t, a);
            Vector3 n = CalculateNormal(t, a);
            Vector3 nt = CalculateNormalDtAt(t, a);
            // Axis
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(p, p + axis);
            // Axis derivative
            Gizmos.DrawLine(p, p + axis_t);
            // Normal
            Gizmos.color = Color.green;
            Gizmos.DrawLine(s, s - n);
            // Normal derivative
            Gizmos.DrawLine(s, s - nt);

        }
    }

    private void OnValidate()
    {
        if (adjacentEnvelopeA0 == this) adjacentEnvelopeA0 = null;
        if (adjacentEnvelopeA1 == this || adjacentEnvelopeA1 == adjacentEnvelopeA0) adjacentEnvelopeA1 = null;
        if (Application.isPlaying)
        {
            Debug.Log("axis . axis_t: " + Vector3.Dot(GetToolAxisAt(t), GetToolAxisDtAt(t)));
            Debug.Log("normal . normal_t: " + Vector3.Dot(CalculateNormal(t, a), CalculateNormalDtAt(t, a)));
        }
    }
}
