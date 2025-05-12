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
    [SerializeField]
    private float toolHeight = 1;
    private Tool tool;

    [Header("Constraints")]
    [SerializeField]
    private Envelope adjacentEnvelopeA0;
    [SerializeField]
    private Envelope adjacentEnvelopeA1;
    [SerializeField]
    private bool tangentContinuity;
    [SerializeField]
    private float toolAxisDegreeT0;
    [SerializeField]
    private float toolAxisDegreeT1;

    [Header("Render")]
    [SerializeField]
    private int tSectors = 50;
    [SerializeField]
    private int aSectors = 20;
    [SerializeField]
    [Range(0, 1)]
    private float t = 0;
    public float getT() { return t; }
    [SerializeField]
    [Range(0, 1)]
    private float a = 0;
    public float getA() { return a; }
    [SerializeField]
    private bool showTool = true;
    [SerializeField]
    private bool showSphere = false;

    public bool IsPositionContinuous { get { return adjacentEnvelopeA0 != null; } }
    public bool IsTangentContinuous { get { return IsPositionContinuous && tangentContinuity; } }
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
        // This function should ideally be on the Tool class, but keep it here for now for convenience
        tool.gameObject.SetActive(showTool);
        if (!showTool) return;
        tool.transform.localScale = new Vector3(toolRadius * 2, toolHeight, toolRadius * 2);
        tool.transform.localPosition = GetToolPathAt(t);
        tool.transform.rotation = Quaternion.LookRotation(GetToolAxisAt(t)) * Quaternion.FromToRotation(Vector3.up, Vector3.forward);
    }

    private void UpdateSphere()
    {
        // This function should ideally be on the Sphere, but keep it here for now for convenience
        if (sphere == null) return;
        sphere.SetActive(showSphere);
        sphere.transform.localScale = new Vector3(toolRadius * 2, toolRadius * 2, toolRadius * 2);
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
        return data;
    }

    public Vector3 GetEnvelopeAt(float t, float a)
    {
        return GetToolPathAt(t) + a * GetToolAxisAt(t) - GetToolRadiusAt(a) * CalculateNormalAt(t, a);
    }

    public Vector3 GetEnvelopeDtAt(float t, float a)
    {
        return GetToolPathDtAt(t) + a * GetToolAxisDtAt(t) - GetToolRadiusAt(a) * CalculateNormalDtAt(t, a);
    }

    public Vector3 GetEnvelopeDt2At(float t, float a)
    {
        return GetToolPathDt2At(t) + a * GetToolAxisDt2At(t) - GetToolRadiusAt(a) * CalculateNormalDt2At(t, a);
    }

    public Vector3 GetToolPathAt(float t)
    {
        if (IsTangentContinuous)
        {
            return adjacentEnvelopeA0.GetEnvelopeAt(t, 1) + GetToolRadiusAt(0) * adjacentEnvelopeA0.CalculateNormalAt(t, 1);
        }
        else if (IsPositionContinuous)
        {
            // TODO only works for cylindrical tool, due to the angle between the normal and the axis. General should be possible though
            Vector3 normal = Vector3.Cross(GetToolAxisAt(t), adjacentEnvelopeA0.GetEnvelopeDtAt(t, 1)).normalized;
            return adjacentEnvelopeA0.GetEnvelopeAt(t, 1) + GetToolRadiusAt(0) * normal;
        }
        else
        {
            return toolPath.Evaluate(t);
        }
    }

    public Vector3 GetToolPathDtAt(float t)
    {
        if (IsTangentContinuous)
        {
            return adjacentEnvelopeA0.GetEnvelopeDtAt(t, 1) + GetToolRadiusAt(0) * adjacentEnvelopeA0.CalculateNormalDtAt(t, 1);
        }
        else if (IsPositionContinuous)
        {
            // TODO only works for cylindrical tool, due to the angle between the normal and the axis. General should be possible though
            Vector3 axis = GetToolAxisAt(t);
            Vector3 axis_t = GetToolAxisDtAt(t);
            Vector3 adjEnv_t = adjacentEnvelopeA0.GetEnvelopeDtAt(t, 1);
            Vector3 adjEnv_tt = adjacentEnvelopeA0.GetEnvelopeDt2At(t, 1);
            Vector3 normal = Vector3.Cross(axis, adjEnv_t);
            Vector3 normal_t = Vector3.Cross(axis_t, adjEnv_t) + Vector3.Cross(axis, adjEnv_tt);
            normal_t = MathUtility.NormalVectorDerivative(normal, normal_t);
            return adjEnv_t + GetToolRadiusAt(0) * normal_t;
        }
        else
        {
            return toolPath.EvaluateDerivative(t);
        }
    }

    public Vector3 GetToolPathDt2At(float t)
    {
        // Need to check if these are required for chaining position continuous envelopes
        return toolPath.EvaluateDerivative2(t);
    }

    public Vector3 GetToolPathDt3At(float t)
    {
        // Need to check if these are required for chaining position continuous envelopes
        return toolPath.EvaluateDerivative3(t);
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

    /// <summary>
    /// Calculates the rotation of the tool axis when the envelope is tangent continuous.
    /// </summary>
    /// <param name="t"></param>
    /// <returns></returns>
    public Quaternion CalculateToolAxisRotationAt(float t)
    {
        if (!IsTangentContinuous) return Quaternion.identity;
        // First rotate w.r.t. tangent continuity. Which rotates around the cross product of the adjacent normal and axis
        float degrees = Mathf.Rad2Deg * Mathf.Acos(-GetToolRadiusDaAt(0));
        Vector3 adjNormal = adjacentEnvelopeA0.CalculateNormalAt(t, 1);
        Vector3 adjAxis = adjacentEnvelopeA0.GetToolAxisAt(t);
        Vector3 rotationAxis = Vector3.Cross(adjNormal, adjAxis);
        Quaternion rotation = Quaternion.AngleAxis(degrees, rotationAxis);

        // Then rotate w.r.t. the last freedom: around the normal of the previous envelope.
        float angle = Mathf.Lerp(toolAxisDegreeT0, toolAxisDegreeT1, t);
        Quaternion rotationFree = Quaternion.AngleAxis(angle, adjNormal);
        return rotationFree * rotation;
    }

    public Vector3 GetToolAxisAt(float t)
    {
        Vector3 axis;
        if (IsAxisConstrained)
        {
            axis = adjacentEnvelopeA1.GetEnvelopeAt(t, 0) - adjacentEnvelopeA0.GetEnvelopeAt(t, 1);
            axis.Normalize();
        }
        else if (IsTangentContinuous)
        {
            // Rotate the normal of the adjacent envelope around the cross product with the axis of the adjacent envelope
            axis = CalculateToolAxisRotationAt(t) * adjacentEnvelopeA0.CalculateNormalAt(t, 1);
        }
        else
        {
            axis = Vector3.Lerp(toolAxisT0, toolAxisT1, t);
            axis.Normalize();
        }
        return axis;
    }

    public Vector3 GetToolAxisDtAt(float t)
    {
        Vector3 axis, axis_t;
        if (IsAxisConstrained)
        {
            axis = adjacentEnvelopeA1.GetEnvelopeAt(t, 0) - adjacentEnvelopeA0.GetEnvelopeAt(t, 1);
            axis_t = adjacentEnvelopeA1.GetEnvelopeDtAt(t, 0) - adjacentEnvelopeA0.GetEnvelopeDtAt(t, 1);
            axis_t = MathUtility.NormalVectorDerivative(axis, axis_t);
        }
        else if (IsTangentContinuous)
        {
            // TODO There is still a slight div by 0 error here. 
            axis = GetToolAxisAt(t);
            Vector3 adjNormal = adjacentEnvelopeA0.CalculateNormalAt(t, 1);
            Vector3 adjNormal_t = adjacentEnvelopeA0.CalculateNormalDtAt(t, 1);
            axis_t.x = axis.x * adjNormal_t.x;
            if (axis_t.x != 0) axis_t.x /= adjNormal.x;

            axis_t.y = axis.y * adjNormal_t.y;
            if (axis_t.y != 0) axis_t.y /= adjNormal.y;

            axis_t.z = axis.z * adjNormal_t.z;
            if (axis_t.z != 0) axis_t.z /= adjNormal.z;

            axis_t *= -1;
        }
        else
        {
            axis = Vector3.Lerp(toolAxisT0, toolAxisT1, t);
            axis_t = toolAxisT1 - toolAxisT0;
            axis_t = MathUtility.NormalVectorDerivative(axis, axis_t);
        }
        return axis_t;
    }

    public Vector3 GetToolAxisDt2At(float t)
    {
        // TODO should find a solution for the axis constrained and tangent continuous cases. For now works, but only in narrow cases.
        Vector3 axis = Vector3.Lerp(toolAxisT0, toolAxisT1, t);
        Vector3 axis_t = toolAxisT1 - toolAxisT0;
        Vector3 axis_tt = Vector3.zero;
        return MathUtility.NormalVectorDerivative2(axis, axis_t, axis_tt);
    }

    public Vector3 GetToolAxisDt3At(float t)
    {
        // TODO should find a solution for the axis constrained and tangent continuous cases. For now works, but only in narrow cases.
        Vector3 axis = Vector3.Lerp(toolAxisT0, toolAxisT1, t);
        Vector3 axis_t = toolAxisT1 - toolAxisT0;
        Vector3 axis_tt = Vector3.zero;
        Vector3 axis_ttt = Vector3.zero;
        return MathUtility.NormalVectorDerivative3(axis, axis_t, axis_tt, axis_ttt);
    }

    // Calculate normal of envelope according to Bassegoda's paper
    // Expects t in [0, 1] and a in [0, 1]
    public Vector3 CalculateNormalAt(float t, float a)
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
        Vector3 sa = GetToolAxisAt(t);
        Vector3 sat = GetToolAxisDtAt(t);
        Vector3 st = GetToolPathDtAt(t) + a * GetToolAxisDtAt(t);
        Vector3 stt = GetToolPathDt2At(t) + a * GetToolAxisDt2At(t);
        Vector3 sNormal = Vector3.Cross(sa, st).normalized;
        Vector3 sNormal_t = MathUtility.NormalVectorDerivative(Vector3.Cross(sa, st), Vector3.Cross(sat, st) + Vector3.Cross(sa, stt));

        float ra = GetToolRadiusDaAt(a);

        float E = Vector3.Dot(sa, sa);
        float Et = 2 * Vector3.Dot(sa, sat);
        float F = Vector3.Dot(sa, st);
        float Ft = Vector3.Dot(sat, st) + Vector3.Dot(sa, stt);
        float G = Vector3.Dot(st, st);
        float Gt = 2 * Vector3.Dot(st, stt);
        float EG_FF = E * G - F * F;
        float EG_FF_t = Et * G + E * Gt - 2 * F * Ft;

        float m11 = G / EG_FF;
        float m11_t = (EG_FF * Gt - G * EG_FF_t) / (EG_FF * EG_FF);
        float m21 = -F / EG_FF;
        float m21_t = -(EG_FF * Ft - F * EG_FF_t) / (EG_FF * EG_FF);

        float alpha = -m11 * ra;
        float alpha_t = -m11_t * ra;
        float beta = -m21 * ra;
        float beta_t = -m21_t * ra;
        float gamma = (EG_FF > 0 ? 1 : -1) * Mathf.Sqrt(1 - ra * ra * m11);
        float gamma_t = (EG_FF > 0 ? 1 : -1) * -ra * ra * m11_t / (2 * Mathf.Sqrt(1 - ra * ra * m11));

        Vector3 n = alpha * sa + beta * st + gamma * sNormal;
        Vector3 nt = alpha * sat + alpha_t * sa + beta * stt + beta_t * st + gamma * sNormal_t + gamma_t * sNormal;
        return MathUtility.NormalVectorDerivative(n, nt);
    }

    public Vector3 CalculateNormalDt2At(float t, float a)
    {
        Vector3 sa = GetToolAxisAt(t);
        Vector3 sat = GetToolAxisDtAt(t);
        Vector3 satt = GetToolAxisDt2At(t);
        Vector3 st = GetToolPathDtAt(t) + a * GetToolAxisDtAt(t);
        Vector3 stt = GetToolPathDt2At(t) + a * GetToolAxisDt2At(t);
        Vector3 sttt = GetToolPathDt3At(t) + a * GetToolAxisDt3At(t);
        Vector3 sNormal = Vector3.Cross(sa, st);
        Vector3 sNormal_t = Vector3.Cross(sat, st) + Vector3.Cross(sa, stt);
        Vector3 sNormal_tt = Vector3.Cross(satt, st) + Vector3.Cross(sat, stt) + Vector3.Cross(sat, stt) + Vector3.Cross(sa, sttt);
        sNormal_tt = MathUtility.NormalVectorDerivative2(sNormal, sNormal_t, sNormal_tt);
        sNormal_t = MathUtility.NormalVectorDerivative(sNormal, sNormal_t);
        sNormal.Normalize();

        float ra = GetToolRadiusDaAt(a);

        float E = Vector3.Dot(sa, sa);
        float Et = 2 * Vector3.Dot(sa, sat);
        float Ett = 2 * (Vector3.Dot(sat, sat) + Vector3.Dot(sa, satt));
        float F = Vector3.Dot(sa, st);
        float Ft = Vector3.Dot(sat, st) + Vector3.Dot(sa, stt);
        float Ftt = Vector3.Dot(satt, st) + Vector3.Dot(sat, stt) + Vector3.Dot(sat, stt) + Vector3.Dot(sa, sttt);
        float G = Vector3.Dot(st, st);
        float Gt = 2 * Vector3.Dot(st, stt);
        float Gtt = 2 * (Vector3.Dot(stt, stt) + Vector3.Dot(st, sttt));
        float EG_FF = E * G - F * F;
        float EG_FF_2 = EG_FF * EG_FF;
        float EG_FF_t = Et * G + E * Gt - 2 * F * Ft;
        float EG_FF_2_t = 2 * EG_FF * EG_FF_t;
        float EG_FF_tt = Ett * G + Et * Gt + Et * Gt + E * Gtt - 2 * (Ft * Ft + F * Ftt);

        float m11 = G / EG_FF;
        float m11_t = (EG_FF * Gt - G * EG_FF_t) / EG_FF_2;
        float m11_tt = (EG_FF_2 * ((EG_FF_t * Gt + EG_FF * Gtt) - (Gt * EG_FF_t + G * EG_FF_tt)) - (EG_FF * Gt - G * EG_FF_t) * EG_FF_2_t) / (EG_FF_2 * EG_FF_2);
        float m21 = -F / EG_FF;
        float m21_t = -(EG_FF * Ft - F * EG_FF_t) / EG_FF_2;
        float m21_tt = -(EG_FF_2 * ((EG_FF_t * Ft + EG_FF * Ftt) - (Ft * EG_FF_t + F * EG_FF_tt)) - (EG_FF * Ft - F * EG_FF_t) * EG_FF_2_t) / (EG_FF_2 * EG_FF_2);

        float alpha = -m11 * ra;
        float alpha_t = -m11_t * ra;
        float alpha_tt = -m11_tt * ra;
        float beta = -m21 * ra;
        float beta_t = -m21_t * ra;
        float beta_tt = -m21_tt * ra;
        float gamma = (EG_FF > 0 ? 1 : -1) * Mathf.Sqrt(1 - ra * ra * m11);
        float gamma_t = (EG_FF > 0 ? 1 : -1) * -ra * ra * m11_t / (2 * Mathf.Sqrt(1 - ra * ra * m11));
        float gamma_tt = 0;

        Vector3 n = alpha * sa +
                    beta * st +
                    gamma * sNormal;
        Vector3 nt = alpha * sat + alpha_t * sa +
                     beta * stt + beta_t * st +
                     gamma * sNormal_t + gamma_t * sNormal;
        Vector3 ntt = alpha_t * sat + alpha * satt + alpha_tt * sa + alpha_t * sat +
                      beta_t * stt + beta * sttt + beta_tt * st + beta_t * st +
                      gamma_t * sNormal_t + gamma * sNormal_tt + gamma_tt * sNormal + gamma_t * sNormal_t;
        return MathUtility.NormalVectorDerivative2(n, nt, ntt);
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
            Vector3 p = GetToolPathAt(t);
            Vector3 axis = GetToolAxisAt(t);
            Vector3 axis_t = GetToolAxisDtAt(t);
            Vector3 aXat = -Vector3.Cross(axis, axis_t);
            Vector3 s = p + a * axis;
            Vector3 x = GetEnvelopeAt(t, a);
            Vector3 xt = GetEnvelopeDtAt(t, a);
            Vector3 n = CalculateNormalAt(t, a);
            Vector3 nt = CalculateNormalDtAt(t, a);
            // Axis
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(p, p + axis);
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(p, p + axis_t);
            // Gizmos.color = Color.magenta;
            // Gizmos.DrawLine(p, p + aXat);

            // Normal
            Gizmos.color = Color.green;
            Gizmos.DrawLine(s, s - n);
            Gizmos.color = Color.magenta;
            Gizmos.DrawLine(s, s - nt);

            // Cross products
            Gizmos.color = Color.black;
            Gizmos.DrawLine(s, s + Vector3.Cross(n, axis));
            Gizmos.DrawLine(s, s + Vector3.Cross(n, axis_t));

            // Envelope
            Gizmos.color = Color.red;
            Gizmos.DrawLine(x, x + xt);

            if (IsAxisConstrained)
            {
                Vector3 x1x2 = adjacentEnvelopeA1.GetEnvelopeAt(t, 0) - adjacentEnvelopeA0.GetEnvelopeAt(t, 1);
                Vector3 x1x2_t = adjacentEnvelopeA1.GetEnvelopeDtAt(t, 0) - adjacentEnvelopeA0.GetEnvelopeDtAt(t, 1);
                Gizmos.color = Color.black;
                Gizmos.DrawLine(p, p + x1x2);
                Gizmos.DrawLine(p, p + x1x2_t);
                Gizmos.color = Color.white;
                Gizmos.DrawLine(p, p + axis + aXat);
            }

        }
    }

    private void OnValidate()
    {
        if (adjacentEnvelopeA0 == this) adjacentEnvelopeA0 = null;
        if (adjacentEnvelopeA1 == this || adjacentEnvelopeA1 == adjacentEnvelopeA0) adjacentEnvelopeA1 = null;
        if (Application.isPlaying && IsPositionContinuous)
        {
            Debug.Log(Vector3.Angle(GetToolAxisAt(t), adjacentEnvelopeA0.GetToolAxisAt(t)));
        }
        if (Application.isPlaying && IsTangentContinuous)
        {
            Debug.Log("Angle A2 n1_t " + Vector3.Angle(GetToolAxisAt(t), -adjacentEnvelopeA0.CalculateNormalDtAt(t, 1)));
            Debug.Log("A2 . n1_t " + Vector3.Dot(GetToolAxisAt(t), -adjacentEnvelopeA0.CalculateNormalDtAt(t, 1)));
            Debug.Log("A2_t . n1 " + Vector3.Dot(GetToolAxisDtAt(t), -adjacentEnvelopeA0.CalculateNormalAt(t, 1)));
            Debug.Log("A2 . A2_t " + Vector3.Dot(GetToolAxisAt(t), GetToolAxisDtAt(t)));
        }
        if (Application.isPlaying && IsAxisConstrained)
        {
            Debug.Log(Vector3.Angle(adjacentEnvelopeA1.GetEnvelopeAt(t, 0) - adjacentEnvelopeA0.GetEnvelopeAt(t, 1), GetToolAxisAt(t) + Vector3.Cross(GetToolAxisAt(t), GetToolAxisDtAt(t))));
        }
    }
}
