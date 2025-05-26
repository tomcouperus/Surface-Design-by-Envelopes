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
    public Tool tool;

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
    [Min(1)]
    private int tSectors = 50;
    [SerializeField]
    [Min(1)]
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
        tool.transform.localPosition = GetToolPathAt(t);
        tool.transform.rotation = Quaternion.LookRotation(GetToolAxisAt(t)) * Quaternion.FromToRotation(Vector3.up, Vector3.forward);
    }

    private void UpdateSphere()
    {
        // This function should ideally be on the Sphere, but keep it here for now for convenience
        if (sphere == null) return;
        sphere.SetActive(showSphere);
        float r = tool.GetSphereRadiusAt(a);
        sphere.transform.localScale = new Vector3(r, r, r);
        sphere.transform.localPosition = GetToolPathAt(t) + tool.GetSphereCenterHeightAt(a) * GetToolAxisAt(t);
    }

    private MeshData GenerateMeshData()
    {
        Debug.Log("Generating mesh for " + gameObject.name);
        int vertexIndex = 0;
        MeshData data = new MeshData(tSectors + 1, aSectors + 1);
        for (int tIdx = 0; tIdx <= tSectors; tIdx++)
        {
            float t = (float)tIdx / tSectors;
            for (int aIdx = 0; aIdx <= aSectors; aIdx++)
            {
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
                    data.AddTriangle(v3, v2, v1);
                    data.AddTriangle(v3, v4, v2);
                }
                vertexIndex++;
            }
        }
        return data;
    }

    public Vector3 GetEnvelopeAt(float t, float a)
    {
        return GetToolPathAt(t) + tool.GetSphereCenterHeightAt(a) * GetToolAxisAt(t) + GetSphereRadiusAt(a) * CalculateNormalAt(t, a);
    }

    public Vector3 GetEnvelopeDtAt(float t, float a)
    {
        return GetToolPathDtAt(t) + tool.GetSphereCenterHeightAt(a) * GetToolAxisDtAt(t) + GetSphereRadiusAt(a) * CalculateNormalDtAt(t, a);
    }

    public Vector3 GetEnvelopeDt2At(float t, float a)
    {
        return GetToolPathDt2At(t) + tool.GetSphereCenterHeightAt(a) * GetToolAxisDt2At(t) + GetSphereRadiusAt(a) * CalculateNormalDt2At(t, a);
    }

    public Quaternion CalculateNormalRotation(float t)
    {
        // First, rotate the adjacent normal's frame to the cross product of the axis and the adjacent envelope's 
        // derivative. This is the normal in the case of a cylinder
        // Then rotate this normal around the adjacent envelope's derivative until the desired

        Vector3 axis = GetToolAxisAt(t);
        Vector3 adjEnv_t = adjacentEnvelopeA0.GetEnvelopeDtAt(t, 1);
        Vector3 axis_x_adjEnv_t = Vector3.Cross(axis, adjEnv_t);
        Vector3 adjNormal = adjacentEnvelopeA0.CalculateNormalAt(t, 1);
        Quaternion rotationFrame = Quaternion.FromToRotation(adjNormal, axis_x_adjEnv_t);

        float degrees = Mathf.Rad2Deg * Mathf.Acos(-GetSphereRadiusDaAt(0) / tool.GetSphereCenterHeightDaAt(0)) - 90;
        Vector3 rotationAxis = Vector3.Cross(adjNormal, axis_x_adjEnv_t);
        Quaternion rotation = Quaternion.AngleAxis(degrees, -adjEnv_t);

        return rotation * rotationFrame;
    }

    public Vector3 GetToolPathAt(float t)
    {
        if (IsTangentContinuous)
        {
            return adjacentEnvelopeA0.GetEnvelopeAt(t, 1) - GetSphereRadiusAt(0) * adjacentEnvelopeA0.CalculateNormalAt(t, 1) - tool.GetSphereCenterHeightAt(0) * GetToolAxisAt(t);
        }
        else if (IsPositionContinuous)
        {
            // The normal is orthogonal to X_t of the adjacent envelope, and at an angle to the axis of its own envelope.
            // Thus, start with the normal of the adjacent envelope (also orthogonal to X_t of the adjacent envelope by definition),
            // and rotate it around X_t of the adjacent envelope until the angle with the axis is achieved.
            // TODO does not actually work yet. Only works for cylindrical case for now, but that is enough for my purposes. If time allows, will expand.
            Vector3 adjEnv_t = adjacentEnvelopeA0.GetEnvelopeDtAt(t, 1);
            Vector3 axis = GetToolAxisAt(t);
            float dotValue = -GetSphereRadiusDaAt(0) / tool.GetSphereCenterHeightDaAt(0);
            Vector3 normal = MathUtility.VectorPerpAndDotProduct(adjEnv_t, axis, dotValue);
            return adjacentEnvelopeA0.GetEnvelopeAt(t, 1) - tool.GetSphereRadiusAt(0) * normal - tool.GetSphereCenterHeightAt(0) * GetToolAxisAt(t);
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
            return adjacentEnvelopeA0.GetEnvelopeDtAt(t, 1) - GetSphereRadiusAt(0) * adjacentEnvelopeA0.CalculateNormalDtAt(t, 1) - tool.GetSphereCenterHeightAt(0) * GetToolAxisDtAt(t);
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
            // Vector3 adjEnv_t = adjacentEnvelopeA0.GetEnvelopeDtAt(t, 1);
            // Vector3 normal_t = CalculateNormalRotation(t) * adjacentEnvelopeA0.CalculateNormalDtAt(t, 1);
            return adjEnv_t - tool.GetRadiusAt(0) * normal_t - tool.GetSphereCenterHeightAt(0) * GetToolAxisDtAt(t);
        }
        else
        {
            return toolPath.EvaluateDerivative(t);
        }
    }

    public Vector3 GetToolPathDt2At(float t)
    {
        // Need to check if these are required for chaining position continuous envelopes
        if (IsTangentContinuous)
        {
            return adjacentEnvelopeA0.GetEnvelopeDt2At(t, 1) - GetSphereRadiusAt(0) * adjacentEnvelopeA0.CalculateNormalDt2At(t, 1) - tool.GetSphereCenterHeightAt(0) * GetToolAxisDt2At(t);
        }
        else
        {
            return toolPath.EvaluateDerivative2(t);
        }
    }

    public Vector3 GetToolPathDt3At(float t)
    {
        // Need to check if these are required for chaining position continuous envelopes
        return toolPath.EvaluateDerivative3(t);
    }

    public float GetSphereRadiusAt(float a)
    {
        return tool.GetSphereRadiusAt(a);
    }

    public float GetSphereRadiusDaAt(float a)
    {
        return tool.GetSphereRadiusDaAt(a);
    }

    /// <summary>
    /// Calculates a quaternion that rotates the axis of the constraining envelope in such a way to achieve tangent continuity for this envelope.
    /// </summary>
    /// <param name="t"></param>
    /// <returns></returns>
    public Quaternion CalculateToolAxisRotationAt(float t)
    {
        if (!IsTangentContinuous) return Quaternion.identity;
        // First rotate the axis of the previous envelope to its normal. 
        // This is to establish a frame of reference for all its derivatives. 
        Vector3 adjNormal = adjacentEnvelopeA0.CalculateNormalAt(t, 1);
        Vector3 adjAxis = adjacentEnvelopeA0.GetToolAxisAt(t);
        Quaternion rotationFrame = Quaternion.FromToRotation(adjAxis, adjNormal);

        // Then rotate w.r.t. tangent continuity. Which rotates around the cross product of the adjacent normal and axis
        float degrees = Mathf.Rad2Deg * Mathf.Acos(-GetSphereRadiusDaAt(0) / tool.GetSphereCenterHeightDaAt(0));
        Vector3 rotationAxis = Vector3.Cross(adjNormal, adjAxis);
        Quaternion rotationTangent = Quaternion.AngleAxis(degrees, rotationAxis);

        // Lastly rotate w.r.t. the last freedom: around the normal of the previous envelope.
        float angle = Mathf.Lerp(toolAxisDegreeT0, toolAxisDegreeT1, t);
        Quaternion rotationFree = Quaternion.AngleAxis(angle, adjNormal);
        return rotationFree * rotationTangent * rotationFrame;
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
            axis = CalculateToolAxisRotationAt(t) * adjacentEnvelopeA0.GetToolAxisAt(t);
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
            axis_t = CalculateToolAxisRotationAt(t) * adjacentEnvelopeA0.GetToolAxisDtAt(t);
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
        Vector3 axis, axis_t, axis_tt;
        // TODO should find a solution for the axis constrained and tangent continuous cases. For now works, but only in narrow cases.
        if (IsTangentContinuous)
        {
            axis_tt = CalculateToolAxisRotationAt(t) * adjacentEnvelopeA0.GetToolAxisDt2At(t);
        }
        else
        {
            axis = Vector3.Lerp(toolAxisT0, toolAxisT1, t);
            axis_t = toolAxisT1 - toolAxisT0;
            axis_tt = Vector3.zero;
            axis_tt = MathUtility.NormalVectorDerivative2(axis, axis_t, axis_tt);
        }
        return axis_tt;
    }

    public Vector3 GetToolAxisDt3At(float t)
    {
        // TODO should find a solution for the axis constrained and tangent continuous cases. For now works, but only in narrow cases.
        Vector3 axis, axis_t, axis_tt, axis_ttt;
        if (IsTangentContinuous)
        {
            axis_ttt = CalculateToolAxisRotationAt(t) * adjacentEnvelopeA0.GetToolAxisDt3At(t);
        }
        else
        {
            axis = Vector3.Lerp(toolAxisT0, toolAxisT1, t);
            axis_t = toolAxisT1 - toolAxisT0;
            axis_tt = Vector3.zero;
            axis_ttt = Vector3.zero;
            axis_ttt = MathUtility.NormalVectorDerivative3(axis, axis_t, axis_tt, axis_ttt);
        }
        return axis_ttt;
    }

    // Calculate normal of envelope according to Bassegoda's paper
    // Expects t in [0, 1] and a in [0, 1]
    public Vector3 CalculateNormalAt(float t, float a)
    {
        Vector3 sa = tool.GetSphereCenterHeightDaAt(a) * GetToolAxisAt(t);
        Vector3 st = GetToolPathDtAt(t) + tool.GetSphereCenterHeightAt(a) * GetToolAxisDtAt(t);
        Vector3 sNormal = Vector3.Cross(sa, st).normalized;

        float ra = GetSphereRadiusDaAt(a);

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
        Vector3 sa = tool.GetSphereCenterHeightDaAt(a) * GetToolAxisAt(t);
        Vector3 sat = tool.GetSphereCenterHeightDaAt(a) * GetToolAxisDtAt(t);
        Vector3 st = GetToolPathDtAt(t) + tool.GetSphereCenterHeightAt(a) * GetToolAxisDtAt(t);
        Vector3 stt = GetToolPathDt2At(t) + tool.GetSphereCenterHeightAt(a) * GetToolAxisDt2At(t);
        Vector3 sNormal = Vector3.Cross(sa, st).normalized;
        Vector3 sNormal_t = MathUtility.NormalVectorDerivative(Vector3.Cross(sa, st), Vector3.Cross(sat, st) + Vector3.Cross(sa, stt));

        float ra = GetSphereRadiusDaAt(a);

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
        Vector3 sa = tool.GetSphereCenterHeightDaAt(a) * GetToolAxisAt(t);
        Vector3 sat = tool.GetSphereCenterHeightDaAt(a) * GetToolAxisDtAt(t);
        Vector3 satt = tool.GetSphereCenterHeightDaAt(a) * GetToolAxisDt2At(t);
        Vector3 st = GetToolPathDtAt(t) + tool.GetSphereCenterHeightAt(a) * GetToolAxisDtAt(t);
        Vector3 stt = GetToolPathDt2At(t) + tool.GetSphereCenterHeightAt(a) * GetToolAxisDt2At(t);
        Vector3 sttt = GetToolPathDt3At(t) + tool.GetSphereCenterHeightAt(a) * GetToolAxisDt3At(t);
        Vector3 sNormal = Vector3.Cross(sa, st);
        Vector3 sNormal_t = Vector3.Cross(sat, st) + Vector3.Cross(sa, stt);
        Vector3 sNormal_tt = Vector3.Cross(satt, st) + Vector3.Cross(sat, stt) + Vector3.Cross(sat, stt) + Vector3.Cross(sa, sttt);
        sNormal_tt = MathUtility.NormalVectorDerivative2(sNormal, sNormal_t, sNormal_tt);
        sNormal_t = MathUtility.NormalVectorDerivative(sNormal, sNormal_t);
        sNormal.Normalize();

        float ra = GetSphereRadiusDaAt(a);

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
        float gamma_tt = (EG_FF > 0 ? 1 : -1) *
                         (2 * Mathf.Sqrt(1 - ra * ra * m11) * -ra * ra * m11_tt - ra * ra * ra * ra * m11_t * m11_t / Mathf.Sqrt(1 - ra * ra * m11)) /
                         (4 - 4 * ra * ra * m11);

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

    private void OnDrawGizmos()
    {
        if (Application.isPlaying)
        {
            Vector3 p = GetToolPathAt(t);
            Vector3 axis = GetToolAxisAt(t);
            Vector3 axis_t = GetToolAxisDtAt(t);
            Vector3 aXat = -Vector3.Cross(axis, axis_t);
            Vector3 s = p + tool.GetSphereCenterHeightAt(a) * axis;
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
            Gizmos.DrawLine(s, s + n);
            Gizmos.color = Color.magenta;
            Gizmos.DrawLine(s, s + nt);

            // Cross products
            Gizmos.color = Color.black;
            // Gizmos.DrawLine(s, s + Vector3.Cross(n, axis));
            // Gizmos.DrawLine(s, s + Vector3.Cross(n, axis_t));

            // Envelope
            Gizmos.color = Color.red;
            Gizmos.DrawLine(x, x + xt);

            if (IsPositionContinuous)
            {
                Vector3 adjN_1 = adjacentEnvelopeA0.CalculateNormalAt(t, 1);
                Vector3 adjEnv = adjacentEnvelopeA0.GetEnvelopeAt(t, 1);
                Vector3 adjEnv_t = adjacentEnvelopeA0.GetEnvelopeDtAt(t, 1);
                float dotValue = -GetSphereRadiusDaAt(0) / tool.GetSphereCenterHeightDaAt(0);
                Vector3 n0_approx = MathUtility.VectorPerpAndDotProduct(adjEnv_t, axis, dotValue);
                Gizmos.color = Color.black;
                Gizmos.DrawLine(adjEnv, adjEnv - n0_approx);


            }

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
        if (Application.isPlaying)
        {
            Vector3 p = GetToolPathAt(t);
            Vector3 axis = GetToolAxisAt(t);
            Vector3 axis_t = GetToolAxisDtAt(t);
            Vector3 aXat = Vector3.Cross(axis, axis_t);
            Vector3 s = p + a * axis;
            Vector3 x = GetEnvelopeAt(t, a);
            Vector3 xt = GetEnvelopeDtAt(t, a);
            Vector3 n = CalculateNormalAt(t, a);
            Vector3 n0 = CalculateNormalAt(t, 0);
            Vector3 n1 = CalculateNormalAt(t, 1);
            Vector3 nt = CalculateNormalDtAt(t, a);

            Debug.Log(axis + " A(t)");
            Debug.Log(xt + " X_t(t,a)");

            if (IsPositionContinuous)
            {
                Vector3 adj0_n_1 = adjacentEnvelopeA0.CalculateNormalAt(t, 1);
                Vector3 adj0_n_t_1 = adjacentEnvelopeA0.CalculateNormalDtAt(t, 1);

                Vector3 adj0_env_1 = adjacentEnvelopeA0.GetEnvelopeAt(t, 1);
                Vector3 adj0_env_t_1 = adjacentEnvelopeA0.GetEnvelopeDtAt(t, 1);

                Debug.LogWarning("G0 debug");
                Debug.Log(Vector3.Angle(n0, axis) + " Angle n(t,0) A(t)");
                Debug.Log(Vector3.Angle(n0, adj0_env_t_1) + " Angle n(t,0) X0_t(t,1)");
                if (IsAxisConstrained)
                {
                    Debug.LogWarning("Connect G0 debug");
                    Vector3 adj1_env_0 = adjacentEnvelopeA1.GetEnvelopeAt(t, 0);
                    Vector3 adj1_env_t_0 = adjacentEnvelopeA1.GetEnvelopeDtAt(t, 0);
                    Debug.Log(Vector3.Angle(adj1_env_0 - adj0_env_1, axis + aXat));

                    Debug.Log(Vector3.Angle(n0, axis) + " Angle n(t,0) A(t)");
                    Debug.Log(Vector3.Angle(n1, axis) + " Angle n(t,1) A(t)");
                    Debug.Log(Vector3.Angle(n0, adj0_env_t_1) + " Angle n(t,0) X0_t(t,1)");
                    Debug.Log(Vector3.Angle(n1, adj1_env_t_0) + " Angle n(t,1) X1_t(t,0)");
                }
                else if (IsTangentContinuous)
                {
                }
            }
        }
    }
}
