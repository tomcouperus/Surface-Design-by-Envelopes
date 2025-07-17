using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using Palmmedia.ReportGenerator.Core.Reporting.Builders.Rendering;
using UnityEngine;

[RequireComponent(typeof(MeshRenderer))]
[RequireComponent(typeof(MeshFilter))]
public class Envelope : MonoBehaviour
{
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
    public bool IsTangentContinuous { get { return IsPositionContinuous && tangentContinuity && !IsAxisConstrained; } }
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
        return GetToolPathAt(t) + tool.GetSphereCenterHeightAt(a) * GetToolAxisAt(t) + tool.GetSphereRadiusAt(a) * CalculateNormalAt(t, a);
    }

    public Vector3 GetEnvelopeDtAt(float t, float a)
    {
        return GetToolPathDtAt(t) + tool.GetSphereCenterHeightAt(a) * GetToolAxisDtAt(t) + tool.GetSphereRadiusAt(a) * CalculateNormalDtAt(t, a);
    }

    public Vector3 GetEnvelopeDt2At(float t, float a)
    {
        return GetToolPathDt2At(t) + tool.GetSphereCenterHeightAt(a) * GetToolAxisDt2At(t) + tool.GetSphereRadiusAt(a) * CalculateNormalDt2At(t, a);
    }

    public Vector3 GetEnvelopeDt3At(float t, float a)
    {
        return GetToolPathDt3At(t) + tool.GetSphereCenterHeightAt(a) * GetToolAxisDt3At(t) + tool.GetSphereRadiusAt(a) * CalculateNormalDt3At(t, a);
    }

    public Vector3 GetToolPathAt(float t)
    {
        if (IsTangentContinuous)
        {
            return adjacentEnvelopeA0.GetEnvelopeAt(t, 1) - tool.GetSphereRadiusAt(0) * adjacentEnvelopeA0.CalculateNormalAt(t, 1) - tool.GetSphereCenterHeightAt(0) * GetToolAxisAt(t);
        }
        else if (IsPositionContinuous)
        {
            // The normal is orthogonal to X_t of the adjacent envelope, and at an angle to the axis of its own envelope.
            // Thus start with the cross product of these two, and rotate it a certain amount around X_t of the adjacent.
            Vector3 adjEnv = adjacentEnvelopeA0.GetEnvelopeAt(t, 1);
            Vector3 adjEnv_t = adjacentEnvelopeA0.GetEnvelopeDtAt(t, 1);
            Vector3 axis = GetToolAxisAt(t);
            float dotValue = -tool.GetSphereRadiusDaAt(0) / tool.GetSphereCenterHeightDaAt(0);

            // MATH
            Vector3 v = adjEnv_t.normalized;
            Vector3 p = Vector3.up;
            if (p == v) p = Vector3.right;
            Vector3 w1 = (p - Vector3.Dot(p, v) * v).normalized;
            Vector3 w2 = Vector3.Cross(v, w1);
            float a_dot_w1 = Vector3.Dot(axis, w1);
            float a_dot_w2 = Vector3.Dot(axis, w2);

            float phi = Mathf.Atan2(a_dot_w2, a_dot_w1);
            float theta = phi - Mathf.Acos(dotValue / Mathf.Sqrt(a_dot_w1 * a_dot_w1 + a_dot_w2 * a_dot_w2));

            Vector3 normal = w1 * Mathf.Cos(theta) + w2 * Mathf.Sin(theta);

            return adjEnv - tool.GetSphereRadiusAt(0) * normal - tool.GetSphereCenterHeightAt(0) * axis;
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
            return adjacentEnvelopeA0.GetEnvelopeDtAt(t, 1) - tool.GetSphereRadiusAt(0) * adjacentEnvelopeA0.CalculateNormalDtAt(t, 1) - tool.GetSphereCenterHeightAt(0) * GetToolAxisDtAt(t);
        }
        else if (IsPositionContinuous)
        {
            Vector3 axis = GetToolAxisAt(t); // unit vector
            Vector3 axis_t = GetToolAxisDtAt(t); // derivative of unit vector
            Vector3 adjEnv_t = adjacentEnvelopeA0.GetEnvelopeDtAt(t, 1); // not yet unit vector
            Vector3 adjEnv_tt = adjacentEnvelopeA0.GetEnvelopeDt2At(t, 1); // not yet derivative of unit vector
            float dotValue = -tool.GetSphereRadiusDaAt(0) / tool.GetSphereCenterHeightDaAt(0);

            // MATH
            Vector3 v = adjEnv_t.normalized;
            Vector3 v_t = MathUtility.NormalVectorDerivative(adjEnv_t, adjEnv_tt);
            Vector3 p = Vector3.up;
            if (p == v) p = Vector3.right;
            Vector3 w1 = p - Vector3.Dot(p, v) * v;
            Vector3 w1_t = -(Vector3.Dot(p, v) * v_t + Vector3.Dot(p, v_t) * v);
            w1_t = MathUtility.NormalVectorDerivative(w1, w1_t);
            w1.Normalize();
            Vector3 w2 = Vector3.Cross(v, w1);
            Vector3 w2_t = Vector3.Cross(v_t, w1) + Vector3.Cross(v, w1_t);
            float a_dot_w1 = Vector3.Dot(axis, w1);
            float a_dot_w1_t = Vector3.Dot(axis_t, w1) + Vector3.Dot(axis, w1_t);
            float a_dot_w2 = Vector3.Dot(axis, w2);
            float a_dot_w2_t = Vector3.Dot(axis_t, w2) + Vector3.Dot(axis, w2_t);

            float phi = Mathf.Atan2(a_dot_w2, a_dot_w1);
            float theta = phi - Mathf.Acos(dotValue / Mathf.Sqrt(a_dot_w1 * a_dot_w1 + a_dot_w2 * a_dot_w2));
            float c_theta = Mathf.Cos(theta);
            float s_theta = Mathf.Sin(theta);
            float theta_t = -(c_theta * a_dot_w1_t + s_theta * a_dot_w2_t) /
                            (-s_theta * a_dot_w1 + c_theta * a_dot_w2);

            Vector3 normal_t = theta_t * (-s_theta * w1 + c_theta * w2) + c_theta * w1_t + s_theta * w2_t;

            return adjEnv_t - tool.GetSphereRadiusAt(0) * normal_t - tool.GetSphereCenterHeightAt(0) * axis_t;
        }
        else
        {
            return toolPath.EvaluateDerivative(t);
        }
    }

    public Vector3 GetToolPathDt2At(float t)
    {
        if (IsTangentContinuous)
        {
            return adjacentEnvelopeA0.GetEnvelopeDt2At(t, 1) - tool.GetSphereRadiusAt(0) * adjacentEnvelopeA0.CalculateNormalDt2At(t, 1) - tool.GetSphereCenterHeightAt(0) * GetToolAxisDt2At(t);
        }
        else if (IsPositionContinuous)
        {
            Vector3 axis = GetToolAxisAt(t); // unit vector
            Vector3 axis_t = GetToolAxisDtAt(t); // derivative of unit vector
            Vector3 axis_tt = GetToolAxisDt2At(t); // 2nd derivative of unit vector
            Vector3 adjEnv_t = adjacentEnvelopeA0.GetEnvelopeDtAt(t, 1); // not yet unit vector
            Vector3 adjEnv_tt = adjacentEnvelopeA0.GetEnvelopeDt2At(t, 1); // not yet derivative of unit vector
            Vector3 adjEnv_ttt = adjacentEnvelopeA0.GetEnvelopeDt3At(t, 1); // not yet derivative of unit vector
            float dotValue = -tool.GetSphereRadiusDaAt(0) / tool.GetSphereCenterHeightDaAt(0);

            // MATH
            Vector3 v = adjEnv_t.normalized;
            Vector3 v_t = MathUtility.NormalVectorDerivative(adjEnv_t, adjEnv_tt);
            Vector3 v_tt = MathUtility.NormalVectorDerivative2(adjEnv_t, adjEnv_tt, adjEnv_ttt);
            Vector3 p = Vector3.up;
            if (p == v) p = Vector3.right;
            Vector3 w1 = p - Vector3.Dot(p, v) * v;
            Vector3 w1_t = -(Vector3.Dot(p, v_t) * v + Vector3.Dot(p, v) * v_t);
            Vector3 w1_tt = -(Vector3.Dot(p, v_tt) * v + 2 * Vector3.Dot(p, v_t) * v_t + Vector3.Dot(p, v) * v_tt);
            w1_tt = MathUtility.NormalVectorDerivative2(w1, w1_t, w1_tt);
            w1_t = MathUtility.NormalVectorDerivative(w1, w1_t);
            w1.Normalize();
            Vector3 w2 = Vector3.Cross(v, w1);
            Vector3 w2_t = Vector3.Cross(v_t, w1) + Vector3.Cross(v, w1_t);
            Vector3 w2_tt = Vector3.Cross(v_tt, w1) + 2 * Vector3.Cross(v_t, w1_t) + Vector3.Cross(v, w1_tt);
            float a_dot_w1 = Vector3.Dot(axis, w1);
            float a_dot_w1_t = Vector3.Dot(axis_t, w1) + Vector3.Dot(axis, w1_t);
            float a_dot_w1_tt = Vector3.Dot(axis_tt, w1) + 2 * Vector3.Dot(axis_t, w1_t) + Vector3.Dot(axis, w1_tt);
            float a_dot_w2 = Vector3.Dot(axis, w2);
            float a_dot_w2_t = Vector3.Dot(axis_t, w2) + Vector3.Dot(axis, w2_t);
            float a_dot_w2_tt = Vector3.Dot(axis_tt, w2) + 2 * Vector3.Dot(axis_t, w2_t) + Vector3.Dot(axis, w2_tt);

            float phi = Mathf.Atan2(a_dot_w2, a_dot_w1);
            float theta = phi - Mathf.Acos(dotValue / Mathf.Sqrt(a_dot_w1 * a_dot_w1 + a_dot_w2 * a_dot_w2));
            float c_theta = Mathf.Cos(theta);
            float s_theta = Mathf.Sin(theta);

            float k = c_theta * a_dot_w1_t + s_theta * a_dot_w2_t;
            float l = -s_theta * a_dot_w1 + c_theta * a_dot_w2;
            float theta_t = -k / l;

            float dk = theta_t * -s_theta * a_dot_w1_t + c_theta * a_dot_w1_tt +
                       theta_t * c_theta * a_dot_w2_t + s_theta * a_dot_w2_tt;
            float dl = -(theta_t * c_theta * a_dot_w1 + s_theta * a_dot_w1_t) +
                       theta_t * -s_theta * a_dot_w2 + c_theta + a_dot_w2_t;
            float theta_tt = -(l * dk - k * dl) / (l * l);


            Vector3 i = -s_theta * w1 + c_theta * w2;
            Vector3 di = -(theta_t * c_theta * w1 + s_theta * w1_t) +
                         theta_t * -s_theta * w2 + c_theta * w2_t;
            Vector3 normal_t = theta_t * i + c_theta * w1_t + s_theta * w2_t;
            Vector3 normal_tt = theta_tt * i + theta_t * di +
                                theta_t * -s_theta * w1_t + c_theta * w1_tt +
                                theta_t * c_theta * w2_t + s_theta * w2_tt;

            return adjEnv_tt - tool.GetSphereRadiusAt(0) * normal_tt - tool.GetSphereCenterHeightAt(0) * axis_tt;
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

    public Vector3 GetToolPathDt4At(float t)
    {
        // Need to check if these are required for chaining position continuous envelopes
        return toolPath.EvaluateDerivative4Plus(t);
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
        float degrees = Mathf.Rad2Deg * Mathf.Acos(-tool.GetSphereRadiusDaAt(0) / tool.GetSphereCenterHeightDaAt(0));
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
            Vector3 x0 = adjacentEnvelopeA0.GetEnvelopeAt(t, 1);
            Vector3 x1 = adjacentEnvelopeA1.GetEnvelopeAt(t, 0);
            Vector3 deltaX = x1 - x0;
            Vector3 deltaX_hat = deltaX.normalized;

            Vector3 x0_t = adjacentEnvelopeA0.GetEnvelopeDtAt(t, 1);
            Vector3 x1_t = adjacentEnvelopeA1.GetEnvelopeDtAt(t, 0);
            Vector3 x0_t_hat = x0_t.normalized;
            Vector3 x1_t_hat = x1_t.normalized;

            // The following method only works when Delta X can be made with a cylinder of hight and radius of 1, where x1 lies on the bottom ring of the cylinder and x2 on the top ring.

            // Separately find the parts of the axis parallel and perpendicular to Delta X
            // By utilizing Delta X dot A = 1 we can calculate the angle between Delta X and A
            float c_theta = Mathf.Clamp(1.0f / deltaX.magnitude, -1, 1);
            float s_theta = Mathf.Sqrt(1 - c_theta * c_theta);
            Vector3 axis_par_deltaX = c_theta * deltaX_hat;

            // For the perpendicular part we make an orthonormal basis on the plane perpendicular to Delta X, with radius Sin(theta)
            Vector3 v = Vector3.up;
            if (v == deltaX_hat) v = Vector3.right;
            Vector3 v1 = v - Vector3.Dot(v, deltaX_hat) * deltaX_hat;
            v1 = s_theta * v1.normalized;
            Vector3 v2 = Vector3.Cross(deltaX_hat, v1);

            // The normals at x1 and x2 are perpendicular to the respective time derivates, as well as the axis.
            // This means each normal is the cross product of the time derivative and the axis (which is split in the parallel and perpendicular part).
            // This eventually leads to the form A*cos(phi) + B*sin(phi)=C, where A, B, and C are all coplanar vectors (by construction), which is the only reason this works.
            Vector3 D = v1 - Vector3.Cross(x1_t_hat, v1) + Vector3.Cross(x0_t_hat, v1);
            Vector3 E = v2 - Vector3.Cross(x1_t_hat, v2) + Vector3.Cross(x0_t_hat, v2);
            Vector3 F = deltaX + Vector3.Cross(x1_t_hat, axis_par_deltaX) - Vector3.Cross(x0_t_hat, axis_par_deltaX) - axis_par_deltaX;

            // By using dot product we can find phi
            float h = Vector3.Dot(D, D);
            float i = Vector3.Dot(D, E);
            float j = Vector3.Dot(E, E);
            float k = Vector3.Dot(D, F);
            float l = Vector3.Dot(E, F);
            float phi = Mathf.Atan2(l * h - k * i, k * j - l * i);
            float c_phi = Mathf.Cos(phi);
            float s_phi = Mathf.Sin(phi);

            Vector3 axis_perp_deltaX = v1 * c_phi + v2 * s_phi;
            axis = axis_par_deltaX + axis_perp_deltaX;
        }
        else if (IsTangentContinuous)
        {
            // Rotate the normal of the adjacent envelope around the cross product with the axis of the adjacent envelope
            axis = CalculateToolAxisRotationAt(t) * adjacentEnvelopeA0.GetToolAxisAt(t);
        }
        else
        {
            axis = Vector3.Lerp(toolAxisT0.normalized, toolAxisT1.normalized, t);
            axis.Normalize();
        }
        return axis;
    }

    public Vector3 GetToolAxisDtAt(float t)
    {
        Vector3 axis, axis_t;
        if (IsAxisConstrained)
        {
            Vector3 x0 = adjacentEnvelopeA0.GetEnvelopeAt(t, 1);
            Vector3 x1 = adjacentEnvelopeA1.GetEnvelopeAt(t, 0);
            Vector3 deltaX = x1 - x0;
            Vector3 deltaX_hat = deltaX.normalized;

            Vector3 x0_t = adjacentEnvelopeA0.GetEnvelopeDtAt(t, 1);
            Vector3 x1_t = adjacentEnvelopeA1.GetEnvelopeDtAt(t, 0);
            Vector3 deltaX_t = x1_t - x0_t;
            Vector3 deltaX_hat_t = MathUtility.NormalVectorDerivative(deltaX, deltaX_t);
            Vector3 x0_t_hat = x0_t.normalized;
            Vector3 x1_t_hat = x1_t.normalized;

            Vector3 x0_tt = adjacentEnvelopeA0.GetEnvelopeDt2At(t, 1);
            Vector3 x1_tt = adjacentEnvelopeA1.GetEnvelopeDt2At(t, 0);
            Vector3 x0_t_hat_t = MathUtility.NormalVectorDerivative(x0_t, x0_tt);
            Vector3 x1_t_hat_t = MathUtility.NormalVectorDerivative(x1_t, x1_tt);

            // The following method only works when Delta X can be made with a cylinder of hight and radius of 1, where x1 lies on the bottom ring of the cylinder and x2 on the top ring.

            // Separately find the parts of the axis parallel and perpendicular to Delta X
            // By utilizing Delta X dot A = 1 we can calculate the angle between Delta X and A
            float c_theta = Mathf.Clamp(1.0f / deltaX.magnitude, -1, 1);
            float c_theta_t = -Vector3.Dot(deltaX, deltaX_t) / Mathf.Pow(deltaX.magnitude, 3);
            float s_theta = Mathf.Sqrt(1 - c_theta * c_theta);
            float s_theta_t = -c_theta * c_theta_t / s_theta;
            if (s_theta == 0)
            {
                s_theta_t = 0;
            }
            Vector3 axis_par_deltaX = c_theta * deltaX_hat;
            Vector3 axis_par_deltaX_t = c_theta_t * deltaX_hat + c_theta * deltaX_hat_t;

            // For the perpendicular part we make an orthonormal basis on the plane perpendicular to Delta X, with radius Sin(theta)
            Vector3 v = Vector3.up;
            if (v == deltaX_hat) v = Vector3.right;
            Vector3 v1 = v - Vector3.Dot(v, deltaX_hat) * deltaX_hat;
            Vector3 v1_t = -(Vector3.Dot(v, deltaX_hat_t) * deltaX_hat + Vector3.Dot(v, deltaX_hat) * deltaX_hat_t);
            v1_t = s_theta_t * v1.normalized + s_theta * MathUtility.NormalVectorDerivative(v1, v1_t);
            v1 = s_theta * v1.normalized;
            Vector3 v2 = Vector3.Cross(deltaX_hat, v1);
            Vector3 v2_t = Vector3.Cross(deltaX_hat_t, v1) + Vector3.Cross(deltaX_hat, v1_t);

            // The normals at x1 and x2 are perpendicular to the respective time derivates, as well as the axis.
            // This means each normal is the cross product of the time derivative and the axis (which is split in the parallel and perpendicular part).
            // This eventually leads to the form A*cos(phi) + B*sin(phi)=C, where A, B, and C are all coplanar vectors (by construction), which is the only reason this works.
            Vector3 D = v1 - Vector3.Cross(x1_t_hat, v1) + Vector3.Cross(x0_t_hat, v1);
            Vector3 D_t = v1_t -
                          (Vector3.Cross(x1_t_hat_t, v1) + Vector3.Cross(x1_t_hat, v1_t)) +
                          (Vector3.Cross(x0_t_hat_t, v1) + Vector3.Cross(x0_t_hat, v1_t));
            Vector3 E = v2 - Vector3.Cross(x1_t_hat, v2) + Vector3.Cross(x0_t_hat, v2);
            Vector3 E_t = v2_t -
                          (Vector3.Cross(x1_t_hat_t, v2) + Vector3.Cross(x1_t_hat, v2_t)) +
                          (Vector3.Cross(x0_t_hat_t, v2) + Vector3.Cross(x0_t_hat, v2_t));
            Vector3 F = deltaX + Vector3.Cross(x1_t_hat, axis_par_deltaX) - Vector3.Cross(x0_t_hat, axis_par_deltaX) - axis_par_deltaX;
            Vector3 F_t = deltaX_t +
                          (Vector3.Cross(x1_t_hat_t, axis_par_deltaX) + Vector3.Cross(x1_t_hat, axis_par_deltaX_t)) -
                          (Vector3.Cross(x0_t_hat_t, axis_par_deltaX) + Vector3.Cross(x0_t_hat, axis_par_deltaX_t)) -
                          axis_par_deltaX_t;

            // By using dot product we can find phi
            float h = Vector3.Dot(D, D);
            float i = Vector3.Dot(D, E);
            float j = Vector3.Dot(E, E);
            float k = Vector3.Dot(D, F);
            float l = Vector3.Dot(E, F);
            float phi = Mathf.Atan2(l * h - k * i, k * j - l * i);
            float c_phi = Mathf.Cos(phi);
            float s_phi = Mathf.Sin(phi);

            Vector3 helper = -D * s_phi + E * c_phi;
            float phi_t = Vector3.Dot(helper, F_t - D_t * c_phi - E_t * s_phi) / helper.sqrMagnitude;
            if (helper.sqrMagnitude == 0)
            {
                phi_t = 0;
            }

            Vector3 axis_perp_deltaX = v1 * c_phi + v2 * s_phi;
            Vector3 axis_perp_deltaX_t = v1_t * c_phi + v1 * phi_t * -s_phi +
                                         v2_t * s_phi + v2 * phi_t * c_phi;
            axis = axis_par_deltaX + axis_perp_deltaX;

            axis_t = axis_par_deltaX_t + axis_perp_deltaX_t;
        }
        else if (IsTangentContinuous)
        {
            axis_t = CalculateToolAxisRotationAt(t) * adjacentEnvelopeA0.GetToolAxisDtAt(t);
        }
        else
        {
            axis = Vector3.Lerp(toolAxisT0.normalized, toolAxisT1.normalized, t);
            axis_t = toolAxisT1.normalized - toolAxisT0.normalized;
            axis_t = MathUtility.NormalVectorDerivative(axis, axis_t);
        }
        return axis_t;
    }

    public Vector3 GetToolAxisDt2At(float t)
    {
        // TODO: for now the axis constrained case only works once. If the tool isn't big enough, chaining constrained envelopes together won't work as it needs higher the higher derivatives of the envelope and normal.
        Vector3 axis, axis_t, axis_tt;
        if (IsTangentContinuous)
        {
            axis_tt = CalculateToolAxisRotationAt(t) * adjacentEnvelopeA0.GetToolAxisDt2At(t);
        }
        else
        {
            axis = Vector3.Lerp(toolAxisT0.normalized, toolAxisT1.normalized, t);
            axis_t = toolAxisT1.normalized - toolAxisT0.normalized;
            axis_tt = Vector3.zero;
            axis_tt = MathUtility.NormalVectorDerivative2(axis, axis_t, axis_tt);
        }
        return axis_tt;
    }

    public Vector3 GetToolAxisDt3At(float t)
    {
        // TODO: for now the axis constrained case only works once. If the tool isn't big enough, chaining constrained envelopes together won't work as it needs higher the higher derivatives of the envelope and normal.
        Vector3 axis, axis_t, axis_tt, axis_ttt;
        if (IsTangentContinuous)
        {
            axis_ttt = CalculateToolAxisRotationAt(t) * adjacentEnvelopeA0.GetToolAxisDt3At(t);
        }
        else
        {
            axis = Vector3.Lerp(toolAxisT0.normalized, toolAxisT1.normalized, t);
            axis_t = toolAxisT1.normalized - toolAxisT0.normalized;
            axis_tt = Vector3.zero;
            axis_ttt = Vector3.zero;
            axis_ttt = MathUtility.NormalVectorDerivative3(axis, axis_t, axis_tt, axis_ttt);
        }
        return axis_ttt;
    }

    public Vector3 GetToolAxisDt4At(float t)
    {
        // TODO: for now the axis constrained case only works once. If the tool isn't big enough, chaining constrained envelopes together won't work as it needs higher the higher derivatives of the envelope and normal.
        Vector3 axis, axis_t, axis_tt, axis_ttt, axis_tttt;
        if (IsTangentContinuous)
        {
            axis_tttt = CalculateToolAxisRotationAt(t) * adjacentEnvelopeA0.GetToolAxisDt4At(t);
        }
        else
        {
            axis = Vector3.Lerp(toolAxisT0.normalized, toolAxisT1.normalized, t);
            axis_t = toolAxisT1.normalized - toolAxisT0.normalized;
            axis_tt = Vector3.zero;
            axis_ttt = Vector3.zero;
            axis_tttt = Vector3.zero;
            axis_tttt = MathUtility.NormalVectorDerivative4(axis, axis_t, axis_tt, axis_ttt, axis_tttt);
        }
        return axis_tttt;
    }

    // Calculate normal of envelope according to Bassegoda's paper
    // Expects t in [0, 1] and a in [0, 1]
    public Vector3 CalculateNormalAt(float t, float a)
    {
        Vector3 sa = tool.GetSphereCenterHeightDaAt(a) * GetToolAxisAt(t);
        Vector3 st = GetToolPathDtAt(t) + tool.GetSphereCenterHeightAt(a) * GetToolAxisDtAt(t);
        Vector3 sNormal = Vector3.Cross(sa, st).normalized;

        float ra = tool.GetSphereRadiusDaAt(a);

        float E = Vector3.Dot(sa, sa);
        float F = Vector3.Dot(sa, st);
        float G = Vector3.Dot(st, st);
        float EG_FF = E * G - F * F;

        float m11 = G / EG_FF;
        float m21 = -F / EG_FF;

        float alpha = -m11 * ra;
        float beta = -m21 * ra;

        float sqrt_term = 1 - ra * ra * m11;
        float gamma = (EG_FF > 0 ? 1 : -1) * Mathf.Sqrt(sqrt_term);

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

        float ra = tool.GetSphereRadiusDaAt(a);

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

        float sqrt_term = 1 - ra * ra * m11;
        float gamma = (EG_FF > 0 ? 1 : -1) * Mathf.Sqrt(sqrt_term);
        float gamma_t = (EG_FF > 0 ? 1 : -1) * -ra * ra * m11_t / (2 * Mathf.Sqrt(sqrt_term));

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

        float ra = tool.GetSphereRadiusDaAt(a);

        float E = Vector3.Dot(sa, sa);
        float Et = 2 * Vector3.Dot(sa, sat);
        float Ett = 2 * Vector3.Dot(sat, sat) + 2 * Vector3.Dot(sa, satt);
        float F = Vector3.Dot(sa, st);
        float Ft = Vector3.Dot(sat, st) + Vector3.Dot(sa, stt);
        float Ftt = Vector3.Dot(satt, st) + 2 * Vector3.Dot(sat, stt) + Vector3.Dot(sa, sttt);
        float G = Vector3.Dot(st, st);
        float Gt = 2 * Vector3.Dot(st, stt);
        float Gtt = 2 * Vector3.Dot(stt, stt) + 2 * Vector3.Dot(st, sttt);
        float EG_FF = E * G - F * F;
        float EG_FF_2 = EG_FF * EG_FF;
        float EG_FF_3 = EG_FF * EG_FF * EG_FF;
        float EG_FF_t = Et * G + E * Gt - 2 * F * Ft;
        float EG_FF_tt = Ett * G + 2 * Et * Gt + E * Gtt - 2 * (Ft * Ft + F * Ftt);

        float m11 = G / EG_FF;
        float m11_t = (EG_FF * Gt - G * EG_FF_t) / EG_FF_2;
        float m11_tt = (Gtt * EG_FF_2 -
                       2 * Gt * EG_FF * EG_FF_t -
                       G * EG_FF * EG_FF_tt +
                       2 * G * EG_FF_t * EG_FF_t) / EG_FF_3;

        float m21 = -F / EG_FF;
        float m21_t = -(EG_FF * Ft - F * EG_FF_t) / EG_FF_2;
        float m21_tt = -(Ftt * EG_FF_2 -
                        2 * Ft * EG_FF * EG_FF_t -
                        F * EG_FF * EG_FF_tt +
                        2 * F * EG_FF_t * EG_FF_t) / EG_FF_3;

        float alpha = -m11 * ra;
        float alpha_t = -m11_t * ra;
        float alpha_tt = -m11_tt * ra;

        float beta = -m21 * ra;
        float beta_t = -m21_t * ra;
        float beta_tt = -m21_tt * ra;

        float sqrt_term = 1 - ra * ra * m11;
        float gamma = (EG_FF > 0 ? 1 : -1) * Mathf.Sqrt(sqrt_term);
        float gamma_t = (EG_FF > 0 ? 1 : -1) * -ra * ra * m11_t / (2 * Mathf.Sqrt(sqrt_term));
        float gamma_tt = (EG_FF > 0 ? 1 : -1) *
                         -ra * ra / 2 *
                         (m11_tt / Mathf.Sqrt(sqrt_term) +
                         (ra * ra * m11_t * m11_t) / (2 * Mathf.Pow(sqrt_term, 1.5f)));

        Vector3 n = alpha * sa +
                    beta * st +
                    gamma * sNormal;
        Vector3 nt = alpha * sat + alpha_t * sa +
                     beta * stt + beta_t * st +
                     gamma * sNormal_t + gamma_t * sNormal;
        Vector3 ntt = alpha * satt +
                      2 * alpha_t * sat +
                      alpha_tt * sa +
                      beta * sttt +
                      2 * beta_t * stt +
                      beta_tt * st +
                      gamma * sNormal_tt +
                      2 * +gamma_t * sNormal_t +
                      gamma_tt * sNormal;
        return MathUtility.NormalVectorDerivative2(n, nt, ntt);
    }

    public Vector3 CalculateNormalDt3At(float t, float a)
    {
        Vector3 sa = tool.GetSphereCenterHeightDaAt(a) * GetToolAxisAt(t);
        Vector3 sat = tool.GetSphereCenterHeightDaAt(a) * GetToolAxisDtAt(t);
        Vector3 satt = tool.GetSphereCenterHeightDaAt(a) * GetToolAxisDt2At(t);
        Vector3 sattt = tool.GetSphereCenterHeightDaAt(a) * GetToolAxisDt3At(t);
        Vector3 st = GetToolPathDtAt(t) + tool.GetSphereCenterHeightAt(a) * GetToolAxisDtAt(t);
        Vector3 stt = GetToolPathDt2At(t) + tool.GetSphereCenterHeightAt(a) * GetToolAxisDt2At(t);
        Vector3 sttt = GetToolPathDt3At(t) + tool.GetSphereCenterHeightAt(a) * GetToolAxisDt3At(t);
        Vector3 stttt = GetToolPathDt4At(t) + tool.GetSphereCenterHeightAt(a) * GetToolAxisDt4At(t);
        Vector3 sNormal = Vector3.Cross(sa, st);
        Vector3 sNormal_t = Vector3.Cross(sat, st) + Vector3.Cross(sa, stt);
        Vector3 sNormal_tt = Vector3.Cross(satt, st) + 2 * Vector3.Cross(sat, stt) + Vector3.Cross(sa, sttt);
        Vector3 sNormal_ttt = Vector3.Cross(sattt, st) + 3 * Vector3.Cross(satt, stt) + 3 * Vector3.Cross(sat, sttt) + Vector3.Cross(sa, stttt);
        sNormal_ttt = MathUtility.NormalVectorDerivative3(sNormal, sNormal_t, sNormal_tt, sNormal_ttt);
        sNormal_tt = MathUtility.NormalVectorDerivative2(sNormal, sNormal_t, sNormal_tt);
        sNormal_t = MathUtility.NormalVectorDerivative(sNormal, sNormal_t);
        sNormal.Normalize();

        float ra = tool.GetSphereRadiusDaAt(a);

        float E = Vector3.Dot(sa, sa);
        float Et = 2 * Vector3.Dot(sa, sat);
        float Ett = 2 * Vector3.Dot(sat, sat) + 2 * Vector3.Dot(sa, satt);
        float Ettt = 6 * Vector3.Dot(sat, satt) + 2 * Vector3.Dot(sa, sattt);
        float F = Vector3.Dot(sa, st);
        float Ft = Vector3.Dot(sat, st) + Vector3.Dot(sa, stt);
        float Ftt = Vector3.Dot(satt, st) + 2 * Vector3.Dot(sat, stt) + Vector3.Dot(sa, sttt);
        float Fttt = Vector3.Dot(sattt, st) + 3 * Vector3.Dot(satt, stt) + 3 * Vector3.Dot(sat, sttt) + Vector3.Dot(sa, stttt);
        float G = Vector3.Dot(st, st);
        float Gt = 2 * Vector3.Dot(st, stt);
        float Gtt = 2 * Vector3.Dot(stt, stt) + 2 * Vector3.Dot(st, sttt);
        float Gttt = 6 * Vector3.Dot(stt, sttt) + 2 * Vector3.Dot(st, stttt);
        float EG_FF = E * G - F * F;
        float EG_FF_2 = EG_FF * EG_FF;
        float EG_FF_3 = EG_FF * EG_FF * EG_FF;
        float EG_FF_4 = EG_FF_2 * EG_FF_2;
        float EG_FF_t = Et * G + E * Gt - 2 * F * Ft;
        float EG_FF_tt = Ett * G + 2 * Et * Gt + E * Gtt - 2 * (Ft * Ft + F * Ftt);
        float EG_FF_ttt = Ettt * G + 3 * Ett * Gt + 3 * Et * Gtt + E * Gttt - 2 * (3 * Ft * Ftt + F * Fttt);

        float m11 = G / EG_FF;
        float m11_t = (EG_FF * Gt - G * EG_FF_t) / EG_FF_2;
        float m11_tt = (Gtt * EG_FF_2 -
                       2 * Gt * EG_FF * EG_FF_t -
                       G * EG_FF * EG_FF_tt +
                       2 * G * EG_FF_t * EG_FF_t) / EG_FF_3;
        float m11_ttt = (Gttt * EG_FF_3 -
                        3 * EG_FF_2 * Gtt * EG_FF_t -
                        3 * EG_FF_2 * Gt * EG_FF_tt +
                        6 * EG_FF * Gt * EG_FF_t * EG_FF_t -
                        G * EG_FF_ttt * EG_FF_2 -
                        6 * G * EG_FF_t * EG_FF_t * EG_FF_t +
                        6 * G * EG_FF * EG_FF_t * EG_FF_tt) / EG_FF_4;

        float m21 = -F / EG_FF;
        float m21_t = -(EG_FF * Ft - F * EG_FF_t) / EG_FF_2;
        float m21_tt = -(Ftt * EG_FF_2 -
                        2 * Ft * EG_FF * EG_FF_t -
                        F * EG_FF * EG_FF_tt +
                        2 * F * EG_FF_t * EG_FF_t) / EG_FF_3;
        float m21_ttt = -(Fttt * EG_FF_3 -
                        3 * EG_FF_2 * Ftt * EG_FF_t -
                        3 * EG_FF_2 * Ft * EG_FF_tt +
                        6 * EG_FF * Ft * EG_FF_t * EG_FF_t -
                        F * EG_FF_ttt * EG_FF_2 -
                        6 * F * EG_FF_t * EG_FF_t * EG_FF_t +
                        6 * F * EG_FF * EG_FF_t * EG_FF_tt) / EG_FF_4;

        float alpha = -m11 * ra;
        float alpha_t = -m11_t * ra;
        float alpha_tt = -m11_tt * ra;
        float alpha_ttt = -m11_ttt * ra;

        float beta = -m21 * ra;
        float beta_t = -m21_t * ra;
        float beta_tt = -m21_tt * ra;
        float beta_ttt = -m21_ttt * ra;

        float sqrt_term = 1 - ra * ra * m11;
        float gamma = (EG_FF > 0 ? 1 : -1) * Mathf.Sqrt(sqrt_term);
        float gamma_t = (EG_FF > 0 ? 1 : -1) * -ra * ra * m11_t / (2 * Mathf.Sqrt(sqrt_term));
        float gamma_tt = (EG_FF > 0 ? 1 : -1) *
                         -ra * ra / 2 *
                         (m11_tt / Mathf.Sqrt(sqrt_term) +
                          (ra * ra * m11_t * m11_t) / (2 * Mathf.Pow(sqrt_term, 1.5f)));
        float gamma_ttt = (EG_FF > 0 ? 1 : -1) *
                          -ra * ra / 2 *
                          ((m11_ttt * Mathf.Sqrt(sqrt_term) + (ra * ra * m11_t * m11_tt) / (2 * Mathf.Sqrt(sqrt_term))) / (sqrt_term) +
                          (ra * ra * m11_t * m11_tt * sqrt_term + 1.5f * Mathf.Pow(ra, 4) * Mathf.Pow(m11_t, 3)) / (Mathf.Pow(sqrt_term, 2.5f)));

        Vector3 n = alpha * sa +
                    beta * st +
                    gamma * sNormal;
        Vector3 nt = alpha * sat + alpha_t * sa +
                     beta * stt + beta_t * st +
                     gamma * sNormal_t + gamma_t * sNormal;
        Vector3 ntt = alpha * satt +
                      2 * alpha_t * sat +
                      alpha_tt * sa +
                      beta * sttt +
                      2 * beta_t * stt +
                      beta_tt * st +
                      gamma * sNormal_tt +
                      2 * gamma_t * sNormal_t +
                      gamma_tt * sNormal;
        Vector3 nttt = alpha * sattt +
                       3 * alpha_t * satt +
                       3 * alpha_tt * sat +
                       alpha_ttt * sa +
                       beta * stttt +
                       3 * beta_t * sttt +
                       3 * beta_tt * stt +
                       beta_ttt * st +
                       gamma * sNormal_ttt +
                       3 * gamma_t * sNormal_tt +
                       3 * gamma_tt * sNormal_t +
                       gamma_ttt * sNormal;
        return MathUtility.NormalVectorDerivative3(n, nt, ntt, nttt);
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
            Vector3 origin = Vector3.zero;
            Vector3 p = GetToolPathAt(t);
            Vector3 pt = GetToolPathDtAt(t);
            Vector3 axis = GetToolAxisAt(t);
            Vector3 axis_t = GetToolAxisDtAt(t);
            Vector3 s = p + tool.GetSphereCenterHeightAt(a) * axis;
            Vector3 s0 = p + tool.GetSphereCenterHeightAt(0) * axis;
            Vector3 s1 = p + tool.GetSphereCenterHeightAt(1) * axis;
            Vector3 x = GetEnvelopeAt(t, a);
            Vector3 x_at_0 = GetEnvelopeAt(t, 0);
            Vector3 x_at_1 = GetEnvelopeAt(t, 1);
            Vector3 xt = GetEnvelopeDtAt(t, a);
            Vector3 xt0 = GetEnvelopeDtAt(t, 0);
            Vector3 xt1 = GetEnvelopeDtAt(t, 1);
            Vector3 xt0_hat = xt0.normalized;
            Vector3 xt1_hat = xt1.normalized;
            Vector3 xtt0 = GetEnvelopeDt2At(t, 0);
            Vector3 xtt1 = GetEnvelopeDt2At(t, 1);
            Vector3 xt0_hat_t = MathUtility.NormalVectorDerivative(xt0, xtt0);
            Vector3 xt1_hat_t = MathUtility.NormalVectorDerivative(xt1, xtt1);
            Vector3 n = CalculateNormalAt(t, a);
            Vector3 n0 = CalculateNormalAt(t, 0);
            Vector3 n1 = CalculateNormalAt(t, 1);
            Vector3 nt = CalculateNormalDtAt(t, a);
            Vector3 deltaX = x_at_1 - x_at_0;
            Vector3 deltaX_t = xt1 - xt0;
            Vector3 deltaX_hat = deltaX.normalized;
            Vector3 deltaX_hat_t = MathUtility.NormalVectorDerivative(deltaX, deltaX_t);

            float deltaX_mag_t = Vector3.Dot(deltaX, deltaX_t) / deltaX.magnitude;

            // Axis
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(p, p + axis);
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(p, p + axis_t);
            // Gizmos.color = Color.magenta;
            // Gizmos.DrawLine(p, p + aXat);

            // Normal
            Gizmos.color = Color.green;
            Gizmos.DrawLine(s0, s0 + n0);
            Gizmos.DrawLine(s1, s1 + n1);
            Gizmos.color = Color.magenta;
            // Gizmos.DrawLine(s, s + nt);

            // Envelope
            Gizmos.color = Color.red;
            Gizmos.DrawLine(x_at_0, x_at_0 + xt0);
            Gizmos.DrawLine(x_at_1, x_at_1 + xt1);

            Gizmos.color = Color.black;
            Gizmos.DrawLine(x_at_0, x_at_1);
            Gizmos.DrawLine(p, p + deltaX);

            /*
            ** NOTE Nearly all the math and gizmos below serve as a visualization of the method 
            ** described in section 5.2 of the MSc Thesis Toward Surface Design by Envelopes. 
            ** Play around with enabling and disabling them if you are interested in continuing 
            ** the research of that thesis.
            */

            float c_theta = Mathf.Clamp(1.0f / deltaX.magnitude, -1, 1);
            // float c_theta_t = -Vector3.Dot(deltaX, deltaX_t) / Mathf.Pow(deltaX.magnitude, 3);
            float s_theta = Mathf.Sqrt(1 - c_theta * c_theta);
            // float s_theta_t = -c_theta * c_theta_t / s_theta;
            // if (s_theta == 0)
            // {
            //     s_theta_t = 0;
            // }
            Vector3 axis_par_deltaX = c_theta * deltaX_hat;
            // Vector3 axis_par_deltaX_t = c_theta_t * deltaX_hat + c_theta * deltaX_hat_t;

            Gizmos.color = Color.blue;
            Gizmos.DrawLine(p, p + axis_par_deltaX);
            // Gizmos.color = Color.cyan;
            // Gizmos.DrawLine(p, p + axis_par_deltaX_t);

            Vector3 v = Vector3.up;
            if (v == deltaX_hat) v = Vector3.right;
            Vector3 v1 = v - Vector3.Dot(v, deltaX_hat) * deltaX_hat;
            // Vector3 v1_t = -(Vector3.Dot(v, deltaX_hat_t) * deltaX_hat + Vector3.Dot(v, deltaX_hat) * deltaX_hat_t);
            // v1_t = s_theta_t * v1.normalized + s_theta * MathUtility.NormalVectorDerivative(v1, v1_t);
            v1 = s_theta * v1.normalized;
            Vector3 v2 = Vector3.Cross(deltaX_hat, v1);
            // Vector3 v2_t = Vector3.Cross(deltaX_hat_t, v1) + Vector3.Cross(deltaX_hat, v1_t);

            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(p + axis_par_deltaX, p + axis_par_deltaX + v1);
            Gizmos.DrawLine(p + axis_par_deltaX, p + axis_par_deltaX + v2);
            // Gizmos.DrawLine(p + axis_par_deltaX_t, p + axis_par_deltaX_t + v1_t);
            // Gizmos.DrawLine(p + axis_par_deltaX_t, p + axis_par_deltaX_t + v2_t);

            Vector3 D = v1 - Vector3.Cross(xt1_hat, v1) + Vector3.Cross(xt0_hat, v1);
            // Vector3 D_t = v1_t -
            //               (Vector3.Cross(xt1_hat_t, v1) + Vector3.Cross(xt1_hat, v1_t)) +
            //               (Vector3.Cross(xt0_hat_t, v1) + Vector3.Cross(xt0_hat, v1_t));
            Vector3 E = v2 - Vector3.Cross(xt1_hat, v2) + Vector3.Cross(xt0_hat, v2);
            // Vector3 E_t = v2_t -
            //               (Vector3.Cross(xt1_hat_t, v2) + Vector3.Cross(xt1_hat, v2_t)) +
            //               (Vector3.Cross(xt0_hat_t, v2) + Vector3.Cross(xt0_hat, v2_t));
            Vector3 F = deltaX + Vector3.Cross(xt1_hat, axis_par_deltaX) - Vector3.Cross(xt0_hat, axis_par_deltaX) - axis_par_deltaX;
            // Vector3 F_t = deltaX_t +
            //               (Vector3.Cross(xt1_hat_t, axis_par_deltaX) + Vector3.Cross(xt1_hat, axis_par_deltaX_t)) -
            //               (Vector3.Cross(xt0_hat_t, axis_par_deltaX) + Vector3.Cross(xt0_hat, axis_par_deltaX_t)) -
            //               axis_par_deltaX_t;

            Gizmos.color = Color.red;
            Gizmos.DrawLine(p + axis_par_deltaX, p + axis_par_deltaX + D);
            // Gizmos.DrawLine(p + axis_par_deltaX_t, p + axis_par_deltaX_t + D_t);
            Gizmos.color = Color.magenta;
            Gizmos.DrawLine(p + axis_par_deltaX, p + axis_par_deltaX + E);
            // Gizmos.DrawLine(p + axis_par_deltaX_t, p + axis_par_deltaX_t + E_t);
            Gizmos.color = Color.gray;
            Gizmos.DrawLine(p + axis_par_deltaX, p + axis_par_deltaX + F);
            // Gizmos.DrawLine(p + axis_par_deltaX_t, p + axis_par_deltaX_t + F_t);

            float h = Vector3.Dot(D, D);
            float i = Vector3.Dot(D, E);
            float j = Vector3.Dot(E, E);
            float k = Vector3.Dot(D, F);
            float l = Vector3.Dot(E, F);
            float phi = Mathf.Atan2(l * h - k * i, k * j - l * i);
            float c_phi = Mathf.Cos(phi);
            float s_phi = Mathf.Sin(phi);

            // Vector3 helper = -D * s_phi + E * c_phi;
            // float phi_t = Vector3.Dot(helper, F_t - D_t * c_phi - E_t * s_phi) / helper.sqrMagnitude;
            // if (helper.sqrMagnitude == 0)
            // {
            //     phi_t = 0;
            // }

            Vector3 axis_perp_deltaX = v1 * c_phi + v2 * s_phi;
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(p + axis_par_deltaX, p + axis_par_deltaX + axis_perp_deltaX);
            // Vector3 axis_perp_deltaX_t = v1_t * c_phi + v1 * phi_t * -s_phi +
            //                              v2_t * s_phi + v2 * phi_t * c_phi;
            // Gizmos.color = Color.cyan;
            // Gizmos.DrawLine(p + axis_par_deltaX_t, p + axis_par_deltaX_t + axis_perp_deltaX_t);


            if (IsPositionContinuous)
            {
                Vector3 adjN_1 = adjacentEnvelopeA0.CalculateNormalAt(t, 1);
                Vector3 adjEnv = adjacentEnvelopeA0.GetEnvelopeAt(t, 1);
                Vector3 adjEnv_t = adjacentEnvelopeA0.GetEnvelopeDtAt(t, 1);
                float dotValue = -tool.GetSphereRadiusDaAt(0) / tool.GetSphereCenterHeightDaAt(0);
                Gizmos.color = Color.black;
                Gizmos.DrawLine(p, p + pt);
            }
        }
    }

    private void OnValidate()
    {
        if (adjacentEnvelopeA0 == this) adjacentEnvelopeA0 = null;
        if (adjacentEnvelopeA1 == this || adjacentEnvelopeA1 == adjacentEnvelopeA0) adjacentEnvelopeA1 = null;
    }
}
