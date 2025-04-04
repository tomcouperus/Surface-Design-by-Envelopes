using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(MeshRenderer))]
[RequireComponent(typeof(MeshFilter))]
public class Envelope : MonoBehaviour
{
    public enum EnvelopeType { Bassegoda, Rajain, Bizarri, MOS };

    [SerializeField]
    private Mesh mesh;
    [SerializeField]
    private GameObject sphere;
    [SerializeField]
    private EnvelopeType envelopeType = EnvelopeType.MOS;
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
        Vector3 vertex = Vector3.zero;
        switch (envelopeType)
        {
            case EnvelopeType.Bassegoda:
                vertex = GetEnvelopeAt_Bassegoda(t, a);
                break;
            case EnvelopeType.Rajain:
                vertex = GetEnvelopeAt_Rajain(t, a);
                break;
            case EnvelopeType.Bizarri:
                vertex = GetEnvelopeAt_Bizarri(t, a);
                break;
            case EnvelopeType.MOS:
                vertex = GetEnvelopeAt_MOS(t, a);
                break;
        }
        return vertex;
    }

    public Vector3 GetEnvelopeAt_Bizarri(float t, float a)
    {
        Vector3 s = GetToolPathAt(t) + a * GetToolAxisAt(t);
        Vector3 sa = GetToolAxisAt(t);
        Vector3 st = GetToolPathDerivativeAt(t) + a * GetToolAxisDerivativeAt(t);

        float r = GetToolRadiusAt(a);
        float ra = GetToolRadiusDerivativeAt(a);
        float rt = 0;

        float E = Vector3.Dot(sa, sa);
        float F = Vector3.Dot(sa, st);
        float G = Vector3.Dot(st, st);

        float Ebar = E - ra * ra;
        float Fbar = F - ra * rt;
        float Gbar = G - rt * rt;

        Vector3 n = ((ra * G - rt * F) * sa + (rt * E - ra * F) * st) / (E * G - F * F) +
                    Vector3.Cross(sa, st) * Mathf.Sqrt(Ebar * Gbar - Fbar * Fbar) / (E * G - F * F);
        return s - r * n.normalized;
    }

    public Vector3 GetEnvelopeAt_MOS(float t, float a)
    {
        Vector3 s = GetToolPathAt(t) + a * GetToolAxisAt(t);
        Vector3 sa = GetToolAxisAt(t);
        Vector3 st = GetToolPathDerivativeAt(t) + a * GetToolAxisDerivativeAt(t);

        float r = GetToolRadiusAt(a);
        float ra = GetToolRadiusDerivativeAt(a);
        float rt = 0;

        float p01 = sa.x * st.y - sa.y * st.x;
        float p02 = sa.x * st.z - sa.z * st.x;
        float p03 = sa.x * rt - ra * st.x;
        float p23 = sa.z * rt - ra * st.z;
        float p31 = ra * st.y - sa.y * rt;
        float p12 = sa.y * st.z - sa.z * st.y;

        Vector3 A = new(
            p02 * p23 - p01 * p31,
            p23 * p12 - p01 * p03,
            p31 * p12 - p02 * p03
        );
        Vector3 B = new(p12, -p02, p01);
        float C = (p01 * p01) + (p02 * p02) - (p03 * p03) - (p23 * p23) - (p31 * p31) + (p12 * p12);
        float D = r / ((p01 * p01) + (p02 * p02) + (p12 * p12));

        if (C < 0)
        {
            Debug.LogWarning("C is negative, thus the envelope is not defined");
        }

        return s + D * (A - Mathf.Sqrt(C) * B);
    }

    public Vector3 GetEnvelopeAt_Rajain(float t, float a)
    {
        return GetToolPathAt(t) + a * GetToolAxisAt(t) - GetToolRadiusAt(a) * CalculateNormal_Rajain(t, a);
    }
    public Vector3 GetEnvelopeAt_Bassegoda(float t, float a)
    {
        return GetToolPathAt(t) + a * GetToolAxisAt(t) - GetToolRadiusAt(a) * CalculateNormal_Bassegoda(t, a);
    }

    public Vector3 GetEnvelopeDerivativeTAt(float t, float a)
    {
        return GetToolPathDerivativeAt(t) + a * GetToolAxisDerivativeAt(t) - GetToolRadiusAt(a) * CalculateNormalDerivativeT_Rajain(t, a);
    }

    public Vector3 GetToolPathAt(float t)
    {
        if (IsPositionContinuous)
        {
            return adjacentEnvelopeA0.GetToolPathAt(t) + adjacentEnvelopeA0.GetToolAxisAt(t) /* * adjacentEnvelopeA0.toolHeight */ +
                   adjacentEnvelopeA0.GetToolRadiusAt(1) * adjacentEnvelopeA0.CalculateNormal_Bassegoda(t, 1) -
                   GetToolRadiusAt(0) * CalculateNormal_Bassegoda(t, 0);
        }
        else
        {
            return toolPath.Evaluate(t);
        }
    }

    public Vector3 GetToolPathDerivativeAt(float t)
    {
        if (IsPositionContinuous)
        {
            return adjacentEnvelopeA0.GetToolPathDerivativeAt(t);
        }
        else
        {
            return toolPath.EvaluateTangent(t);
        }
    }

    public Vector3 GetToolPathSecondDerivativeAt(float t)
    {
        if (IsPositionContinuous)
        {
            return adjacentEnvelopeA0.GetToolPathSecondDerivativeAt(t);
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

    public float GetToolRadiusDerivativeAt(float a)
    {
        // Todo replace with actual derivate. Is fixed for now
        return 0;
    }

    public Vector3 GetToolAxisAt(float t)
    {
        Vector3 axis = Vector3.Lerp(toolAxisT0, toolAxisT1, t).normalized;
        if (IsAxisConstrained)
        {
            if (perfectFit)
            {
                axis = perfectFitToolAxes[(int)(t * tSectors)];
            }
            else
            {
                axis = adjacentEnvelopeA1.GetEnvelopeAt_Bassegoda(t, 0) - adjacentEnvelopeA0.GetEnvelopeAt_Bassegoda(t, 1);
            }
        }
        return axis.normalized;
    }

    public Vector3 GetToolAxisDerivativeAt(float t)
    {
        // Todo replace with derivative of tool axis, but since that's fixed it's just zero for now
        Vector3 axisDeriv = GetToolAxisAt(1) - GetToolAxisAt(0);
        if (IsAxisConstrained)
        {
            axisDeriv = adjacentEnvelopeA1.GetEnvelopeDerivativeTAt(t, 0) - adjacentEnvelopeA1.GetEnvelopeDerivativeTAt(t, 1);
        }
        return axisDeriv;
        // return Vector3.zero;
    }

    public Vector3 GetToolAxisSecondDerivativeAt(float t)
    {
        return Vector3.zero;
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
            Vector3 x1 = adjacentEnvelopeA0.GetEnvelopeAt_Bassegoda(t, 1);
            Vector3 x3 = adjacentEnvelopeA1.GetEnvelopeAt_Bassegoda(s, 0);
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
                x3 = adjacentEnvelopeA1.GetEnvelopeAt_Bassegoda(s + deltaS, 0);
                sqrDiff = (x3 - x1).sqrMagnitude - d * d;
                s += deltaS;
            }
        }
    }

    public Vector3 CalculateNormal_Rajain(float t, float a)
    {
        float deltaR = GetToolRadiusAt(1) - GetToolRadiusAt(0);
        Vector3 sa = GetToolAxisAt(t);
        Vector3 st = GetToolPathDerivativeAt(t) + a * GetToolAxisDerivativeAt(t);
        // First fundamental form
        float E = Vector3.Dot(sa, sa);
        float F = Vector3.Dot(sa, st);
        float G = Vector3.Dot(st, st);
        // Debug.Log("E: " + E.ToString("F2") + " -- F: " + F.ToString("F2") + " -- G: " + G.ToString("F2"));

        Vector3 numerator = deltaR * (G * sa - F * st) +
                            Vector3.Cross(sa, st) * Mathf.Sqrt((E - deltaR * deltaR) * G - F * F);
        float denominator = E * G - F * F;

        // Calculate the normal
        Vector3 normal = numerator / denominator;
        return normal.normalized;
    }

    public Vector3 CalculateNormalDerivativeT_Rajain(float t, float a)
    {
        // Tool axis ruled surface derivatives
        Vector3 sa = GetToolAxisAt(t);
        Vector3 st = GetToolPathDerivativeAt(t) + a * GetToolAxisDerivativeAt(t);
        Vector3 sat = GetToolAxisDerivativeAt(t);
        Vector3 stt = GetToolPathSecondDerivativeAt(t) + a * GetToolAxisSecondDerivativeAt(t);
        // First fundamental form of tool axis ruled surface
        float E = Vector3.Dot(sa, sa);
        float Et = 2 * Vector3.Dot(sa, sat);
        float F = Vector3.Dot(sa, st);
        float Ft = Vector3.Dot(sat, st) + Vector3.Dot(sa, stt);
        float G = Vector3.Dot(st, st);
        float Gt = 2 * Vector3.Dot(st, stt);

        float EG_FF = E * G - F * F;
        float EG_FFDeriv = Et * G + E * Gt - 2 * F * Ft;
        float sqrtEG_FF = Mathf.Sqrt(EG_FF);
        float sqrtEG_FFDeriv = EG_FFDeriv / (2 * sqrtEG_FF);
        Vector3 saXst = Vector3.Cross(sa, st);
        Vector3 saXstDeriv = Vector3.Cross(sat, st) + Vector3.Cross(sa, stt);

        Vector3 numerator = EG_FF * (saXstDeriv * sqrtEG_FF + saXst * sqrtEG_FFDeriv) -
                            saXst * sqrtEG_FF * EG_FFDeriv;
        float denominator = EG_FF * EG_FF;

        Vector3 normalDerivativeT = numerator / denominator;
        return normalDerivativeT;
    }

    // Calculate normal of envelope according to Bassegoda's paper
    // Expects t in [0, 1] and a in [0, 1]
    public Vector3 CalculateNormal_Bassegoda(float t, float a)
    {
        // tool surface derivative wrt a
        Vector3 sa = /* toolHeight * */ GetToolAxisAt(t);
        // tool surface derivative wrt t
        Vector3 st = GetToolPathDerivativeAt(t) + a /* * toolHeight */ * GetToolAxisDerivativeAt(t);
        Vector3 sNormal = Vector3.Cross(sa, st).normalized;

        // tool radius derivate wrt a
        float ra = GetToolRadiusDerivativeAt(a);
        float alpha, beta, gamma;
        // Determinant of partial derivative matrix
        float determinant = (Vector3.Dot(sa, sa) * Vector3.Dot(st, st)) - (Vector3.Dot(sa, st) * Vector3.Dot(st, sa));
        float m11 = Vector3.Dot(st, st) / determinant;
        alpha = m11 * -ra;
        float m21 = -Vector3.Dot(st, sa) / determinant;
        beta = m21 * -ra;
        gamma = (determinant > 0 ? 1 : -1) * Mathf.Sqrt(1 - ra * ra * m11);

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
            float distance = 0;
            for (int tIdx = 0; tIdx <= tSectors; tIdx++)
            {
                float t = (float)tIdx / tSectors;
                Vector3 axis = adjacentEnvelopeA1.GetEnvelopeAt_Bassegoda(t, 0) - adjacentEnvelopeA0.GetEnvelopeAt_Bassegoda(t, 1);
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
            // Vector3 p = GetEnvelopeAt(t, 0);
            // Vector3 pt = GetEnvelopeDerivativeTAt(t, 0);
            // Vector3 n = CalculateNormal_Rajain(t, 0);
            // Vector3 nt = CalculateNormalDerivativeT_Rajain(t, 0);
            // Gizmos.color = Color.red;
            // Gizmos.DrawLine(p, p + n);
            // Gizmos.color = Color.green;
            // Gizmos.DrawLine(p + n, p + n + nt);
            // Gizmos.color = Color.blue;
            // Gizmos.DrawLine(p, p + pt);

            Vector3 p = GetToolPathAt(t);
            Vector3 axis = GetToolAxisAt(t);
            Gizmos.color = Color.green;
            Gizmos.DrawLine(p, p + axis);
            Vector3 s = p + a * axis;
            Vector3 x = GetEnvelopeAt(t, a);
            Gizmos.color = Color.green;
            Gizmos.DrawLine(s, x);
        }
    }

    private void OnValidate()
    {
        if (adjacentEnvelopeA0 == this) adjacentEnvelopeA0 = null;
        if (adjacentEnvelopeA1 == this || adjacentEnvelopeA1 == adjacentEnvelopeA0) adjacentEnvelopeA1 = null;
    }
}
