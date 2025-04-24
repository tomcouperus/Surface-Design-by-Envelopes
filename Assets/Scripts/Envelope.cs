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

    public Vector3 GetToolAxisDerivativeAt(float t)
    {
        Vector3 axisLerp = Vector3.Lerp(toolAxisT0, toolAxisT1, t);
        Vector3 axisLerpDeriv = toolAxisT1 - toolAxisT0;
        Vector3 axisDeriv = (axisLerp.magnitude * axisLerpDeriv - (axisLerp * Vector3.Dot(axisLerp, axisLerpDeriv) / axisLerp.magnitude)) / axisLerp.sqrMagnitude;
        return axisDeriv;
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
            Vector3 axis_t = GetToolAxisDerivativeAt(t);
            Vector3 s = p + a * axis;
            Vector3 x = GetEnvelopeAt(t, a);
            // Axis
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(p, p + axis);
            // Axis derivative
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(p, p + axis_t);
            // Normal
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
