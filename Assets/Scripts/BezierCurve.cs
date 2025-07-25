using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(LineRenderer))]
public class BezierCurve : MonoBehaviour
{
    [SerializeField]
    private List<Vector3> points;
    private LineRenderer lineRenderer;
    public int resolution = 10;
    public float lineThickness = 0.1f;

    public int NumSegments
    {
        get
        {
            return (points.Count - 4) / 3 + 1;
        }
    }

    private bool showControlPoints = true;

    private void Awake()
    {
        lineRenderer = GetComponent<LineRenderer>();

        points = new List<Vector3>{
            Vector3.left,
            Vector3.left/3,
            Vector3.right/3,
            Vector3.right
        };
        string parentName = GetComponentInParent<Envelope>()?.gameObject.name;
        for (int i = 0; i < points.Count; i++)
        {
            if (parentName == "Envelope Perfect Fit") points[i] -= new Vector3(0, 0.1f * i, 0);
            else if (parentName == "Envelope Curve Base") points[i] += new Vector3(0, 0, (Mathf.Pow(2.0f, i) - 1) / 10);
        }

        for (int i = 0; i < points.Count; i++)
        {
            points[i] += transform.position;
        }

        lineRenderer.widthMultiplier = lineThickness;
        UpdateLineRenderer();
    }

    public void AddSegment(Vector3 anchorPos)
    {
        // Add extra control points
        Vector3 newControl1 = points[points.Count - 1] * 2 - points[points.Count - 2];
        Vector3 newControl2 = (newControl1 + anchorPos) / 2;
        points.Add(newControl1);
        points.Add(newControl2);

        // Add extra anchor point
        points.Add(anchorPos);
    }

    // Expect t in range of [0, 1]
    public Vector3 Evaluate(float t)
    {
        t *= NumSegments;
        t = Mathf.Clamp(t, 0, NumSegments);
        // Determine which segment to interpolate based on t
        int segment = Mathf.Max(Mathf.FloorToInt(t - 0.00001f), 0);
        // Get the decimal part of t
        t -= segment;

        return points[segment * 3 + 0] * (-t * t * t + 3 * t * t - 3 * t + 1) +
               points[segment * 3 + 1] * (3 * t * t * t - 6 * t * t + 3 * t) +
               points[segment * 3 + 2] * (-3 * t * t * t + 3 * t * t) +
               points[segment * 3 + 3] * t * t * t;
    }

    // Expects t in range of [0, 1]
    public Vector3 EvaluateDerivative(float t)
    {
        t *= NumSegments;
        t = Mathf.Clamp(t, 0, NumSegments);
        // Determine which segment to interpolate based on t
        int segment = Mathf.Max(Mathf.FloorToInt(t - 0.00001f), 0);
        // Get the decimal part of t
        t -= segment;

        return points[segment * 3 + 0] * (-3 * t * t + 6 * t - 3) +
               points[segment * 3 + 1] * (9 * t * t - 12 * t + 3) +
               points[segment * 3 + 2] * (-9 * t * t + 6 * t) +
               points[segment * 3 + 3] * (3 * t * t);
    }

    public Vector3 EvaluateDerivative2(float t)
    {
        t *= NumSegments;
        t = Mathf.Clamp(t, 0, NumSegments);
        // Determine which segment to interpolate based on t
        int segment = Mathf.Max(Mathf.FloorToInt(t - 0.00001f), 0);
        // Get the decimal part of t
        t -= segment;

        return points[segment * 3 + 0] * (-6 * t + 6) +
               points[segment * 3 + 1] * (18 * t - 12) +
               points[segment * 3 + 2] * (-18 * t + 6) +
               points[segment * 3 + 3] * (6 * t);
    }

    public Vector3 EvaluateDerivative3(float t)
    {
        t *= NumSegments;
        t = Mathf.Clamp(t, 0, NumSegments);
        // Determine which segment to interpolate based on t
        int segment = Mathf.Max(Mathf.FloorToInt(t - 0.00001f), 0);
        // Get the decimal part of t
        t -= segment;

        return points[segment * 3 + 0] * -6 +
               points[segment * 3 + 1] * 18 +
               points[segment * 3 + 2] * -18 +
               points[segment * 3 + 3] * 6;
    }

    public Vector3 EvaluateDerivative4Plus(float t)
    {
        return Vector3.zero;
    }

    public void UpdateLineRenderer()
    {
        List<Vector3> renderPoints = new();

        for (int i = 0; i <= NumSegments * resolution; i++)
        {
            renderPoints.Add(Evaluate(1.0f / (NumSegments * resolution) * i));
        }

        UpdateLineRenderer(renderPoints);
        showControlPoints = true;
    }

    public void UpdateLineRenderer(List<Vector3> points)
    {
        lineRenderer.positionCount = points.Count;
        lineRenderer.SetPositions(points.ToArray());
        showControlPoints = false;
    }

    private void OnDrawGizmosSelected()
    {
        if (showControlPoints)
        {
            Gizmos.color = Color.white;
            for (int i = 0; i < points.Count; i++)
            {
                Gizmos.DrawSphere(points[i], 0.05f);
            }
            for (int i = 0; i < points.Count - 1; i++)
            {
                Gizmos.DrawLine(points[i], points[i + 1]);
            }
        }
    }

}
