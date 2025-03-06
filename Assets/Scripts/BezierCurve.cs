using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(LineRenderer))]
public class BezierCurve : MonoBehaviour
{
    [SerializeField]
    public List<Vector3> points;
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

    [SerializeField]
    [Range(0, 1)]
    private float t;

    float tb = 1.0f / 3;
    float tc = 2.0f / 3;
    private void Awake()
    {
        lineRenderer = GetComponent<LineRenderer>();
        points = new List<Vector3>{
            Vector3.left,
            (Vector3.left + Vector3.up)/2,
            (Vector3.right + Vector3.down)/2,
            Vector3.right
        };

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

        return points[segment * 3 + 0] * (1 - t) * (1 - t) * (1 - t) +
               points[segment * 3 + 1] * 3 * t * (1 - t) * (1 - t) +
               points[segment * 3 + 2] * 3 * t * t * (1 - t) +
               points[segment * 3 + 3] * t * t * t;
    }

    // Expects t in range of [0, 1]
    public Vector3 EvaluateTangent(float t)
    {
        t *= NumSegments;
        t = Mathf.Clamp(t, 0, NumSegments);
        // Determine which segment to interpolate based on t
        int segment = Mathf.Max(Mathf.FloorToInt(t - 0.00001f), 0);
        // Get the decimal part of t
        t -= segment;

        return points[segment * 3 + 0] * -3 * (1 - t) * (1 - t) +
               points[segment * 3 + 1] * (-6 * t * (1 - t) + 3 * (1 - t) * (1 - t)) +
               points[segment * 3 + 2] * (-3 * t * t + 6 * t * (1 - t)) +
               points[segment * 3 + 3] * 3 * t * t;
    }

    public void UpdateLineRenderer()
    {
        List<Vector3> renderPoints = new();

        for (int i = 0; i <= NumSegments * resolution; i++)
        {
            renderPoints.Add(Evaluate(1.0f / (NumSegments * resolution) * i));
        }

        UpdateLineRenderer(renderPoints);
    }

    public void UpdateLineRenderer(List<Vector3> points)
    {
        lineRenderer.positionCount = points.Count;
        lineRenderer.SetPositions(points.ToArray());
    }

    private void OnDrawGizmosSelected()
    {
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
