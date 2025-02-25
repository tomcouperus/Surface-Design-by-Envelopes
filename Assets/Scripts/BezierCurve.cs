using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(LineRenderer))]
public class BezierCurve : MonoBehaviour
{
    private List<Vector3> points;
    private LineRenderer lineRenderer;
    public int resolution = 10;

    public int NumSegments
    {
        get
        {
            return (points.Count - 4) / 3 + 1;
        }
    }

    private void Awake()
    {
        lineRenderer = GetComponent<LineRenderer>();
        points = new List<Vector3>{
            Vector3.left,
            (Vector3.left + Vector3.up)/2,
            (Vector3.right + Vector3.down)/2,
            Vector3.right
        };

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

    public Vector3 Evaluate(float t)
    {
        Debug.Log("Evaluating point");
        Debug.Log(t);
        t = Mathf.Clamp(t, 0, NumSegments);
        Debug.Log(t);
        // Determine which segment to interpolate based on t
        int segment = Mathf.Max(Mathf.FloorToInt(t - 0.0001f), 0);
        Debug.Log(segment);
        // Get the decimal part of t
        t -= segment;
        Debug.Log(t);

        return points[segment * 3 + 0] * (1 - t) * (1 - t) * (1 - t) +
               points[segment * 3 + 1] * 3 * t * (1 - t) * (1 - t) +
               points[segment * 3 + 2] * 3 * t * t * (1 - t) +
               points[segment * 3 + 3] * t * t * t;
    }

    public void UpdateLineRenderer()
    {
        List<Vector3> renderPoints = new();

        float deltaT = 1.0f / resolution;
        for (int i = 0; i < NumSegments * resolution + 1; i++)
        {
            renderPoints.Add(Evaluate(i * deltaT));
        }

        lineRenderer.positionCount = renderPoints.Count;
        lineRenderer.SetPositions(renderPoints.ToArray());
    }

}
