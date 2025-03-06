using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(BezierCurve))]
public class BezierCurveEditor : Editor
{
    private BezierCurve bezierCurve;


    private GUIContent addSegmentButton = new GUIContent("Add Segment");
    private GUIContent updateButton = new GUIContent("Update");

    private void OnEnable()
    {
        bezierCurve = (BezierCurve)target;
    }

    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        // if (GUILayout.Button(addSegmentButton))
        // {
        //     bezierCurve.AddSegment(new Vector3(2, -2, 0));
        //     bezierCurve.UpdateLineRenderer();
        // }

        if (GUILayout.Button(updateButton))
        {
            bezierCurve.UpdateLineRenderer();
        }
    }
}
