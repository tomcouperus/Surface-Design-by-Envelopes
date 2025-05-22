using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(Envelope))]
public class EnvelopeEditor : Editor
{
    private Envelope envelope;


    private GUIContent updateButton = new GUIContent("Update");

    private void OnEnable()
    {
        envelope = (Envelope)target;
    }

    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        if (GUILayout.Button(updateButton))
        {
            envelope.UpdatePath();
            envelope.UpdateEnvelope();
            envelope.tool.UpdateTool();
        }
        if (GUILayout.Button("Test"))
        {
            Debug.Log("Axis: " + envelope.GetToolAxisAt(envelope.getT()));
            Debug.Log("d/dt Axis: " + envelope.GetToolAxisDtAt(envelope.getT()));
            Debug.Log("d2/dt2 Axis: " + envelope.GetToolAxisDt2At(envelope.getT()));
            Debug.Log("Normal: " + envelope.CalculateNormalAt(envelope.getT(), envelope.getA()));
            Debug.Log("d/dt Normal: " + envelope.CalculateNormalDtAt(envelope.getT(), envelope.getA()));
        }
    }
}
