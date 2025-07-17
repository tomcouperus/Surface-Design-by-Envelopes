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
    }
}
