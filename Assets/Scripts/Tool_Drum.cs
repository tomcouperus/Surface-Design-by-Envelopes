using UnityEngine;

public class Tool_Drum : Tool
{
    [SerializeField]
    private float radius = 0.5f;
    [SerializeField]
    private float circleRadius = 5;

    // Helper values
    private float R { get { return circleRadius; } }
    private float B { get { return R - Mathf.Sqrt(R * R - height * height / 4); } }
    private float R_b { get { return radius; } }
    private float R_c { get { return R_b + B; } }
    private float V { get { return R - R_c; } }

    private float GetAngleAt(float a)
    {
        return Mathf.Asin(a / R);
    }

    private float GetAngleDaAt(float a)
    {
        throw new System.NotImplementedException();

    }

    public override float GetRadiusAt(float a)
    {
        float theta = GetAngleAt(a);
        float sphRad = GetSphereRadiusAt(a);
        return sphRad * Mathf.Cos(theta);
    }

    public override float GetRadiusDaAt(float a)
    {
        throw new System.NotImplementedException();

    }

    public override float GetSphereCenterHeightAt(float a)
    {
        float theta = GetAngleAt(a);
        return V * Mathf.Tan(theta);
    }

    public override float GetSphereCenterHeightDaAt(float a)
    {
        throw new System.NotImplementedException();

    }

    public override float GetSphereRadiusAt(float a)
    {
        float theta = GetAngleAt(a);
        return R - V / Mathf.Cos(theta);
    }

    public override float GetSphereRadiusDaAt(float a)
    {
        throw new System.NotImplementedException();

    }

    protected override Vector3 GetToolSurfaceAt(float tRad, float a)
    {
        float toolRad = GetRadiusAt(a);
        return new Vector3(
            toolRad * Mathf.Cos(tRad),
            a * height + height / 2,
            toolRad * Mathf.Sin(tRad)
        );
    }

    private void OnValidate()
    {
        // float p = V / Mathf.Cos(GetAngleAt(a));
        // Debug.Log("R: " + R);
        // Debug.Log("R_b: " + R_b);
        // Debug.Log("R_c: " + R_c);
        // Debug.Log("P: " + p);
        UpdateTool();
    }
}
