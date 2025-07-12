using UnityEngine;

public class Tool_Drum : Tool
{
    [SerializeField]
    private float radius = 0.5f;
    [SerializeField]
    private float rho = 5;

    // Helper values
    private float D { get { return rho - radius; } }

    private float GetAngleAt(float a)
    {
        a = a * 2 - 1;
        float term = (a * height) / (2 * rho);
        return Mathf.Asin(term);
    }

    private float GetAngleDaAt(float a)
    {
        a = a * 2 - 1;
        float term = (a * height) / (2 * rho);
        float term_a = height / (2 * rho);
        return term_a / Mathf.Sqrt(1 - term * term);

    }

    public override float GetRadiusAt(float a)
    {
        float theta = GetAngleAt(a);
        return GetSphereRadiusAt(a) * Mathf.Cos(theta);
    }

    public override float GetRadiusDaAt(float a)
    {
        float theta = GetAngleAt(a);
        float theta_a = GetAngleDaAt(a);
        float c_theta = Mathf.Cos(theta);
        float s_theta = Mathf.Sin(theta);
        return GetSphereRadiusDaAt(a) * c_theta +
               GetSphereRadiusAt(a) * -s_theta * theta_a;
    }

    public override float GetSphereCenterHeightAt(float a)
    {
        float theta = GetAngleAt(a);
        return D * Mathf.Tan(theta) + height / 2;
    }

    public override float GetSphereCenterHeightDaAt(float a)
    {
        float theta = GetAngleAt(a);
        float theta_a = GetAngleDaAt(a);
        float c_theta = Mathf.Cos(theta);
        return D / (c_theta * c_theta) * theta_a;
    }

    public override float GetSphereRadiusAt(float a)
    {
        float theta = GetAngleAt(a);
        return rho - D / Mathf.Cos(theta);
    }

    public override float GetSphereRadiusDaAt(float a)
    {
        float theta = GetAngleAt(a);
        float theta_a = GetAngleDaAt(a);
        return -D * Mathf.Tan(theta) / Mathf.Cos(theta) * theta_a;

    }

    protected override Vector3 GetToolSurfaceAt(float tRad, float a)
    {
        float toolRad = GetRadiusAt(a);
        return new Vector3(
            toolRad * Mathf.Cos(tRad),
            (a * 2 - 1) * height / 2 + height / 2,
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
