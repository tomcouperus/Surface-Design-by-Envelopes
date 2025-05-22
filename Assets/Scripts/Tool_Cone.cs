using UnityEngine;

public class Tool_Cone : Tool
{
    [SerializeField]
    [Range(0, 45)]
    private float openingAngle;
    private float OpeningRad { get { return Mathf.Deg2Rad * openingAngle; } }

    public override float GetSphereCenterHeightAt(float a)
    {
        return a * height + GetToolRadiusAt(a) * Mathf.Tan(OpeningRad);
    }

    public override float GetSphereCenterHeightDaAt(float a)
    {
        return height + GetToolRadiusDaAt(a) * Mathf.Tan(OpeningRad);
    }

    public override float GetSphereRadiusAt(float a)
    {
        return GetToolRadiusAt(a) / Mathf.Cos(OpeningRad);
    }

    public override float GetSphereRadiusDaAt(float a)
    {
        return GetToolRadiusDaAt(a) / Mathf.Cos(OpeningRad);
    }

    protected override float GetToolRadiusAt(float a)
    {
        return radius + a * height * Mathf.Tan(OpeningRad);
    }

    protected override float GetToolRadiusDaAt(float a)
    {
        return height * Mathf.Tan(OpeningRad);
    }

    protected override Vector3 GetToolSurfaceAt(float t, float a)
    {
        float tRad = t * 2 * Mathf.PI;
        float r = GetToolRadiusAt(a);
        return new Vector3(
            r * Mathf.Cos(tRad),
            a * height,
            r * Mathf.Sin(tRad)
        );
    }
}
