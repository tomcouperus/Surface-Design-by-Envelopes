using UnityEngine;

public class Tool_Cone : Tool
{
    [SerializeField]
    [Range(0, 45)]
    private float openingAngle;
    private float OpeningRad { get { return Mathf.Deg2Rad * openingAngle; } }

    public override float GetSphereCenterHeightAt(float a)
    {
        return a * height + GetRadiusAt(a) * Mathf.Tan(OpeningRad);
    }

    public override float GetSphereCenterHeightDaAt(float a)
    {
        return height + GetRadiusDaAt(a) * Mathf.Tan(OpeningRad);
    }

    public override float GetSphereRadiusAt(float a)
    {
        return GetRadiusAt(a) / Mathf.Cos(OpeningRad);
    }

    public override float GetSphereRadiusDaAt(float a)
    {
        return GetRadiusDaAt(a) / Mathf.Cos(OpeningRad);
    }

    public override float GetRadiusAt(float a)
    {
        return radius + a * height * Mathf.Tan(OpeningRad);
    }

    public override float GetRadiusDaAt(float a)
    {
        return height * Mathf.Tan(OpeningRad);
    }

    protected override Vector3 GetToolSurfaceAt(float t, float a)
    {
        float tRad = t * 2 * Mathf.PI;
        float r = GetRadiusAt(a);
        return new Vector3(
            r * Mathf.Cos(tRad),
            a * height,
            r * Mathf.Sin(tRad)
        );
    }
}
