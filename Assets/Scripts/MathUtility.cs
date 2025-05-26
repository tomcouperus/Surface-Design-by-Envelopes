using UnityEngine;

public class MathUtility
{
    /// <summary>
    /// Calculates the derivative of a normalized vector.
    /// </summary>
    /// <param name="a">The unnormalized vector</param>
    /// <param name="da">The derivative of the unnormalized vector</param>
    /// <returns></returns>
    public static Vector3 NormalVectorDerivative(Vector3 a, Vector3 da)
    {
        // Unit vector b
        float L = a.magnitude;
        Vector3 b = a / L;

        // First derivative db
        float dL = Vector3.Dot(a, da) / L;
        Vector3 c = L * da - a * dL;
        float L2 = L * L;
        Vector3 db = c / L2;
        return db;
    }

    /// <summary>
    /// Calculates the second derivative of a normalized vector.
    /// </summary>
    /// <param name="a">The unnormalized vector</param>
    /// <param name="da">The derivative of the unnormalized vector</param>
    /// <param name="dda">The second derivative of the unnormalized vector</param>
    /// <returns></returns>
    public static Vector3 NormalVectorDerivative2(Vector3 a, Vector3 da, Vector3 dda)
    {
        // Unit vector b
        float L = a.magnitude;
        Vector3 b = a / L;

        // First derivative db
        float dL = Vector3.Dot(a, da) / L;
        Vector3 c = L * da - a * dL;
        float L2 = L * L;
        Vector3 db = c / L2;

        // Second derivative ddb
        float m = L * (Vector3.Dot(da, da) + Vector3.Dot(a, dda)) - Vector3.Dot(a, da) * dL;
        float ddL = m / L2;
        Vector3 dc = L * dda - a * ddL;
        float L4 = L2 * L2;
        float dL2 = 2 * L * dL;
        Vector3 f = L2 * dc - c * dL2;
        Vector3 ddb = f / L4;
        return ddb;
    }

    /// <summary>
    /// Calculates the third derivative of a normalized vector.
    /// </summary>
    /// <param name="a">The unnormalized vector</param>
    /// <param name="da">The derivative of the unnormalized vector</param>
    /// <param name="dda">The second derivative of the unnormalized vector</param>
    /// <param name="ddda">The third derivative of the unnormalized vector</param>
    /// <returns></returns>
    public static Vector3 NormalVectorDerivative3(Vector3 a, Vector3 da, Vector3 dda, Vector3 ddda)
    {
        // Unit vector b
        float L = a.magnitude;
        Vector3 b = a / L;

        // First derivative db
        float dL = Vector3.Dot(a, da) / L;
        Vector3 c = L * da - a * dL;
        float L2 = L * L;
        Vector3 db = c / L2;

        // Second derivative ddb
        float m = L * (Vector3.Dot(da, da) + Vector3.Dot(a, dda)) - Vector3.Dot(a, da) * dL;
        float ddL = m / L2;
        Vector3 dc = L * dda - a * ddL;
        float L4 = L2 * L2;
        float dL2 = 2 * L * dL;
        Vector3 f = L2 * dc - c * dL2;
        Vector3 ddb = f / L4;

        // Third derivative dddb
        float dm = L * (Vector3.Dot(a, ddda) + 3 * Vector3.Dot(da, dda)) - Vector3.Dot(a, da) * ddL;
        float dddL = (L2 * dm - m * dL2) / L4;
        Vector3 ddc = (dL * dda + L * ddda) - (da * ddL + a * dddL);
        float ddL2 = 2 * (dL * dL + L * ddL);
        Vector3 df = L2 * ddc - c * ddL2;
        float L8 = L4 * L4;
        float dL4 = 4 * (L * L * L) * dL;
        Vector3 dddb = (L4 * df - f * dL4) / L8;

        return dddb;
    }

    /// <summary>
    /// Find the unit vector that is perpendicular to one vector, and at an angle to another.
    /// I.e.
    /// Find U such that U dot F = 0 and U dot G = p, where F, G, and p are known.
    /// </summary>
    /// <param name="perpVec"></param>
    /// <param name="dotVec"></param>
    /// <param name="dotValue"></param>
    /// <returns></returns>
    public static Vector3 VectorPerpAndDotProduct(Vector3 perpVec, Vector3 dotVec, float dotValue)
    {
        // Derived from the rotation matrix of an angle around an arbitrary unit axis, and trigonometric identities.
        Vector3 f = perpVec.normalized;
        Vector3 g = dotVec.normalized;
        Vector3 k = Vector3.Cross(f, g).normalized;

        float A = k.x * g.x * (1 - f.x * f.x) - k.x * g.y * f.x * f.y - k.x * g.z * f.x * f.z +
                  -k.y * g.x * f.x * f.y + k.y * g.y * (1 - f.y * f.y) - k.y * g.z * f.y * f.z +
                  -k.z * g.x * f.x * f.z - k.z * g.y * f.y * f.z + k.z * g.z * (1 - f.z * f.z);
        float B = -k.x * g.y * f.z + k.x * g.z * f.y +
                  k.y * g.x * f.z - k.y * g.z * f.x +
                  -k.z * g.x * f.y + k.z * g.y * f.x;
        float C = k.x * g.x * f.x * f.x + k.x * g.y * f.x * f.y + k.x * g.z * f.x * f.z +
                  k.y * g.x * f.x * f.y + k.y * g.y * f.y * f.y + k.y * g.z * f.y * f.z +
                  k.z * g.x * f.x * f.z + k.z * g.y * f.y * f.z + k.z * g.z * f.z * f.z -
                  dotValue;

        float angle = Mathf.Acos(-C / Mathf.Sqrt(A * A + B * B)) + Mathf.Atan2(B, A);

        return Quaternion.AngleAxis(-angle * Mathf.Rad2Deg, perpVec) * k;
    }
}