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
}