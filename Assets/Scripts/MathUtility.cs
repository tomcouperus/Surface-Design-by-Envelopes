using Unity.VisualScripting;
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
    /// Calculates the fourth derivative of a normalized vector.
    /// </summary>
    /// <param name="a">The unnormalized vector</param>
    /// <param name="da">The derivative of the unnormalized vector</param>
    /// <param name="dda">The second derivative of the unnormalized vector</param>
    /// <param name="ddda">The third derivative of the unnormalized vector</param>
    /// <param name="dddda">The fourth derivative of the unnormalized vector</param>
    /// <returns></returns>
    public static Vector3 NormalVectorDerivative4(Vector3 a, Vector3 da, Vector3 dda, Vector3 ddda, Vector3 dddda)
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
        float dm = L * (Vector3.Dot(a, ddda) + 3 * Vector3.Dot(da, dda)) - (Vector3.Dot(a, da) * ddL);
        float n = L2 * dm - m * dL2;
        float dddL = n / L4;
        Vector3 ddc = (dL * dda + L * ddda) - (da * ddL + a * dddL);
        float ddL2 = 2 * (dL * dL + L * ddL);
        Vector3 df = L2 * ddc - c * ddL2;
        float L8 = L4 * L4;
        float dL4 = 4 * Mathf.Pow(L, 3) * dL;
        Vector3 g = L4 * df - f * dL4;
        Vector3 dddb = g / L8;

        // Fourth derivative ddddb
        float ddm = (dL * (Vector3.Dot(a, ddda) + 3 * Vector3.Dot(da, dda)) + L * (Vector3.Dot(a, dddda) + 3 * Vector3.Dot(dda, dda) + 4 * Vector3.Dot(da, ddda))) - ((Vector3.Dot(da, da) + Vector3.Dot(a, dda)) * ddL + Vector3.Dot(a, da) * dddL);
        float dn = L2 * ddm - m * ddL2;
        float ddddL = L4 * dn - n * dL4 / L8;
        Vector3 dddc = (ddL * dda + dL * ddda + dL * ddda + L * dddda) - (dda * ddL + da * dddL + da * dddL + a * ddddL);
        float dddL2 = 2 * (2 * dL * ddL + dL * ddL + L * dddL);
        Vector3 ddf = (dL2 * ddc + L2 * dddc) - (dc * ddL2 + c * dddL2);
        float ddL4 = 4 * (3 * Mathf.Pow(L, 2) * dL + Mathf.Pow(L, 3) * ddL);
        Vector3 dg = L4 * ddf - f * ddL4;
        float L16 = L8 * L8;
        float dL8 = 8 * Mathf.Pow(L, 7) * dL;
        Vector3 h = L8 * dg - g * dL8;
        Vector3 ddddb = h / L16;

        return ddddb;
    }
}