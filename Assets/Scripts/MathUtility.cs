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
        float l = a.magnitude;
        float dl = Vector3.Dot(a, da) / l;
        return (l * da - a * dl) / (l * l);
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
        float l = a.magnitude;
        float dl = Vector3.Dot(a, da) / l;
        float ddl = (l * (Vector3.Dot(da, da) + Vector3.Dot(a, dda)) - Vector3.Dot(a, da) * dl) / (l * l);

        Vector3 c = l * da - a * dl;
        Vector3 dc = l * dda - a * ddl;
        return (l * l * dc - c * 2 * l * dl) / (l * l * l * l);
    }
}