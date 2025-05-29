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
        angle *= -1;

        return Quaternion.AngleAxis(angle * Mathf.Rad2Deg, perpVec) * k;
    }

    public static Vector3 VectorPerpAndDotProductDt(Vector3 perpVec, Vector3 perpVecDt, Vector3 dotVec, Vector3 dotVecDt, float dotValue)
    {
        // Derived from the rotation matrix of an angle around an arbitrary unit axis, and trigonometric identities.
        Vector3 f = perpVec.normalized;
        Vector3 f_t = NormalVectorDerivative(perpVec, perpVecDt);
        Vector3 g = dotVec.normalized;
        Vector3 g_t = NormalVectorDerivative(dotVec, dotVecDt);
        Vector3 k = Vector3.Cross(perpVec, dotVec).normalized;
        Vector3 k_t = Vector3.Cross(perpVecDt, dotVec) + Vector3.Cross(perpVec, dotVecDt);
        k_t = NormalVectorDerivative(Vector3.Cross(perpVec, dotVec), k_t);

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

        float A_t = (k_t.x * g.x * (1 - f.x * f.x) + k.x * g_t.x * (1 - f.x * f.x) + k.x * g.x * -2 * f.x * f_t.x) +
                    -(k_t.x * g.x * f.x * f.y + k.x * g_t.y * f.x * f.y + k.x * g.y * f_t.x * f.y + k.x * g.y * f.x * f_t.y) +
                    -(k_t.x * g.z * f.x * f.z + k.x * g_t.z * f.x * f.z + k.x * g.z * f_t.x * f.z + k.x * g.z * f.x * f_t.z) +
                    -(k_t.y * g.x * f.x * f.y + k.y * g_t.x * f.x * f.y + k.y * g.x * f_t.x * f.y + k.y * g.x * f.x * f_t.y) +
                    (k_t.y * g.y * (1 - f.y * f.y) + k.y * g_t.y * (1 - f.y * f.y) + k.y * g.y * -2 * f.y * f_t.y) -
                    -(k_t.y * g.z * f.y * f.z + k.y * g_t.z * f.y * f.z + k.y * g.z * f_t.y * f.z + k.y * g.z * f.y * f_t.z) +
                    -(k_t.z * g.x * f.x * f.z + k.z * g_t.x * f.x * f.z + k.z * g.x * f_t.x * f.z + k.z * g.x * f.x * f_t.z) +
                    -(k_t.z * g.y * f.y * f.z + k.z * g_t.y * f.y * f.z + k.z * g.y * f_t.y * f.z + k.z * g.y * f.y * f_t.z) +
                    (k_t.z * g.z * (1 - f.z * f.z) + k.z * g_t.z * (1 - f.z * f.z) + k.z * g.z * -2 * f.z * f_t.z);
        float B_t = -(k_t.x * g.y * f.z + k.x * g_t.y * f.z + k.x * g.y * f_t.z) +
                    (k_t.x * g.z * f.y + k.x * g_t.z * f.y + k.x * g.z * f_t.y) +
                    (k_t.y * g.x * f.z + k.y * g_t.x * f.z + k.y * g.x * f_t.z) +
                    -(k_t.y * g.z * f.x + k.y * g_t.z * f.x + k.y * g.z * f_t.x) +
                    -(k_t.z * g.x * f.y + k.z * g_t.x * f.y + k.z * g.x * f_t.y) +
                    (k_t.z * g.y * f.x + k.z * g_t.y * f.x + k.z * g.y * f_t.x);
        float C_t = (k_t.x * g.x * f.x * f.x + k.x * g_t.x * f.x * f.x + k.x * g.x * f_t.x * f.x + k.x * g.x * f.x * f_t.x) +
                    (k_t.x * g.y * f.x * f.y + k.x * g_t.y * f.x * f.y + k.x * g.y * f_t.x * f.y + k.x * g.y * f.x * f_t.y) +
                    (k_t.x * g.z * f.x * f.z + k.x * g_t.z * f.x * f.z + k.x * g.z * f_t.x * f.z + k.x * g.z * f.x * f_t.z) +
                    (k_t.y * g.x * f.x * f.y + k.y * g_t.x * f.x * f.y + k.y * g.x * f_t.x * f.y + k.y * g.x * f.x * f_t.y) +
                    (k_t.y * g.y * f.y * f.y + k.y * g_t.y * f.y * f.y + k.y * g.y * f_t.y * f.y + k.y * g.y * f.y * f_t.y) +
                    (k_t.y * g.z * f.y * f.z + k.y * g_t.z * f.y * f.z + k.y * g.z * f_t.y * f.z + k.y * g.z * f.y * f_t.z) +
                    (k_t.z * g.x * f.x * f.z + k.z * g_t.x * f.x * f.z + k.z * g.x * f_t.x * f.z + k.z * g.x * f.x * f_t.z) +
                    (k_t.z * g.y * f.y * f.z + k.z * g_t.y * f.y * f.z + k.z * g.y * f_t.y * f.z + k.z * g.y * f.y * f_t.z) +
                    (k_t.z * g.z * f.z * f.z + k.z * g_t.z * f.z * f.z + k.z * g.z * f_t.z * f.z + k.z * g.z * f.z * f_t.z);

        float acos_arg = -C / Mathf.Sqrt(A * A + B * B);
        float acos_arg_t = (-Mathf.Sqrt(A * A + B * B) * C_t + C * (A + B) / Mathf.Sqrt(A * A + B * B)) / (A * A + B * B);
        float angle = Mathf.Acos(acos_arg) + Mathf.Atan2(B, A);
        angle *= -1;
        float angle_t = -acos_arg_t / Mathf.Sqrt(1 - acos_arg) + (-B * A_t + A * B_t) / (A * A + B * B);
        angle_t *= -1;

        Matrix4x4 R = Matrix4x4.Rotate(Quaternion.AngleAxis(Mathf.Rad2Deg * angle, perpVec));

        // The derivative of the rotation matrix is a doozy. As the rotation matrix is a function of 4 variables
        // (technically 2, but 1 is a vec3), it has 4 partial derivatives, which are each multiplied by their 
        // variable's derivative and summed together for the derivative wrt t.
        float cos = Mathf.Cos(angle);
        float sin = Mathf.Sin(angle);

        Matrix4x4 R_t = new();
        R_t[0, 0] = 2 * f.x * f_t.x * (1 - cos) + 0 + 0 - angle_t * (1 - f.x * f.x) * sin;
        R_t[0, 1] = f_t.x * f.y * (1 - cos) + f.x * f_t.y * (1 - cos) - f_t.z * sin + angle_t * (f.x * f.y * sin - f.z * cos);
        R_t[0, 2] = f_t.x * f.z * (1 - cos) + f_t.y * sin + f.x * f_t.z * (1 - cos) + angle_t * (f.x * f.z * sin + f.y * cos);
        R_t[0, 3] = 0;

        R_t[1, 0] = f_t.x * f.y * (1 - cos) + f.x * f_t.y * (1 - cos) + f_t.z * sin + angle_t * (f.x * f.y * sin + f.z * cos);
        R_t[1, 1] = 0 + 2 * f.y * f_t.y * (1 - cos) + 0 - angle_t * (1 - f.y * f.y) * sin;
        R_t[1, 2] = -f_t.x * sin + f_t.y * f.z * (1 - cos) + f.y * f_t.z * (1 - cos) + angle_t * (f.y * f.z * sin - f.x * cos);
        R_t[1, 3] = 0;

        R_t[2, 0] = f_t.x * f.z * (1 - cos) - f_t.y * sin + f.x * f_t.z * (1 - cos) + angle_t * (f.x * f.z * sin - f.y * cos);
        R_t[2, 1] = f_t.x * sin + f_t.y * f.z * (1 - cos) + f.y * f_t.z * (1 - cos) + angle_t * (f.y * f.z * sin + f.x * cos);
        R_t[2, 2] = 0 + 0 + 2 * f.z * f_t.z * (1 - cos) - angle_t * (1 - f.z * f.z) * sin;
        R_t[2, 3] = 0;

        R_t[3, 0] = 0;
        R_t[3, 1] = 0;
        R_t[3, 2] = 0;
        R_t[3, 3] = 1;

        return R_t.MultiplyVector(k) + R.MultiplyVector(k_t);
    }
}