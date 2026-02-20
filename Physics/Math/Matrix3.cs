
namespace Physics.Math;

public class Matrix3
{
    // Row-major
    public double M00, M01, M02;
    public double M10, M11, M12;
    public double M20, M21, M22;

    public Matrix3(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
    {
        M00 = m00;
        M01 = m01;
        M02 = m02;
       
        M10 = m10;
        M11 = m11;
        M12 = m12;
        
        M20 = m20;
        M21 = m21;
        M22 = m22;
    }

    public static Matrix3 Zero => new(
        0d, 0d, 0d,
        0d, 0d, 0d,
        0d, 0d, 0d);

    public static readonly Matrix3 Identity = new(
        1d, 0d, 0d,
        0d, 1d, 0d,
        0d, 0d, 1d);

    public Matrix3 Transposed() => new(
        M00, M10, M20,
        M01, M11, M21,
        M02, M12, M22);

    public static Vector3 operator *(Matrix3 m, Vector3 v)
    {
        return new Vector3(
            m.M00 * v.X + m.M01 * v.Y + m.M02 * v.Z,
            m.M10 * v.X + m.M11 * v.Y + m.M12 * v.Z,
            m.M20 * v.X + m.M21 * v.Y + m.M22 * v.Z
            );
    }

    public static Matrix3 operator *(Matrix3 m1, Matrix3 m2)
    {
        return new Matrix3(
            m1.M00 * m2.M00 + m1.M01 * m2.M10 + m1.M02 * m2.M20,
            m1.M00 * m2.M01 + m1.M01 * m2.M11 + m1.M02 * m2.M21,
            m1.M00 * m2.M02 + m1.M01 * m2.M12 + m1.M02 * m2.M22,

            m1.M10 * m2.M00 + m1.M11 * m2.M10 + m1.M12 * m2.M20,
            m1.M10 * m2.M01 + m1.M11 * m2.M11 + m1.M12 * m2.M21,
            m1.M10 * m2.M02 + m1.M11 * m2.M12 + m1.M12 * m2.M22,

            m1.M20 * m2.M00 + m1.M21 * m2.M10 + m1.M22 * m2.M20,
            m1.M20 * m2.M01 + m1.M21 * m2.M11 + m1.M22 * m2.M21,
            m1.M20 * m2.M02 + m1.M21 * m2.M12 + m1.M22 * m2.M22


            );
    }

    public static Matrix3 FromQuaternion(Quaternion q)
    {
        // Assumes q is normalized
        double x = q.X, y = q.Y, z = q.Z, w = q.W;

        double xx = x * x, yy = y * y, zz = z * z;
        double xy = x * y, xz = x * z, yz = y * z;
        double wx = w * x, wy = w * y, wz = w * z;

        return new Matrix3(
            1d - 2d * (yy + zz),    2d * (xy - wz),         2d * (xz + wy),
            2d * (xy + wz),         1d - 2d * (xx + zz),    2d * (yz - wx),
            2d * (xz - wy),         2d * (yz + wx),         1d - 2d * (xx + yy)

            );
    }

    public static Matrix3 operator *(Matrix3 m, double s) => new(
        m.M00 * s, m.M01 * s, m.M02 * s,
        m.M10 * s, m.M11 * s, m.M12 * s,
        m.M20 * s, m.M21 * s, m.M22 * s
        );
}
