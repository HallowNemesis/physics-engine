
namespace Physics.Math;

public struct Quaternion
{
    public double X, Y, Z, W;

    public Quaternion(double x, double y, double z, double w)
    {
        this.X = x;
        this.Y = y;
        this.Z = z;
        this.W = w;
    }

    public static Quaternion Identity => new(0d, 0d, 0d, 1d);

    public Quaternion Normalized()
    {
        double m2 = X * X + Y * Y + Z * Z + W * W;
        if (m2 <= 1e-12f) return Identity;
        double inv = 1f / double.Sqrt(m2);
        return new Quaternion(X * inv, Y * inv, Z * inv, W * inv);
    }

    public static Quaternion operator *(Quaternion q1, Quaternion q2)
    {
        return new Quaternion(
            q1.W * q2.X + q1.X * q2.W + q1.Y * q2.Z - q1.Z * q2.Y,
            q1.W * q2.Y - q1.X * q2.Z + q1.Y * q2.W + q1.Z * q2.X,
            q1.W * q2.Z + q1.X * q2.Y - q1.Y * q2.X + q1.Z * q2.W,
            q1.W * q2.W - q1.X * q2.X - q1.Y * q2.Y - q1.Z * q2.Z
            );
    }

    //Integrate orientation using angular velocity (world-space) over dt
    // q' = 0.5 * omegaQuat * q
    public static Quaternion Integrate(Quaternion q, Vector3 angularVelocity, double dt)
    {
        var omega = new Quaternion(angularVelocity.X, angularVelocity.Y, angularVelocity.Z, 0d);
        Quaternion dq = omega * q;

        q.X += 0.5d * dq.X * dt;
        q.Y += 0.5d * dq.Y * dt;
        q.Z += 0.5d * dq.Z * dt;
        q.W += 0.5d * dq.W * dt;

        return q;
    }

    public static Quaternion FromAxisAngle(Vector3 axis, double angle)
    {
        // Ensure axis is normalized
        double len = axis.Length();
        if (len <= 1e-12)
            return Identity;

        double invLen = 1.0 / len;
        double ax = axis.X * invLen;
        double ay = axis.Y * invLen;
        double az = axis.Z * invLen;

        double half = angle * 0.5;
        double s = System.Math.Sin(half);
        double c = System.Math.Cos(half);

        return new Quaternion(
            ax * s,
            ay * s,
            az * s,
            c
        );
    }

}
