
namespace Physics.Math;

public struct Vector3
{
    public double X, Y, Z;

    public Vector3(double x, double y, double z)
    {
        this.X = x;
        this.Y = y;
        this.Z = z;
    }

    public static Vector3 Zero => new Vector3(0d, 0d, 0d);
    public static Vector3 UnitX => new Vector3(1d, 0d, 0d);
    public static Vector3 UnitY => new Vector3(0d, 1d, 0d);
    public static Vector3 UnitZ => new Vector3(0d, 0d, 1d);

    public double Length() => double.Sqrt(LengthSquared());
    public double LengthSquared() => X * X + Y * Y + Z * Z;

    public Vector3 Normalized()
    {
        double len = Length();
        if (len < 1e-8f) return Zero;
        return this / len;
    }

    public static double Dot(in Vector3 v1, in Vector3 v2) => v1.X * v2.X + v1.Y * v2.Y + v1.Z * v2.Z;

    public static Vector3 Cross(in Vector3 v1, in Vector3 v2)
    {
        return new Vector3(v1.Y * v2.Z - v1.Z * v2.Y,
                            v1.Z * v2.X - v1.X * v2.Z,
                            v1.X * v2.Y - v1.Y * v2.X);
    }

    public static Vector3 operator +(Vector3 v1, Vector3 v2) => new(v1.X + v2.X, v1.Y + v2.Y, v1.Z + v2.Z);
    public static Vector3 operator -(Vector3 v1, Vector3 v2) => new(v1.X - v2.X, v1.Y - v2.Y, v1.Z - v2.Z);
    public static Vector3 operator -(Vector3 v) => new(-v.X, -v.Y, -v.Z);
    public static Vector3 operator *(Vector3 v, double s) => new(v.X * s, v.Y * s, v.Z * s);
    public static Vector3 operator *(double s, Vector3 v) => v * s;
    public static Vector3 operator /(Vector3 v, double s) => new(v.X / s, v.Y / s, v.Z / s);

    public override string ToString() => $"({X:0.###}, {Y:0.###}, {Z:0.###})";

}
