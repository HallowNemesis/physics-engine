using Physics.Math;

namespace Physics.Collision;

// Plane: Dot(N, X) - Offset = 0
// N must be normalized. Offset is distance along N from origin.
public sealed class PlaneShape : IShape
{
    public Vector3 Normal { get; }
    public double Offset { get; }

    public PlaneShape(Vector3 normal, double offset)
    {
        var n = normal.Normalized();
        // If normal is near-zero, default to +Y to avoid NaNs
        if (n.LengthSquared() < 1e-12) n = Vector3.UnitY;

        Normal = n;
        Offset = offset;
    }

    public Matrix3 ComputeInvInertiaLocal(double mass)
    {
        // Planes are treated as static/infinite inertia in practice.
        // If someone tries to create a dynamic plane, we still return zero.
        return Matrix3.Zero;
    }
}
