using Physics.Math;

namespace Physics.Collision;

public sealed class BoxShape : IShape
{
    // Half extents in local/body space
    public Vector3 HalfExtents { get; }

    public BoxShape(Vector3 halfExtents)
    {
        HalfExtents = halfExtents;
    }

    public Matrix3 ComputeInvInertiaLocal(double mass)
    {
        if (mass <= 0d) return Matrix3.Zero;

        // Solid box inertia (about center), using half extents hx,hy,hz:
        // Ixx = (1/3) m (hy^2 + hz^2)
        // Iyy = (1/3) m (hx^2 + hz^2)
        // Izz = (1/3) m (hx^2 + hy^2)
        double hx = HalfExtents.X, hy = HalfExtents.Y, hz = HalfExtents.Z;

        double Ixx = (1.0 / 3.0) * mass * (hy * hy + hz * hz);
        double Iyy = (1.0 / 3.0) * mass * (hx * hx + hz * hz);
        double Izz = (1.0 / 3.0) * mass * (hx * hx + hy * hy);

        double invX = (Ixx < 1e-12) ? 0d : 1d / Ixx;
        double invY = (Iyy < 1e-12) ? 0d : 1d / Iyy;
        double invZ = (Izz < 1e-12) ? 0d : 1d / Izz;

        return new Matrix3(
            invX, 0d, 0d,
            0d, invY, 0d,
            0d, 0d, invZ
        );
    }
}
