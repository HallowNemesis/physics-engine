
using Physics.Math;

namespace Physics.Collision;

public sealed class SphereShape : IShape
{
    public double Radius { get; }

    public SphereShape(double radius)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(radius, 0d);
        Radius = radius;
    }

    public Matrix3 ComputeInvInertiaLocal(double mass)
    {
        if (mass < 0) return Matrix3.Zero; //static/infinite mass
        //solid sphere inertia: I = (2/5) m r^2
        double I = 0.4d * mass * Radius * Radius;
        double invI = (I < 1e-12f) ? 0d : 1d / I;
        return Matrix3.Identity * invI;
    }
}
