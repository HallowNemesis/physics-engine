using Physics.Math;

namespace Physics.Dynamics;

public static class JointMath
{
    // Rotate vector by quaternion (assumes q normalized)
    public static Vector3 Rotate(in Quaternion q, in Vector3 v)
    {
        // t = 2 * cross(q.xyz, v)
        // v' = v + q.w * t + cross(q.xyz, t)
        var qv = new Vector3(q.X, q.Y, q.Z);
        Vector3 t = 2.0 * Vector3.Cross(qv, v);
        return v + q.W * t + Vector3.Cross(qv, t);
    }

    public static Quaternion Conjugate(in Quaternion q) => new(-q.X, -q.Y, -q.Z, q.W);

    public static Vector3 InverseRotate(in Quaternion q, in Vector3 v)
    {
        // inverse rotation = rotate by conjugate for unit quaternion
        var qc = Conjugate(q);
        return Rotate(qc, v);
    }

    // Small-angle orientation error vector from A to B in world-ish form.
    // qErr = qB * conjugate(qA), then error ~ 2*qErr.xyz (if w ~ 1)
    public static Vector3 OrientationError(in Quaternion qA, in Quaternion qB)
    {
        Quaternion qErr = Mul(qB, Conjugate(qA));

        // Keep shortest path
        if (qErr.W < 0)
            qErr = new Quaternion(-qErr.X, -qErr.Y, -qErr.Z, -qErr.W);

        return new Vector3(qErr.X, qErr.Y, qErr.Z) * 2.0;
    }

    public static Quaternion Mul(in Quaternion a, in Quaternion b)
    {
        return new Quaternion(
            a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
            a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X,
            a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W,
            a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z
        );
    }

    public static void BuildOrthonormalBasis(in Vector3 n, out Vector3 t1, out Vector3 t2)
    {
        // n assumed normalized
        Vector3 a = (System.Math.Abs(n.Y) < 0.9) ? Vector3.UnitY : new Vector3(1, 0, 0);
        t1 = Vector3.Cross(a, n).Normalized();
        t2 = Vector3.Cross(n, t1); // already normalized if n,t1 normalized
    }

    // --- 1D constraint row solvers (linear + angular) ---

    public static void SolveLinearRow(
        RigidBody A, RigidBody B,
        in Vector3 rA, in Vector3 rB,
        in Vector3 axis,
        double bias,
        double softness,
        ref double accumulatedImpulse)
    {
        // Effective mass along axis
        double k = 0.0;

        if (!A.IsStatic)
        {
            Vector3 rn = Vector3.Cross(rA, axis);
            Vector3 invI_rn = A.InvInertiaWorld * rn;
            Vector3 term = Vector3.Cross(invI_rn, rA);
            k += A.InvMass + Vector3.Dot(term, axis);
        }

        if (!B.IsStatic)
        {
            Vector3 rn = Vector3.Cross(rB, axis);
            Vector3 invI_rn = B.InvInertiaWorld * rn;
            Vector3 term = Vector3.Cross(invI_rn, rB);
            k += B.InvMass + Vector3.Dot(term, axis);
        }

        if (softness > 0.0) k += softness;
        if (k <= 1e-12) return;

        double effMass = 1.0 / k;

        // Relative velocity at anchors
        Vector3 vA = A.LinearVelocity + Vector3.Cross(A.AngularVelocity, rA);
        Vector3 vB = B.LinearVelocity + Vector3.Cross(B.AngularVelocity, rB);
        Vector3 rv = vB - vA;

        double Cdot = Vector3.Dot(rv, axis);

        double lambda = -(Cdot + bias + softness * accumulatedImpulse) * effMass;

        accumulatedImpulse += lambda;

        Vector3 P = axis * lambda;

        ApplyImpulse(A, -P, rA);
        ApplyImpulse(B, P, rB);
    }

    public static void SolveAngularRow(
        RigidBody A, RigidBody B,
        in Vector3 axis,
        double bias,
        double softness,
        ref double accumulatedImpulse)
    {
        // Effective mass for angular constraint: axis·(invI*axis)
        double k = 0.0;

        if (!A.IsStatic)
            k += Vector3.Dot(A.InvInertiaWorld * axis, axis);

        if (!B.IsStatic)
            k += Vector3.Dot(B.InvInertiaWorld * axis, axis);

        if (softness > 0.0) k += softness;
        if (k <= 1e-12) return;

        double effMass = 1.0 / k;

        Vector3 rvw = B.AngularVelocity - A.AngularVelocity;
        double Cdot = Vector3.Dot(rvw, axis);

        double lambda = -(Cdot + bias + softness * accumulatedImpulse) * effMass;

        accumulatedImpulse += lambda;

        ApplyAngularImpulse(A, -axis * lambda);
        ApplyAngularImpulse(B, axis * lambda);
    }

    private static void ApplyImpulse(RigidBody b, in Vector3 impulse, in Vector3 r)
    {
        if (b.IsStatic) return;
        if (!b.IsAwake) b.Wake();

        b.LinearVelocity += impulse * b.InvMass;
        b.AngularVelocity += b.InvInertiaWorld * Vector3.Cross(r, impulse);
    }

    private static void ApplyAngularImpulse(RigidBody b, in Vector3 angularImpulse)
    {
        if (b.IsStatic) return;
        if (!b.IsAwake) b.Wake();

        b.AngularVelocity += b.InvInertiaWorld * angularImpulse;
    }
}
