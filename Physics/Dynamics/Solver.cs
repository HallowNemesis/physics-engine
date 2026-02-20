using Physics.Collision;
using Physics.Math;

namespace Physics.Dynamics;

public static class Solver
{
    private const double RestitutionVelocityThreshold = 1.0;

    public static void Solve(List<Manifold> contacts, List<IConstraint> constraints, int iterations, double dt)
    {
        // Warm start contacts
        foreach (var c in contacts)
            WarmStartContact(c);

        // Warm start joints
        foreach (var con in constraints)
            con.WarmStart();

        // Sequential impulses over both sets
        for (int it = 0; it < iterations; it++)
        {
            foreach (var c in contacts)
                SolveContactVelocity(c, dt);

            foreach (var con in constraints)
                con.Solve();
        }
    }

    private static void WarmStartContact(Manifold c)
    {
        var A = c.A;
        var B = c.B;

        Vector3 n = c.Normal;
        Vector3 rA = c.ContactPoint - A.Position;
        Vector3 rB = c.ContactPoint - B.Position;

        double jn = c.AccumulatedNormalImpulse;
        if (jn != 0.0)
        {
            ApplyImpulse(A, -n * jn, rA);
            ApplyImpulse(B, n * jn, rB);
        }

        Vector3 vA = A.LinearVelocity + Vector3.Cross(A.AngularVelocity, rA);
        Vector3 vB = B.LinearVelocity + Vector3.Cross(B.AngularVelocity, rB);
        Vector3 rv = vB - vA;

        Vector3 t = rv - n * Vector3.Dot(rv, n);
        double tLen2 = t.LengthSquared();
        if (tLen2 <= 1e-12) return;

        t = t / System.Math.Sqrt(tLen2);

        double jt = c.AccumulatedTangentImpulse;
        if (jt != 0.0)
        {
            ApplyImpulse(A, -t * jt, rA);
            ApplyImpulse(B, t * jt, rB);
        }
    }

    private static void SolveContactVelocity(Manifold c, double dt)
    {
        var A = c.A;
        var B = c.B;

        Vector3 n = c.Normal;
        Vector3 rA = c.ContactPoint - A.Position;
        Vector3 rB = c.ContactPoint - B.Position;

        Vector3 vA = A.LinearVelocity + Vector3.Cross(A.AngularVelocity, rA);
        Vector3 vB = B.LinearVelocity + Vector3.Cross(B.AngularVelocity, rB);

        Vector3 rv = vB - vA;
        double vn = Vector3.Dot(rv, n);
        if (vn > 0.0) return;

        double denomN = EffectiveMassAlong(A, rA, n) + EffectiveMassAlong(B, rB, n);
        if (denomN <= 1e-12) return;

        double e = c.Restitution;
        if (-vn < RestitutionVelocityThreshold) e = 0.0;

        const double baumgarte = 0.15;
        const double slop = 0.01;

        double depth = System.Math.Max(c.Penetration - slop, 0.0);
        double bias = (depth > 0.0) ? (baumgarte * depth / System.Math.Max(dt, 1e-6)) : 0.0;

        double jn = (-(1.0 + e) * vn + bias) / denomN;

        double oldJn = c.AccumulatedNormalImpulse;
        c.AccumulatedNormalImpulse = System.Math.Max(oldJn + jn, 0.0);
        double dJn = c.AccumulatedNormalImpulse - oldJn;

        ApplyImpulse(A, -n * dJn, rA);
        ApplyImpulse(B, n * dJn, rB);

        // Friction
        vA = A.LinearVelocity + Vector3.Cross(A.AngularVelocity, rA);
        vB = B.LinearVelocity + Vector3.Cross(B.AngularVelocity, rB);
        rv = vB - vA;

        Vector3 t = rv - n * Vector3.Dot(rv, n);
        double tLen2 = t.LengthSquared();
        if (tLen2 <= 1e-12) return;

        t = t / System.Math.Sqrt(tLen2);

        double vt = Vector3.Dot(rv, t);

        double denomT = EffectiveMassAlong(A, rA, t) + EffectiveMassAlong(B, rB, t);
        if (denomT <= 1e-12) return;

        double jt = -vt / denomT;

        double maxF = c.Friction * c.AccumulatedNormalImpulse;

        double oldJt = c.AccumulatedTangentImpulse;
        c.AccumulatedTangentImpulse = Clamp(oldJt + jt, -maxF, maxF);
        double dJt = c.AccumulatedTangentImpulse - oldJt;

        ApplyImpulse(A, -t * dJt, rA);
        ApplyImpulse(B, t * dJt, rB);
    }

    private static double EffectiveMassAlong(RigidBody b, Vector3 r, Vector3 dir)
    {
        double invMass = b.InvMass;
        if (invMass == 0.0) return 0.0;

        Vector3 rxDir = Vector3.Cross(r, dir);
        Vector3 invI_rxDir = b.InvInertiaWorld * rxDir;
        Vector3 crossTerm = Vector3.Cross(invI_rxDir, r);

        return invMass + Vector3.Dot(crossTerm, dir);
    }

    private static void ApplyImpulse(RigidBody b, Vector3 impulse, Vector3 r)
    {
        if (b.IsStatic) return;

        if (!b.IsAwake) b.Wake();

        b.LinearVelocity += impulse * b.InvMass;
        b.AngularVelocity += b.InvInertiaWorld * Vector3.Cross(r, impulse);
    }

    public static void PositionalCorrection(List<Manifold> contacts)
    {
        const double slop = 0.01;
        const double percent = 0.10;
        const double maxCorrection = 0.20;

        foreach (var c in contacts)
        {
            var A = c.A;
            var B = c.B;

            double invMassSum = A.InvMass + B.InvMass;
            if (invMassSum <= 1e-12) continue;

            double depth = System.Math.Max(c.Penetration - slop, 0.0);
            if (depth <= 0.0) continue;

            double corrMag = percent * depth / invMassSum;
            if (corrMag > maxCorrection) corrMag = maxCorrection;

            Vector3 correction = c.Normal * corrMag;

            if (!A.IsStatic) A.Position -= correction * A.InvMass;
            if (!B.IsStatic) B.Position += correction * B.InvMass;
        }
    }

    private static double Clamp(double x, double lo, double hi)
        => x < lo ? lo : (x > hi ? hi : x);
}
