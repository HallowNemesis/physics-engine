using Physics.Math;

namespace Physics.Dynamics;

public sealed class DistanceJoint : IConstraint
{
    public readonly RigidBody A;
    public readonly RigidBody B;

    // Anchors in each body's LOCAL space
    public Vector3 LocalAnchorA;
    public Vector3 LocalAnchorB;

    public double TargetLength;

    // Tuning
    public double Beta = 0.2;      // Baumgarte (position error to velocity bias)
    public double Softness = 0.0;  // Gamma (0 = hard constraint)

    // Warm starting accumulator
    public double AccumulatedImpulse;

    // Cached per-step values
    private Vector3 _rA, _rB;
    private Vector3 _n;      // constraint axis (world)
    private double _effMass; // scalar effective mass
    private double _bias;    // velocity bias

    public DistanceJoint(RigidBody a, RigidBody b, Vector3 localAnchorA, Vector3 localAnchorB, double targetLength)
    {
        A = a;
        B = b;
        LocalAnchorA = localAnchorA;
        LocalAnchorB = localAnchorB;
        TargetLength = targetLength;
    }

    public void PreStep(double dt)
    {
        // World anchors
        Vector3 pA = A.Position + Rotate(A.Orientation, LocalAnchorA);
        Vector3 pB = B.Position + Rotate(B.Orientation, LocalAnchorB);

        _rA = pA - A.Position;
        _rB = pB - B.Position;

        Vector3 d = pB - pA;
        double len2 = d.LengthSquared();

        if (len2 < 1e-18)
        {
            _n = Vector3.UnitY; // fallback axis
            _bias = 0.0;
            _effMass = 0.0;
            return;
        }

        double len = System.Math.Sqrt(len2);
        _n = d / len;

        // Position error C = current - target
        double C = len - TargetLength;

        // Bias (Baumgarte stabilization)
        _bias = (dt > 1e-9) ? (Beta * C / dt) : 0.0;

        // Effective mass (scalar) along axis n
        double k = 0.0;

        if (!A.IsStatic)
        {
            // invMass + n · ( (invI * (r×n)) × r )
            Vector3 rn = Vector3.Cross(_rA, _n);
            Vector3 invI_rn = A.InvInertiaWorld * rn;
            Vector3 term = Vector3.Cross(invI_rn, _rA);
            k += A.InvMass + Vector3.Dot(term, _n);
        }

        if (!B.IsStatic)
        {
            Vector3 rn = Vector3.Cross(_rB, _n);
            Vector3 invI_rn = B.InvInertiaWorld * rn;
            Vector3 term = Vector3.Cross(invI_rn, _rB);
            k += B.InvMass + Vector3.Dot(term, _n);
        }

        // Soft constraint (optional)
        if (Softness > 0.0)
            k += Softness;

        _effMass = (k > 1e-12) ? (1.0 / k) : 0.0;

        // If you change anchors/length at runtime, consider zeroing AccumulatedImpulse.
    }

    public void WarmStart()
    {
        if (_effMass <= 0.0) return;
        if (AccumulatedImpulse == 0.0) return;

        Vector3 P = _n * AccumulatedImpulse;

        ApplyImpulse(A, -P, _rA);
        ApplyImpulse(B, P, _rB);
    }

    public void Solve()
    {
        if (_effMass <= 0.0) return;

        Vector3 vA = A.LinearVelocity + Vector3.Cross(A.AngularVelocity, _rA);
        Vector3 vB = B.LinearVelocity + Vector3.Cross(B.AngularVelocity, _rB);
        Vector3 rv = vB - vA;

        double Cdot = Vector3.Dot(rv, _n);

        // Solve: lambda = - (Cdot + bias + softness*lambda) * effMass
        double lambda = -(Cdot + _bias + Softness * AccumulatedImpulse) * _effMass;

        AccumulatedImpulse += lambda;

        Vector3 P = _n * lambda;

        ApplyImpulse(A, -P, _rA);
        ApplyImpulse(B, P, _rB);
    }

    // --- Helpers (all your math) ---

    private static void ApplyImpulse(RigidBody b, Vector3 impulse, Vector3 r)
    {
        if (b.IsStatic) return;

        if (!b.IsAwake) b.Wake();

        b.LinearVelocity += impulse * b.InvMass;
        b.AngularVelocity += b.InvInertiaWorld * Vector3.Cross(r, impulse);
    }

    // Rotate vector by quaternion (assumes q is normalized)
    // v' = q * (v,0) * q^-1
    private static Vector3 Rotate(Quaternion q, Vector3 v)
    {
        // Using quaternion-vector rotation formula:
        // t = 2 * cross(q.xyz, v)
        // v' = v + q.w * t + cross(q.xyz, t)
        var qv = new Vector3(q.X, q.Y, q.Z);
        Vector3 t = 2.0 * Vector3.Cross(qv, v);
        return v + q.W * t + Vector3.Cross(qv, t);
    }
}
