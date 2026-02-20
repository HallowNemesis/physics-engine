using Physics.Math;

namespace Physics.Dynamics;

public sealed class BallSocketJoint : IConstraint
{
    public readonly RigidBody A;
    public readonly RigidBody B;

    public Vector3 LocalAnchorA;
    public Vector3 LocalAnchorB;

    public double Beta = 0.2;      // Baumgarte
    public double Softness = 0.0;  // compliance

    // Warm-start impulses along 3 axes
    private double _impX, _impY, _impZ;

    // Cached
    private Vector3 _rA, _rB;
    private Vector3 _axX, _axY, _axZ;
    private double _biasX, _biasY, _biasZ;

    public BallSocketJoint(RigidBody a, RigidBody b, Vector3 localAnchorA, Vector3 localAnchorB)
    {
        A = a; B = b;
        LocalAnchorA = localAnchorA;
        LocalAnchorB = localAnchorB;
    }

    public void PreStep(double dt)
    {
        Vector3 pA = A.Position + JointMath.Rotate(A.Orientation, LocalAnchorA);
        Vector3 pB = B.Position + JointMath.Rotate(B.Orientation, LocalAnchorB);

        _rA = pA - A.Position;
        _rB = pB - B.Position;

        Vector3 error = pB - pA; // want 0

        // Use world axes (stable and fine for point joint)
        _axX = new Vector3(1, 0, 0);
        _axY = new Vector3(0, 1, 0);
        _axZ = new Vector3(0, 0, 1);

        if (dt > 1e-9)
        {
            _biasX = (Beta / dt) * Vector3.Dot(error, _axX);
            _biasY = (Beta / dt) * Vector3.Dot(error, _axY);
            _biasZ = (Beta / dt) * Vector3.Dot(error, _axZ);
        }
        else
        {
            _biasX = _biasY = _biasZ = 0.0;
        }
    }

    public void WarmStart()
    {
        JointMath.SolveLinearRow(A, B, _rA, _rB, _axX, 0.0, 0.0, ref _impX);
        JointMath.SolveLinearRow(A, B, _rA, _rB, _axY, 0.0, 0.0, ref _impY);
        JointMath.SolveLinearRow(A, B, _rA, _rB, _axZ, 0.0, 0.0, ref _impZ);
    }

    public void Solve()
    {
        JointMath.SolveLinearRow(A, B, _rA, _rB, _axX, _biasX, Softness, ref _impX);
        JointMath.SolveLinearRow(A, B, _rA, _rB, _axY, _biasY, Softness, ref _impY);
        JointMath.SolveLinearRow(A, B, _rA, _rB, _axZ, _biasZ, Softness, ref _impZ);
    }
}
