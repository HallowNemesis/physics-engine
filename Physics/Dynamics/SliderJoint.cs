using Physics.Math;

namespace Physics.Dynamics;

public sealed class SliderJoint : IConstraint
{
    public readonly RigidBody A;
    public readonly RigidBody B;

    public Vector3 LocalAnchorA;
    public Vector3 LocalAnchorB;

    // Slider axis in local A (and optionally B). We'll use A as reference.
    public Vector3 LocalAxisA = new Vector3(1, 0, 0);

    public double Beta = 0.2;
    public double Softness = 0.0;

    // Warm start accumulators
    private double _pU, _pV;      // 2 linear constraints (perp to axis)
    private double _wX, _wY, _wZ; // 3 angular constraints (lock rotation)

    // Cached
    private Vector3 _rA, _rB;
    private Vector3 _axis; // world
    private Vector3 _u, _v; // perp directions
    private double _bU, _bV;

    private Vector3 _angErr;
    private Vector3 _wx, _wy, _wz; // angular axes
    private double _bWX, _bWY, _bWZ;

    public SliderJoint(RigidBody a, RigidBody b, Vector3 localAnchorA, Vector3 localAnchorB, Vector3 localAxisA)
    {
        A = a; B = b;
        LocalAnchorA = localAnchorA;
        LocalAnchorB = localAnchorB;
        LocalAxisA = localAxisA.Normalized();
    }

    public void PreStep(double dt)
    {
        Vector3 pA = A.Position + JointMath.Rotate(A.Orientation, LocalAnchorA);
        Vector3 pB = B.Position + JointMath.Rotate(B.Orientation, LocalAnchorB);

        _rA = pA - A.Position;
        _rB = pB - B.Position;

        // Slider axis from A
        _axis = JointMath.Rotate(A.Orientation, LocalAxisA).Normalized();

        // Build perpendicular basis (u,v) so we constrain motion along u and v only
        JointMath.BuildOrthonormalBasis(_axis, out _u, out _v);

        Vector3 errP = pB - pA;

        // Remove axis component; constrain the perpendicular error only
        double errU = Vector3.Dot(errP, _u);
        double errV = Vector3.Dot(errP, _v);

        if (dt > 1e-9)
        {
            _bU = (Beta / dt) * errU;
            _bV = (Beta / dt) * errV;
        }
        else _bU = _bV = 0.0;

        // Lock relative rotation completely (3 angular constraints)
        _angErr = JointMath.OrientationError(A.Orientation, B.Orientation);

        // Use world axes for angular locking (stable)
        _wx = new Vector3(1, 0, 0);
        _wy = new Vector3(0, 1, 0);
        _wz = new Vector3(0, 0, 1);

        if (dt > 1e-9)
        {
            _bWX = (Beta / dt) * Vector3.Dot(_angErr, _wx);
            _bWY = (Beta / dt) * Vector3.Dot(_angErr, _wy);
            _bWZ = (Beta / dt) * Vector3.Dot(_angErr, _wz);
        }
        else _bWX = _bWY = _bWZ = 0.0;
    }

    public void WarmStart()
    {
        // 2 linear constraints (perp to axis)
        JointMath.SolveLinearRow(A, B, _rA, _rB, _u, 0.0, 0.0, ref _pU);
        JointMath.SolveLinearRow(A, B, _rA, _rB, _v, 0.0, 0.0, ref _pV);

        // lock rotation (3)
        JointMath.SolveAngularRow(A, B, _wx, 0.0, 0.0, ref _wX);
        JointMath.SolveAngularRow(A, B, _wy, 0.0, 0.0, ref _wY);
        JointMath.SolveAngularRow(A, B, _wz, 0.0, 0.0, ref _wZ);
    }

    public void Solve()
    {
        // Constrain perpendicular translation => allows only sliding along axis
        JointMath.SolveLinearRow(A, B, _rA, _rB, _u, _bU, Softness, ref _pU);
        JointMath.SolveLinearRow(A, B, _rA, _rB, _v, _bV, Softness, ref _pV);

        // Lock all relative rotation
        JointMath.SolveAngularRow(A, B, _wx, _bWX, Softness, ref _wX);
        JointMath.SolveAngularRow(A, B, _wy, _bWY, Softness, ref _wY);
        JointMath.SolveAngularRow(A, B, _wz, _bWZ, Softness, ref _wZ);
    }
}
