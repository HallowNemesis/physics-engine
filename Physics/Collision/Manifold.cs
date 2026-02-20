using Physics.Dynamics;
using Physics.Math;

namespace Physics.Collision;

public sealed class Manifold
{
    public RigidBody A = null!;
    public RigidBody B = null!;

    public Vector3 Normal; // from A->B
    public double Penetration;
    public Vector3 ContactPoint;

    public double Restitution;
    public double Friction;
    
    public double AccumulatedNormalImpulse;
    public double AccumulatedTangentImpulse;

    public ulong ContactKey;
}
