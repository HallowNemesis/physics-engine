using Physics.Collision;
using Physics.Math;

namespace Physics.Dynamics;

public sealed class RigidBody
{
    private static int _nextId = 1;
    public readonly int Id = _nextId++;

    public Vector3 Position;
    public Quaternion Orientation = Quaternion.Identity;

    public Vector3 LinearVelocity;
    public Vector3 AngularVelocity;

    public Vector3 Force;
    public Vector3 Torque;

    public double InvMass; // 0 => static
    public Matrix3 InvInertiaLocal; // body-space
    public Matrix3 InvInertiaWorld; // world-space (updated)

    public double Restitution = 0.2d;
    public double Friction = 0.6d;

    public IShape Shape;

    public bool IsStatic => InvMass == 0d;

    // ---- Sleeping ----
    public bool IsAwake = true;
    public double SleepTimer = 0.0;

    public void Wake()
    {
        IsAwake = true;
        SleepTimer = 0.0;
    }

    public RigidBody(IShape shape, double mass, Vector3 position)
    {
        Shape = shape;
        Position = position;

        if (mass <= 0d)
        {
            InvMass = 0d;
            InvInertiaLocal = Matrix3.Zero;
            InvInertiaWorld = Matrix3.Zero;
        }
        else
        {
            InvMass = 1d / mass;
            InvInertiaLocal = shape.ComputeInvInertiaLocal(mass);
            InvInertiaWorld = InvInertiaLocal;
        }
    }

    public void AddForce(Vector3 force)
    {
        Force += force;
        if (!IsStatic) Wake();
    }

    public void AddTorque(Vector3 torque)
    {
        Torque += torque;
        if (!IsStatic) Wake();
    }

    public void AddForceAtPoint(Vector3 force, Vector3 worldPoint)
    {
        Force += force;
        Vector3 r = worldPoint - Position;
        Torque += Vector3.Cross(r, force);
        if (!IsStatic) Wake();
    }

    public void ClearAccumulators()
    {
        Force = Vector3.Zero;
        Torque = Vector3.Zero;
    }

    public void UpdateInvInertiaWorld()
    {
        if (IsStatic)
        {
            InvInertiaWorld = Matrix3.Zero;
            return;
        }

        Matrix3 r = Matrix3.FromQuaternion(Orientation);
        InvInertiaWorld = r * InvInertiaLocal * r.Transposed();
    }
}
