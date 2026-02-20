using Physics.Math;
namespace Game;

public enum DebugShapeKind
{
    Box,
    Sphere,
    Model
}

public readonly record struct DebugBody(
    DebugShapeKind Shape,
    Vector3 Position,
    Quaternion Rotation,
    Vector3 Scale,
    Vector3 Color,
    string? ModelKey = null,
    Vector3 ModelPivot = default
    );

public interface IPhysicsWorldDebug
{
    void Step(double dt);
    IEnumerable<DebugBody> GetBodies();
}

