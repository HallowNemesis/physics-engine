using Physics.Collision;
using Physics.Dynamics;
using Physics.Math;

namespace Game;

public sealed class PhysicsWorldAdapter : IPhysicsWorldDebug
{
    private readonly World _world;

    // RigidBody is a reference type, so Dictionary<RigidBody,...> is fine
    private readonly Dictionary<RigidBody, string> _models = new();
    private readonly Dictionary<RigidBody, Vector3> _modelScales = new();
    private readonly Dictionary<RigidBody, Vector3> _modelPivots = new();
    public PhysicsWorldAdapter(World world) => _world = world;

    public void AttachModel(RigidBody body, string modelPath, Vector3 scale, Vector3 pivots)
    {
        _models[body] = modelPath;
        _modelScales[body] = scale;
        _modelPivots[body] = pivots;
    }

    public void AttachModel(RigidBody body, string modelPath)
    => AttachModel(body, modelPath, new Vector3(1, 1, 1), Vector3.Zero);

    public void Step(double dt)
    {
        _world.Step(dt);

        // TEMP DEBUG: watch one dynamic body
        //if (_world.Bodies.Count > 1)
        //{
        //    var b = _world.Bodies[1];
        //    Console.WriteLine($"y={b.Position.Y:0.000} vy={b.LinearVelocity.Y:0.000}");
        //}
    }

    public IEnumerable<DebugBody> GetBodies()
    {
        foreach (var rb in _world.Bodies)
        {
            // 1) Render-model override
            if (_models.TryGetValue(rb, out var modelPath))
            {
                var scale = _modelScales.TryGetValue(rb, out var s) ? s : new Vector3(1, 1, 1);
                var pivots = _modelPivots.TryGetValue(rb, out var p) ? p : Vector3.Zero;

                yield return new DebugBody(
                    Shape: DebugShapeKind.Model,
                    Position: rb.Position,
                    Rotation: rb.Orientation,
                    Scale: scale,
                    Color: rb.IsStatic
                        ? new Vector3(0.25, 0.25, 0.28)
                        : new Vector3(0.2, 0.7, 1.0),
                    ModelKey: modelPath
                );
                continue;
            }

            // 2) Shapes
            switch (rb.Shape)
            {
                case SphereShape s:
                    {
                        float r = (float)s.Radius;
                        yield return new DebugBody(
                            Shape: DebugShapeKind.Sphere,
                            Position: rb.Position,
                            Rotation: rb.Orientation,
                            Scale: new Vector3(r * 2f, r * 2f, r * 2f),
                            Color: rb.IsStatic
                                ? new Vector3(0.25, 0.25, 0.28)
                                : new Vector3(0.2, 0.7, 1.0)
                        );
                        break;
                    }

                case PlaneShape p:
                    {
                        // NOTE: your renderer assumes UnitY plane. Keeping that assumption:
                        float y = (float)p.Offset;

                        yield return new DebugBody(
                            Shape: DebugShapeKind.Box,
                            Position: new Vector3(0, y - 0.5, 0),
                            Rotation: Quaternion.Identity,
                            Scale: new Vector3(40, 1, 40),
                            Color: new Vector3(0.25, 0.25, 0.28)
                        );
                        break;
                    }

                case BoxShape bx:
                    {
                        var he = bx.HalfExtents;
                        yield return new DebugBody(
                            Shape: DebugShapeKind.Box,
                            Position: rb.Position,
                            Rotation: rb.Orientation,
                            Scale: new Vector3(he.X * 2, he.Y * 2, he.Z * 2),
                            Color: rb.IsStatic
                                ? new Vector3(0.25, 0.25, 0.28)
                                : new Vector3(0.8, 0.3, 0.9)
                        );
                        break;
                    }

                default:
                    {
                        // Fallback
                        yield return new DebugBody(
                            Shape: DebugShapeKind.Box,
                            Position: rb.Position,
                            Rotation: rb.Orientation,
                            Scale: new Vector3(1, 1, 1),
                            Color: new Vector3(1.0, 0.2, 0.7)
                        );
                        break;
                    }
            }
        }
    }
}
