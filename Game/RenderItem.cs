using System;
using System.Collections.Generic;
using System.Text;
using Assets;
using Game;

namespace Graphics;
public readonly record struct RenderItem(
    DebugShapeKind Kind,
    System.Numerics.Vector3 Position,
    System.Numerics.Quaternion Rotation,
    System.Numerics.Vector3 Scale,
    System.Numerics.Vector3 Color,
    string? ModelKey = null,
    System.Numerics.Vector3 ModelPivot = default
);
