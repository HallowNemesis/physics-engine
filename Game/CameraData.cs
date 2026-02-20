namespace Graphics;

public readonly record struct CameraData(
    System.Numerics.Vector3 Position,
    System.Numerics.Vector3 Forward,
    float Aspect,
    float FovDeg,
    float Near,
    float Far
);
