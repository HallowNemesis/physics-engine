using OpenTK.Mathematics;
using OpenTK.Windowing.GraphicsLibraryFramework;

namespace Game;

public sealed class Camera
{
    public Vector3 Position;
    public float Pitch = -10f;
    public float Yaw = -90f;

    public float Aspect = 16f / 9f;
    public float FovDeg = 60f;
    public float Near = 0.05f;
    public float Far = 500f;

    private Vector2 _lastMouse;
    private bool _firstMouse = true;

    public Camera(Vector3 position, Vector3 target, Vector3 up)
    {
        Position = position;

        var dir = (target - position).Normalized();
        Pitch = MathHelper.RadiansToDegrees(MathF.Asin(dir.Y));
        Yaw = MathHelper.RadiansToDegrees(MathF.Atan2(dir.Z, dir.X)) - 90f;
    }

    public Matrix4 ViewMatrix => Matrix4.LookAt(Position, Position + Forward, Vector3.UnitY);
    public Matrix4 ProjectionMatrix => Matrix4.CreatePerspectiveFieldOfView(
        MathHelper.DegreesToRadians(FovDeg), Aspect, Near, Far);

    public Vector3 Forward
    {
        get
        {
            var pitch = MathHelper.DegreesToRadians(Pitch);
            var yaw = MathHelper.DegreesToRadians(Yaw);

            var f = new Vector3(
                MathF.Cos(pitch) * MathF.Cos(yaw),
                MathF.Sin(pitch),
                MathF.Cos(pitch) * MathF.Sin(yaw));

            return f.Normalized();
        }
    }

    public Vector3 Right => Vector3.Cross(Forward, Vector3.UnitY).Normalized();

    public void UpdateMouseLook(MouseState ms)
    {
        var mouse = ms.Position;

        if (_firstMouse)
        {
            _lastMouse = mouse;
            _firstMouse = false;
        }

        var delta = mouse - _lastMouse;
        _lastMouse = mouse;

        const float sensitivity = 0.12f;
        Yaw += delta.X * sensitivity;
        Pitch -= delta.Y * sensitivity;
        Pitch = MathHelper.Clamp(Pitch, -89f, 89f);
    }
}
