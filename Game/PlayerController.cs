using OpenTK.Windowing.GraphicsLibraryFramework;
using Physics.Dynamics;
using Physics.Math;
using OT = OpenTK.Mathematics;

namespace Game;

public sealed class PlayerController
{
    public RigidBody Body { get; }

    public double MoveSpeed = 6.0;
    public double JumpSpeed = 6.0;

    public PlayerController(RigidBody body) => Body = body;

    public void Update(KeyboardState kb, bool grounded, OT.Vector3 camForward, OT.Vector3 camRight)
    {
        // Flatten for ground movement
        camForward.Y = 0;
        camRight.Y = 0;

        if (camForward.LengthSquared < 1e-10f) camForward = new OT.Vector3(0, 0, -1);
        else camForward = camForward.Normalized();

        if (camRight.LengthSquared < 1e-10f) camRight = new OT.Vector3(1, 0, 0);
        else camRight = camRight.Normalized();

        OT.Vector3 wish = OT.Vector3.Zero;
        if (kb.IsKeyDown(Keys.W)) wish += camForward;
        if (kb.IsKeyDown(Keys.S)) wish -= camForward;
        if (kb.IsKeyDown(Keys.D)) wish += camRight;
        if (kb.IsKeyDown(Keys.A)) wish -= camRight;

        if (wish.LengthSquared > 1e-10f) wish = wish.Normalized();

        // Keep vertical from physics, override horizontal
        var v = Body.LinearVelocity;
        v.X = wish.X * MoveSpeed;
        v.Z = wish.Z * MoveSpeed;

        if (grounded && kb.IsKeyPressed(Keys.Space))
            v.Y = JumpSpeed;

        Body.LinearVelocity = v;
    }
}
