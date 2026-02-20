using System;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.GraphicsLibraryFramework;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using Graphics;

namespace Game;

public sealed class PhysicsViewer : GameWindow
{
    private Renderer3D _renderer = null!;
    private Camera _camera = null!;

    private double _accum;

    // More stable for contacts than 120
    private const float FixedDt = 1.0f / 120.0f;

    // Avoid hitch explosions
    private const double MaxFrameDt = 1.0 / 30.0; // 33ms cap
    private const int MaxSubSteps = 4;

    private readonly IPhysicsWorldDebug _world;
    private PhysicsWorldAdapter _adapter = null!;
    private Physics.Dynamics.World _rawWorld = null!;
    private Physics.Dynamics.RigidBody _playerBody = null!;
    private PlayerController _player = null!;

    private bool _thirdPerson = false;

    // tune these
    private float _eyeHeight = 0.85f;      // first-person: camera height above player center
    private float _thirdDist = 3.5f;       // third-person: distance behind player
    private float _thirdHeight = 1.2f;     // third-person: height above player center
                                           // camera collision tuning
    private float _camRadius = 0.25f;     // "camera sphere"
    private float _camSkin = 0.05f;       // extra push off wall to prevent buzzing
    private readonly List<int> _camCandidates = new(512);

    public PhysicsViewer(GameWindowSettings gws, NativeWindowSettings nws) : base(gws, nws)
    {
        VSync = VSyncMode.On;

        var (world, player, bindModels) = Scenes.CreateSphereDropScene();
        _rawWorld = world;
        _playerBody = player;

        _adapter = new PhysicsWorldAdapter(world);
        bindModels(_adapter);

        _world = _adapter;
    }

    protected override void OnLoad()
    {
        base.OnLoad();

        GL.Enable(EnableCap.DepthTest);
        GL.Enable(EnableCap.CullFace);
        GL.CullFace(TriangleFace.Back);

        _renderer = new Renderer3D();
        _renderer.Load();

        _camera = new Camera(
            position: new Vector3(0, 3, 8),
            target: Vector3.Zero,
            up: Vector3.UnitY);
        _player = new PlayerController(_playerBody);

        CursorState = CursorState.Grabbed;
    }

    protected override void OnResize(ResizeEventArgs e)
    {
        base.OnResize(e);
        GL.Viewport(0, 0, Size.X, Size.Y);
        _camera.Aspect = Size.X / (float)Size.Y;
    }

    protected override void OnUpdateFrame(FrameEventArgs e)
    {
        base.OnUpdateFrame(e);
        if (!IsFocused) return;

        var kb = KeyboardState;
        var ms = MouseState;

        if (kb.IsKeyDown(Keys.Escape)) Close();

        // 1) Mouse look
        _camera.UpdateMouseLook(ms);

        // 2) Player movement (uses grounded from last step)
        bool grounded = _rawWorld.IsGrounded(_playerBody);
        _player.Update(kb, grounded, _camera.Forward, _camera.Right);
        if (kb.IsKeyPressed(Keys.F5))
            _thirdPerson = !_thirdPerson;

        // 3) Physics fixed step
        double frameDt = e.Time;
        if (frameDt > MaxFrameDt) frameDt = MaxFrameDt;

        _accum += frameDt;
        int steps = 0;

        while (_accum >= FixedDt && steps < MaxSubSteps)
        {
            _world.Step(FixedDt);
            _accum -= FixedDt;
            steps++;
        }

        if (steps == MaxSubSteps) _accum = 0;

        var playerPos = ToGL(_playerBody.Position);

        if (!_thirdPerson)
        {
            // First-person: eyes + tiny forward nudge
            var eye = new Vector3(0, _eyeHeight, 0);
            var forwardNudge = _camera.Forward * 0.15f;
            _camera.Position = playerPos + eye + forwardNudge;

            // (optional) if you implemented OverrideTarget, clear it:
            // _camera.OverrideTarget = null;
        }
        else
        {
            var target = playerPos + new Vector3(0, _thirdHeight, 0);
            var desired = target - _camera.Forward * _thirdDist;

            // push camera forward if it would collide with static meshes
            var resolved = ResolveThirdPersonCamera(target, desired, _camRadius);

            _camera.Position = resolved;

            // (optional) if you implemented OverrideTarget:
            // _camera.OverrideTarget = target;
        }


        if (kb.IsKeyPressed(Keys.F1))
            _renderer.Wireframe = !_renderer.Wireframe;
    }

    protected override void OnRenderFrame(FrameEventArgs e)
    {
        base.OnRenderFrame(e);

        GL.ClearColor(0.07f, 0.08f, 0.10f, 1f);
        GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

        _renderer.BeginFrame(_camera);

        _renderer.DrawGrid(size: 20, spacing: 1);

        foreach (var body in _world.GetBodies())
            _renderer.DrawBody(body);

        _renderer.EndFrame();

        SwapBuffers();
    }

    protected override void OnUnload()
    {
        base.OnUnload();
        _renderer.Dispose();
    }

    private Vector3 ResolveThirdPersonCamera(Vector3 targetWorld, Vector3 desiredWorld, float radius)
    {
        var dir = desiredWorld - targetWorld;
        float len = dir.Length;
        if (len < 1e-6f) return desiredWorld;

        var dirN = dir / len;

        bool hit = false;
        float bestT = 1.0f; // along segment [0..1]

        // Check all static meshes in the world
        foreach (var rb in _rawWorld.Bodies)
        {
            if (rb.Shape is not Physics.Collision.StaticMeshShape mesh) continue;

            // Transform endpoints into mesh local (your mesh currently assumes translation-only)
            var p0 = targetWorld - ToGL(rb.Position);
            var p1 = desiredWorld - ToGL(rb.Position);

            // Segment AABB expanded by radius (BVH broadphase)
            var segMin = Vector3.ComponentMin(p0, p1) - new Vector3(radius);
            var segMax = Vector3.ComponentMax(p0, p1) + new Vector3(radius);

            var q = new Physics.Collision.Aabb(
                new Physics.Math.Vector3(segMin.X, segMin.Y, segMin.Z),
                new Physics.Math.Vector3(segMax.X, segMax.Y, segMax.Z)
            );

            _camCandidates.Clear();
            mesh.Query(q, _camCandidates);

            // Narrowphase: segment vs candidate triangles
            for (int i = 0; i < _camCandidates.Count; i++)
            {
                var tri = mesh.Tris[_camCandidates[i]];

                var a = new Vector3((float)tri.A.X, (float)tri.A.Y, (float)tri.A.Z);
                var b = new Vector3((float)tri.B.X, (float)tri.B.Y, (float)tri.B.Z);
                var c = new Vector3((float)tri.C.X, (float)tri.C.Y, (float)tri.C.Z);

                if (SegmentTriangle(p0, p1, a, b, c, out float t))
                {
                    if (t < bestT)
                    {
                        bestT = t;
                        hit = true;
                    }
                }
            }
        }

        if (!hit) return desiredWorld;

        // Place camera at hit point, pulled forward along the viewing ray to avoid clipping
        float pull = radius + _camSkin;

        // Convert t to distance along segment:
        float hitDist = bestT * len;
        float newDist = hitDist - pull;

        // don’t let camera go in front of target
        if (newDist < 0.05f) newDist = 0.05f;

        return targetWorld + dirN * newDist;
    }

    private static bool SegmentTriangle(
        Vector3 p0, Vector3 p1,
        Vector3 a, Vector3 b, Vector3 c,
        out float t)
    {
        // Möller–Trumbore ray-triangle; clamp to segment length.
        // Ray: p0 + d * u, u in [0,1] where d = p1-p0
        t = 0;

        var d = p1 - p0;
        var e1 = b - a;
        var e2 = c - a;

        var pvec = Vector3.Cross(d, e2);
        float det = Vector3.Dot(e1, pvec);

        // If near-parallel, no hit
        if (MathF.Abs(det) < 1e-8f) return false;

        float invDet = 1.0f / det;

        var tvec = p0 - a;
        float u = Vector3.Dot(tvec, pvec) * invDet;
        if (u < 0 || u > 1) return false;

        var qvec = Vector3.Cross(tvec, e1);
        float v = Vector3.Dot(d, qvec) * invDet;
        if (v < 0 || (u + v) > 1) return false;

        float tt = Vector3.Dot(e2, qvec) * invDet;

        // Convert from "distance along d" to segment parameter [0..1]
        // Since d is the full segment vector, tt in [0..1] means within the segment.
        if (tt < 0 || tt > 1) return false;

        t = tt;
        return true;
    }

    private static Vector3 ToGL(Physics.Math.Vector3 v)
        => new((float)v.X, (float)v.Y, (float)v.Z);

}
