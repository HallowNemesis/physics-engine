using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using System.Collections.Concurrent;
using Assets;
using Game;

namespace Graphics;

public sealed class Renderer3D : IDisposable
{
    public bool Wireframe { get; set; }

    private Shader _shader = null!;
    private Mesh _cube = null!;
    private Mesh _sphere = null!;
    private Mesh _grid = null!;
    private readonly ConcurrentDictionary<string, Mesh> _modelCache = new();

    private Camera _camera = null!;

    public void Load()
    {
        _shader = new Shader(VertexSrc, FragmentSrc);

        _cube = MeshFactory.CreateCube();
        _sphere = MeshFactory.CreateIcoSphere(subdivisions: 2);

        _grid = MeshFactory.CreateGridLines(size: 20, spacing: 1);
    }

    public void BeginFrame(Camera camera)
    {
        _camera = camera;

        GL.PolygonMode(TriangleFace.FrontAndBack, Wireframe ? PolygonMode.Line : PolygonMode.Fill);

        _shader.Use();
        _shader.SetMat4("uView", _camera.ViewMatrix);
        _shader.SetMat4("uProj", _camera.ProjectionMatrix);
    }

    public void EndFrame()
    {
        GL.BindVertexArray(0);
        GL.UseProgram(0);
    }

    public void DrawGrid(int size, int spacing)
    {
        var model = Matrix4.Identity;
        _shader.SetMat4("uModel", model);
        _shader.SetVec3("uColor", new Vector3(0.22f, 0.23f, 0.26f));
        _grid.Draw(PrimitiveType.Lines);
    }

    public void DrawBody(in DebugBody body)
    {
        var pos = ToGL(body.Position);
        var rot = ToGL(body.Rotation);
        var scl = ToGL(body.Scale);
        var col = ToGL(body.Color);
        var pivot = ToGL(body.ModelPivot);

        var model =
            Matrix4.CreateTranslation(pivot) *
            Matrix4.CreateScale(scl) *
            Matrix4.CreateFromQuaternion(rot) *
            Matrix4.CreateTranslation(pos);

        _shader.SetMat4("uModel", model);
        _shader.SetVec3("uColor", col);

        switch (body.Shape)
        {
            case DebugShapeKind.Box:
                _cube.Draw(PrimitiveType.Triangles);
                return;

            case DebugShapeKind.Sphere:
                _sphere.Draw(PrimitiveType.Triangles);
                return;

            case DebugShapeKind.Model:
                {
                    if (string.IsNullOrWhiteSpace(body.ModelKey))
                        return;

                    var gpu = AssetCache.GetGpuModel(body.ModelKey);

                    for (int i = 0; i < gpu.SubMeshes.Length; i++)
                    {
                        var sm = gpu.SubMeshes[i];
                        var mat = gpu.Materials[sm.MaterialIndex];

                        if (mat.BaseColor != null)
                        {
                            mat.BaseColor.Bind(OpenTK.Graphics.OpenGL4.TextureUnit.Texture0);
                            _shader.SetInt("uBaseColorTex", 0);
                            _shader.SetBool("uHasBaseColor", true);
                        }
                        else _shader.SetBool("uHasBaseColor", false);

                        gpu.Mesh.DrawRange(OpenTK.Graphics.OpenGL4.PrimitiveType.Triangles, sm.IndexStart, sm.IndexCount);
                    }

                    return;
                }
        }
    }

    private static Vector3 ToGL(Physics.Math.Vector3 v)
        => new((float)v.X, (float)v.Y, (float)v.Z);

    private static Quaternion ToGL(Physics.Math.Quaternion q)
        => new((float)q.X, (float)q.Y, (float)q.Z, (float)q.W);

    public void Dispose()
    {
        foreach (var kv in _modelCache) kv.Value.Dispose();
        _modelCache.Clear();

        _grid?.Dispose();
        _cube?.Dispose();
        _sphere?.Dispose();
        _shader?.Dispose();
    }

    private const string VertexSrc = @"
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUV;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProj;

out vec3 vNormal;
out vec3 vWorldPos;
out vec2 vUV;

void main()
{
    vUV = aUV;

    vec4 world = uModel * vec4(aPos, 1.0);
    vWorldPos = world.xyz;

    vNormal = mat3(transpose(inverse(uModel))) * aNormal;

    gl_Position = uProj * uView * world;
}
";

    private const string FragmentSrc = @"
#version 330 core
out vec4 FragColor;

in vec3 vNormal;
in vec3 vWorldPos;
in vec2 vUV;

uniform vec3 uColor;
uniform bool uHasBaseColor;
uniform sampler2D uBaseColorTex;

void main()
{
    vec3 base = uColor;
    if (uHasBaseColor)
        base *= texture(uBaseColorTex, vUV).rgb;

    vec3 N = normalize(vNormal);
    vec3 L = normalize(vec3(0.6, 1.0, 0.4));
    float ndotl = max(dot(N, L), 0.0);

    vec3 ambient = base * 0.25;
    vec3 diffuse = base * (0.75 * ndotl);

    FragColor = vec4(ambient + diffuse, 1.0);
}
";

}
