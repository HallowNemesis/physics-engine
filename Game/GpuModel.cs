using Graphics;

namespace Assets;

public sealed class GpuModel : IDisposable
{
    public required Mesh Mesh;
    public required SubMesh[] SubMeshes;
    public required GpuMaterial[] Materials;

    public void Dispose()
    {
        foreach (var m in Materials) m.Dispose();
        Mesh.Dispose();
    }
}

public sealed class GpuMaterial : IDisposable
{
    public Texture2D? BaseColor;
    public Texture2D? MetallicRoughness;
    public Texture2D? Normal;

    public void Dispose()
    {
        BaseColor?.Dispose();
        MetallicRoughness?.Dispose();
        Normal?.Dispose();
    }
}
