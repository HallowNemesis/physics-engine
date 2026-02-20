namespace Assets;

public sealed class ModelData
{
    public required float[] Vertices { get; init; }
    public required uint[] Indices { get; init; }
    public required SubMesh[] SubMeshes { get; init; }
    public required MaterialData[] Materials { get; init; }
}

public readonly record struct SubMesh(int IndexStart, int IndexCount, int MaterialIndex);

public sealed class MaterialData
{
    // CPU-side “what textures exist”
    public int? BaseColorImage;
    public int? MetallicRoughnessImage;
    public int? NormalImage;

    // Factors fallback
    public System.Numerics.Vector4 BaseColorFactor = new(1, 1, 1, 1);
    public float MetallicFactor = 1f;
    public float RoughnessFactor = 1f;
}
