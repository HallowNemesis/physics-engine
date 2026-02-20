using System.Collections.Concurrent;
using Graphics;

namespace Assets;

public sealed class ModelAsset
{
    public required ModelData Cpu;   // float[] + uint[]
    public GpuModel? Gpu;                // created lazily
}

public static class AssetCache
{
    private static readonly ConcurrentDictionary<string, ModelAsset> _cache = new();

    public static ModelAsset Get(string path)
    {
        return _cache.GetOrAdd(path, p => new ModelAsset { Cpu = ModelLoader.Load(p) });
    }

    public static Mesh GetGpuMesh(string path)
    {
        return GetGpuModel(path).Mesh;
    }

    public static GpuModel GetGpuModel(string path)
    {
        var asset = Get(path);
        if (asset.Gpu is null) // change ModelAsset.Gpu to GpuModel
        {
            asset.Gpu = GpuModelFactory.Create(asset.Cpu, path);
        }
        return asset.Gpu;
    }
}
