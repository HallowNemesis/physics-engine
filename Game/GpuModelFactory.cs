using System.Collections.Generic;
using SharpGLTF.Schema2;
using Graphics;

using GfxMesh = Graphics.Mesh;

namespace Assets;

public static class GpuModelFactory
{
    public static GpuModel Create(ModelData cpu, string glbPath)
    {
        var mesh = new GfxMesh(cpu.Vertices, cpu.Indices);

        var model = ModelRoot.Load(glbPath);

        var imageTexCache = new Dictionary<int, Texture2D>();

        Texture2D GetOrCreate(int imageIndex, bool srgb)
        {
            if (imageTexCache.TryGetValue(imageIndex, out var t))
                return t;

            var img = model.LogicalImages[imageIndex];

            // MemoryImage -> byte[]
            var bytes = ReadAllBytesFromMemoryImage(img.Content);

            var tex = new Texture2D(bytes, srgb);
            imageTexCache[imageIndex] = tex;
            return tex;
        }

        var gpuMats = new GpuMaterial[cpu.Materials.Length];
        for (int i = 0; i < gpuMats.Length; i++)
        {
            var md = cpu.Materials[i];
            var gm = new GpuMaterial();

            if (md.BaseColorImage is int bc) gm.BaseColor = GetOrCreate(bc, srgb: true);
            if (md.MetallicRoughnessImage is int mr) gm.MetallicRoughness = GetOrCreate(mr, srgb: false);
            if (md.NormalImage is int n) gm.Normal = GetOrCreate(n, srgb: false);

            gpuMats[i] = gm;
        }

        return new GpuModel
        {
            Mesh = mesh,
            SubMeshes = cpu.SubMeshes,
            Materials = gpuMats
        };
    }

    private static byte[] ReadAllBytesFromMemoryImage(SharpGLTF.Memory.MemoryImage mem)
    {
        // Depending on version, this might be Open() or OpenRead()
        using var s = mem.Open();
        using var ms = new System.IO.MemoryStream();
        s.CopyTo(ms);
        return ms.ToArray();
    }
}
