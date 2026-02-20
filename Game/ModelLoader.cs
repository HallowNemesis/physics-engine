using System.IO;

namespace Assets;

public static class ModelLoader
{
    public static ModelData Load(string path)
    {
        var ext = Path.GetExtension(path).ToLowerInvariant();
        return ext switch
        {
            //".obj" => ObjLoader.Load(path),
            ".glb" => GlbLoader.Load(path),
            ".gltf" => GlbLoader.Load(path), // (SharpGLTF can load .gltf too)
            _ => throw new NotSupportedException($"Unsupported model extension: {ext}")
        };
    }
}
