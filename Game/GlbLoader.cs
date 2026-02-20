using SharpGLTF.Schema2;

namespace Assets;

public static class GlbLoader
{
    public static ModelData Load(string path)
    {
        var model = ModelRoot.Load(path);

        var outVerts = new List<float>();
        var outIdx = new List<uint>();
        var submeshes = new List<SubMesh>();

        // Build a material table (unique materials in file)
        var materials = new MaterialData[model.LogicalMaterials.Count];
        for (int i = 0; i < materials.Length; i++)
            materials[i] = ReadMaterial(model.LogicalMaterials[i]);


        foreach (var mesh in model.LogicalMeshes)
        {
            foreach (var prim in mesh.Primitives)
            {
                var posAcc = prim.GetVertexAccessor("POSITION");
                var nrmAcc = prim.GetVertexAccessor("NORMAL"); // may be null
                var uvAcc = prim.GetVertexAccessor("TEXCOORD_0");   // may be null

                var positions = posAcc.AsVector3Array();
                var normals = nrmAcc != null ? nrmAcc.AsVector3Array() : null;
                var uvs = uvAcc != null ? uvAcc.AsVector2Array() : null;

                int baseVertex = outVerts.Count / 8; // pos3 + nrm3 + uv2 = 8 floats

                for (int i = 0; i < positions.Count; i++)
                {
                    var p = positions[i];
                    outVerts.Add(p.X); outVerts.Add(p.Y); outVerts.Add(p.Z);

                    if (normals != null)
                    {
                        var n = normals[i];
                        outVerts.Add(n.X); outVerts.Add(n.Y); outVerts.Add(n.Z);
                    }
                    else
                    {
                        outVerts.Add(0); outVerts.Add(0); outVerts.Add(0);
                    }

                    if (uvs != null)
                    {
                        var uv = uvs[i];
                        outVerts.Add(uv.X);
                        outVerts.Add(uv.Y);
                    }
                    else
                    {
                        outVerts.Add(0); outVerts.Add(0);
                    }
                }

                int indexStart = outIdx.Count;
                var indices = prim.GetIndices();
                foreach (var ix in indices)
                    outIdx.Add((uint)(baseVertex + ix));
                int indexCount = outIdx.Count - indexStart;

                int materialIndex = prim.Material?.LogicalIndex ?? 0;
                submeshes.Add(new SubMesh(indexStart, indexCount, materialIndex));
            }
        }

        // Generate normals if missing (or if they are all zero)
        if (!HasAnyNonZeroNormal(outVerts))
            GenerateVertexNormalsInPlace(outVerts, outIdx);

        return new ModelData
        {
            Vertices = outVerts.ToArray(),
            Indices = outIdx.ToArray(),
            SubMeshes = submeshes.ToArray(),
            Materials = materials
        };
    }

    private static bool HasAnyNonZeroNormal(List<float> v)
    {
        for (int i = 0; i < v.Count; i += 8)
        {
            float nx = v[i + 3], ny = v[i + 4], nz = v[i + 5];
            if (nx != 0f || ny != 0f || nz != 0f) return true;
        }
        return false;
    }

    private static void GenerateVertexNormalsInPlace(List<float> verts, List<uint> idx)
    {
        // reset normals
        for (int i = 0; i < verts.Count; i += 8)
        {
            verts[i + 3] = 0;
            verts[i + 4] = 0;
            verts[i + 5] = 0;
        }

        for (int i = 0; i < idx.Count; i += 3)
        {
            int v0 = (int)idx[i + 0];
            int v1 = (int)idx[i + 1];
            int v2 = (int)idx[i + 2];

            int i0 = v0 * 8;
            int i1 = v1 * 8;
            int i2 = v2 * 8;

            var p0 = new System.Numerics.Vector3(verts[i0], verts[i0 + 1], verts[i0 + 2]);
            var p1 = new System.Numerics.Vector3(verts[i1], verts[i1 + 1], verts[i1 + 2]);
            var p2 = new System.Numerics.Vector3(verts[i2], verts[i2 + 1], verts[i2 + 2]);

            var fn = System.Numerics.Vector3.Cross(p1 - p0, p2 - p0);
            if (fn.LengthSquared() <= 1e-20f) continue;

            verts[i0 + 3] += fn.X; verts[i0 + 4] += fn.Y; verts[i0 + 5] += fn.Z;
            verts[i1 + 3] += fn.X; verts[i1 + 4] += fn.Y; verts[i1 + 5] += fn.Z;
            verts[i2 + 3] += fn.X; verts[i2 + 4] += fn.Y; verts[i2 + 5] += fn.Z;
        }

        for (int i = 0; i < verts.Count; i += 8)
        {
            var n = new System.Numerics.Vector3(verts[i + 3], verts[i + 4], verts[i + 5]);
            n = n.LengthSquared() <= 1e-20f ? new System.Numerics.Vector3(0, 1, 0)
                                            : System.Numerics.Vector3.Normalize(n);

            verts[i + 3] = n.X; verts[i + 4] = n.Y; verts[i + 5] = n.Z;
        }
    }
    public static Bounds3 GetLocalBounds(string path)
    {
        var model = SharpGLTF.Schema2.ModelRoot.Load(path);

        var min = new System.Numerics.Vector3(float.PositiveInfinity);
        var max = new System.Numerics.Vector3(float.NegativeInfinity);

        foreach (var mesh in model.LogicalMeshes)
            foreach (var prim in mesh.Primitives)
            {
                var posAcc = prim.GetVertexAccessor("POSITION");
                var positions = posAcc.AsVector3Array();

                for (int i = 0; i < positions.Count; i++)
                {
                    var p = positions[i];
                    min = System.Numerics.Vector3.Min(min, p);
                    max = System.Numerics.Vector3.Max(max, p);
                }
            }

        if (float.IsInfinity(min.X))
            return new Bounds3(System.Numerics.Vector3.Zero, System.Numerics.Vector3.Zero);

        return new Bounds3(min, max);
    }
    private static MaterialData ReadMaterial(Material mat)
    {
        var md = new MaterialData();

        // BaseColor
        var baseCh = mat.FindChannel("BaseColor");
        if (baseCh.HasValue)
        {
            // Texture -> Image index
            var tex = baseCh.Value.Texture;
            md.BaseColorImage = tex?.PrimaryImage?.LogicalIndex;

            // Factor (optional, if you want it now)
            // baseCh.Value.Parameter is usually a Vector4 (RGBA) for BaseColor
            // var f = (System.Numerics.Vector4)baseCh.Value.Parameter;
            // md.BaseColorFactor = f;
        }

        // MetallicRoughness (packed: G=roughness, B=metallic)
        var mrCh = mat.FindChannel("MetallicRoughness");
        if (mrCh.HasValue)
        {
            var tex = mrCh.Value.Texture;
            md.MetallicRoughnessImage = tex?.PrimaryImage?.LogicalIndex;

            // Factors (optional)
            // mrCh.Value.Parameter is often a Vector2-ish (metallic, roughness) depending on version
            // so I recommend reading factors later from your old pbr properties IF you find them.
        }

        // Normal
        var nCh = mat.FindChannel("Normal");
        if (nCh.HasValue)
        {
            var tex = nCh.Value.Texture;
            md.NormalImage = tex?.PrimaryImage?.LogicalIndex;
        }

        return md;
    }

}

public readonly record struct Bounds3(System.Numerics.Vector3 Min, System.Numerics.Vector3 Max)
{
    public System.Numerics.Vector3 Size => Max - Min;
    public System.Numerics.Vector3 Center => (Min + Max) * 0.5f;
}