//using System.Globalization;

//namespace Assets;

//public static class ObjLoader
//{
//    public static ModelData Load(string path)
//    {
//        var inv = CultureInfo.InvariantCulture;

//        var positions = new List<System.Numerics.Vector3>();
//        var normals = new List<System.Numerics.Vector3>();

//        // unique vertex per (positionIndex, normalIndex)
//        var vertMap = new Dictionary<(int vp, int vn), int>();
//        var outVerts = new List<float>();     // pos(3)+normal(3)
//        var outIdx = new List<uint>();        // FIX: uint

//        foreach (var raw in File.ReadLines(path))
//        {
//            var line = raw.Trim();
//            if (line.Length == 0 || line[0] == '#') continue;

//            if (line.StartsWith("v "))
//            {
//                var p = Split(line);
//                positions.Add(new System.Numerics.Vector3(
//                    float.Parse(p[1], inv),
//                    float.Parse(p[2], inv),
//                    float.Parse(p[3], inv)
//                ));
//            }
//            else if (line.StartsWith("vn "))
//            {
//                var p = Split(line);
//                normals.Add(System.Numerics.Vector3.Normalize(new System.Numerics.Vector3(
//                    float.Parse(p[1], inv),
//                    float.Parse(p[2], inv),
//                    float.Parse(p[3], inv)
//                )));
//            }
//            else if (line.StartsWith("f "))
//            {
//                var parts = Split(line);

//                // triangulate fan: f a b c d -> (a,b,c) (a,c,d)
//                Span<int> face = stackalloc int[parts.Length - 1];
//                for (int i = 1; i < parts.Length; i++)
//                    face[i - 1] = GetOrCreateVertex(parts[i], positions, normals, vertMap, outVerts);

//                for (int i = 1; i + 1 < face.Length; i++)
//                {
//                    outIdx.Add((uint)face[0]);
//                    outIdx.Add((uint)face[i]);
//                    outIdx.Add((uint)face[i + 1]);
//                }
//            }
//        }

//        // If OBJ provided no normals, compute vertex normals from triangles
//        if (normals.Count == 0)
//            GenerateVertexNormalsInPlace(outVerts, outIdx);

//        return new ModelData
//        {
//            Vertices = outVerts.ToArray(),
//            Indices = outIdx.ToArray()
//        };
//    }

//    private static string[] Split(string line)
//        => line.Split(' ', StringSplitOptions.RemoveEmptyEntries);

//    private static int FixIndex(int idx, int count)
//    {
//        // OBJ: 1-based, negative allowed (relative to end)
//        if (idx > 0) return idx - 1;
//        if (idx < 0) return count + idx;
//        return 0;
//    }

//    private static int GetOrCreateVertex(
//        string token,
//        List<System.Numerics.Vector3> positions,
//        List<System.Numerics.Vector3> normals,
//        Dictionary<(int vp, int vn), int> map,
//        List<float> outVerts)
//    {
//        // token formats: v, v/vt, v//vn, v/vt/vn
//        int vp = 0, vn = -1;

//        var s = token.Split('/');
//        vp = FixIndex(int.Parse(s[0], CultureInfo.InvariantCulture), positions.Count);

//        if (s.Length == 3 && !string.IsNullOrEmpty(s[2]))
//            vn = FixIndex(int.Parse(s[2], CultureInfo.InvariantCulture), normals.Count);

//        var key = (vp, vn);
//        if (map.TryGetValue(key, out var existing))
//            return existing;

//        var p = positions[vp];
//        System.Numerics.Vector3 n = vn >= 0 && vn < normals.Count ? normals[vn] : new System.Numerics.Vector3(0, 0, 0);

//        int newIndex = map.Count;
//        map[key] = newIndex;

//        outVerts.Add(p.X); outVerts.Add(p.Y); outVerts.Add(p.Z);
//        outVerts.Add(n.X); outVerts.Add(n.Y); outVerts.Add(n.Z);

//        return newIndex;
//    }

//    private static void GenerateVertexNormalsInPlace(List<float> verts, List<uint> idx)
//    {
//        // verts layout per vertex: [px,py,pz,nx,ny,nz]
//        // reset normals to 0
//        for (int i = 0; i < verts.Count; i += 6)
//        {
//            verts[i + 3] = 0;
//            verts[i + 4] = 0;
//            verts[i + 5] = 0;
//        }

//        for (int i = 0; i < idx.Count; i += 3)
//        {
//            int v0 = (int)idx[i + 0];
//            int v1 = (int)idx[i + 1];
//            int v2 = (int)idx[i + 2];

//            int i0 = v0 * 6;
//            int i1 = v1 * 6;
//            int i2 = v2 * 6;

//            var p0 = new System.Numerics.Vector3(verts[i0], verts[i0 + 1], verts[i0 + 2]);
//            var p1 = new System.Numerics.Vector3(verts[i1], verts[i1 + 1], verts[i1 + 2]);
//            var p2 = new System.Numerics.Vector3(verts[i2], verts[i2 + 1], verts[i2 + 2]);

//            var e1 = p1 - p0;
//            var e2 = p2 - p0;

//            // area-weighted face normal
//            var fn = System.Numerics.Vector3.Cross(e1, e2);
//            if (fn.LengthSquared() <= 1e-20f) continue;

//            // accumulate to vertices
//            verts[i0 + 3] += fn.X; verts[i0 + 4] += fn.Y; verts[i0 + 5] += fn.Z;
//            verts[i1 + 3] += fn.X; verts[i1 + 4] += fn.Y; verts[i1 + 5] += fn.Z;
//            verts[i2 + 3] += fn.X; verts[i2 + 4] += fn.Y; verts[i2 + 5] += fn.Z;
//        }

//        // normalize normals
//        for (int i = 0; i < verts.Count; i += 6)
//        {
//            var n = new System.Numerics.Vector3(verts[i + 3], verts[i + 4], verts[i + 5]);
//            if (n.LengthSquared() <= 1e-20f)
//                n = new System.Numerics.Vector3(0, 1, 0);
//            else
//                n = System.Numerics.Vector3.Normalize(n);

//            verts[i + 3] = n.X; verts[i + 4] = n.Y; verts[i + 5] = n.Z;
//        }
//    }
//}
