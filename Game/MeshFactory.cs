using OpenTK.Mathematics;
using Assets;

namespace Graphics;

public static class MeshFactory
{
    // Engine-wide vertex format:
    // vec3 pos, vec3 normal, vec2 uv  => 8 floats per vertex
    private const int FloatsPerVertex = 8;

    public static Mesh CreateCube()
    {
        // 24 unique verts (per-face normals)
        var verts = new List<float>(24 * FloatsPerVertex);
        var inds = new List<uint>(36);

        void AddVert(Vector3 p, Vector3 n, Vector2 uv)
        {
            verts.Add(p.X); verts.Add(p.Y); verts.Add(p.Z);
            verts.Add(n.X); verts.Add(n.Y); verts.Add(n.Z);
            verts.Add(uv.X); verts.Add(uv.Y);
        }

        void AddFace(Vector3 n, Vector3 a, Vector3 b, Vector3 c, Vector3 d)
        {
            uint start = (uint)(verts.Count / FloatsPerVertex);

            // Simple per-face UVs
            AddVert(a, n, new Vector2(0, 0));
            AddVert(b, n, new Vector2(1, 0));
            AddVert(c, n, new Vector2(1, 1));
            AddVert(d, n, new Vector2(0, 1));

            // two triangles: 0-1-2, 0-2-3
            inds.Add(start + 0); inds.Add(start + 1); inds.Add(start + 2);
            inds.Add(start + 0); inds.Add(start + 2); inds.Add(start + 3);
        }

        float s = 0.5f;

        // +Z
        AddFace(new Vector3(0, 0, 1), new(-s, -s, s), new(s, -s, s), new(s, s, s), new(-s, s, s));
        // -Z
        AddFace(new Vector3(0, 0, -1), new(s, -s, -s), new(-s, -s, -s), new(-s, s, -s), new(s, s, -s));
        // +X
        AddFace(new Vector3(1, 0, 0), new(s, -s, s), new(s, -s, -s), new(s, s, -s), new(s, s, s));
        // -X
        AddFace(new Vector3(-1, 0, 0), new(-s, -s, -s), new(-s, -s, s), new(-s, s, s), new(-s, s, -s));
        // +Y
        AddFace(new Vector3(0, 1, 0), new(-s, s, s), new(s, s, s), new(s, s, -s), new(-s, s, -s));
        // -Y
        AddFace(new Vector3(0, -1, 0), new(-s, -s, -s), new(s, -s, -s), new(s, -s, s), new(-s, -s, s));

        return new Mesh(verts.ToArray(), inds.ToArray());
    }

    public static Mesh CreateIcoSphere(int subdivisions)
    {
        var t = (1f + MathF.Sqrt(5f)) / 2f;

        var v = new List<Vector3>
        {
            new(-1,  t,  0), new( 1,  t,  0), new(-1, -t,  0), new( 1, -t,  0),
            new( 0, -1,  t), new( 0,  1,  t), new( 0, -1, -t), new( 0,  1, -t),
            new( t,  0, -1), new( t,  0,  1), new(-t,  0, -1), new(-t,  0,  1)
        };

        for (int i = 0; i < v.Count; i++) v[i] = v[i].Normalized();

        var faces = new List<(int a, int b, int c)>
        {
            (0,11,5),(0,5,1),(0,1,7),(0,7,10),(0,10,11),
            (1,5,9),(5,11,4),(11,10,2),(10,7,6),(7,1,8),
            (3,9,4),(3,4,2),(3,2,6),(3,6,8),(3,8,9),
            (4,9,5),(2,4,11),(6,2,10),(8,6,7),(9,8,1),
        };

        var midpointCache = new Dictionary<(int, int), int>();

        int Mid(int a, int b)
        {
            var key = a < b ? (a, b) : (b, a);
            if (midpointCache.TryGetValue(key, out int idx)) return idx;

            var m = (v[a] + v[b]) * 0.5f;
            v.Add(m.Normalized());
            idx = v.Count - 1;
            midpointCache[key] = idx;
            return idx;
        }

        for (int s = 0; s < subdivisions; s++)
        {
            midpointCache.Clear();
            var newFaces = new List<(int a, int b, int c)>(faces.Count * 4);

            foreach (var (a, b, c) in faces)
            {
                int ab = Mid(a, b);
                int bc = Mid(b, c);
                int ca = Mid(c, a);

                newFaces.Add((a, ab, ca));
                newFaces.Add((b, bc, ab));
                newFaces.Add((c, ca, bc));
                newFaces.Add((ab, bc, ca));
            }

            faces = newFaces;
        }

        var interleaved = new float[v.Count * FloatsPerVertex];
        for (int i = 0; i < v.Count; i++)
        {
            var p = v[i];
            var n = p; // unit sphere

            // basic spherical uv (debug-friendly)
            float u = 0.5f + MathF.Atan2(p.Z, p.X) / (2f * MathF.PI);
            float vv = 0.5f - MathF.Asin(p.Y) / MathF.PI;

            int o = i * FloatsPerVertex;
            interleaved[o + 0] = p.X; interleaved[o + 1] = p.Y; interleaved[o + 2] = p.Z;
            interleaved[o + 3] = n.X; interleaved[o + 4] = n.Y; interleaved[o + 5] = n.Z;
            interleaved[o + 6] = u; interleaved[o + 7] = vv;
        }

        var indices = new uint[faces.Count * 3];
        for (int i = 0; i < faces.Count; i++)
        {
            indices[i * 3 + 0] = (uint)faces[i].a;
            indices[i * 3 + 1] = (uint)faces[i].b;
            indices[i * 3 + 2] = (uint)faces[i].c;
        }

        return new Mesh(interleaved, indices);
    }

    public static Mesh CreateGridLines(int size, int spacing)
    {
        var verts = new List<float>();
        var inds = new List<uint>();

        float half = size * 0.5f;

        void AddV(Vector3 p)
        {
            // normal unused for lines; keep something valid
            var n = Vector3.UnitY;

            verts.Add(p.X); verts.Add(p.Y); verts.Add(p.Z);
            verts.Add(n.X); verts.Add(n.Y); verts.Add(n.Z);
            verts.Add(0f); verts.Add(0f); // dummy UV
        }

        void AddLine(Vector3 a, Vector3 b)
        {
            uint start = (uint)(verts.Count / FloatsPerVertex);
            AddV(a);
            AddV(b);

            inds.Add(start + 0);
            inds.Add(start + 1);
        }

        for (int i = -size; i <= size; i += spacing)
        {
            float x = i;
            AddLine(new Vector3(x, 0, -half), new Vector3(x, 0, half)); // along Z
            AddLine(new Vector3(-half, 0, x), new Vector3(half, 0, x)); // along X
        }

        return new Mesh(verts.ToArray(), inds.ToArray());
    }

    public static Mesh CreateFrom(ModelData data)
    {
        // Assumes ModelData.Vertices is already in the 8-float format
        return new Mesh(data.Vertices, data.Indices);
    }
}
