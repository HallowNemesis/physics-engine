using Physics.Math;

namespace Physics.Collision;

public sealed class StaticMeshShape : IShape
{
    public readonly Tri[] Tris;

    private readonly int[] _triOrder;
    private readonly BvhNode _root;

    public StaticMeshShape(Vector3[] vertices, uint[] indices)
    {
        if (indices.Length % 3 != 0)
            throw new ArgumentException("StaticMeshShape: indices must be triangles (multiple of 3).");

        Tris = new Tri[indices.Length / 3];
        for (int t = 0; t < Tris.Length; t++)
        {
            int i0 = (int)indices[t * 3 + 0];
            int i1 = (int)indices[t * 3 + 1];
            int i2 = (int)indices[t * 3 + 2];

            Tris[t] = new Tri(vertices[i0], vertices[i1], vertices[i2]);
        }

        _triOrder = new int[Tris.Length];
        for (int i = 0; i < _triOrder.Length; i++) _triOrder[i] = i;

        _root = Build(start: 0, count: Tris.Length);
    }

    public Matrix3 ComputeInvInertiaLocal(double mass) => Matrix3.Zero;

    public void Query(in Aabb aabb, List<int> results)
    {
        QueryNode(_root, aabb, results);
    }

    private void QueryNode(BvhNode node, in Aabb aabb, List<int> results)
    {
        if (!node.Bounds.Overlaps(aabb)) return;

        if (node.IsLeaf)
        {
            for (int i = 0; i < node.Count; i++)
                results.Add(_triOrder[node.Start + i]);
            return;
        }

        QueryNode(node.Left!, aabb, results);
        QueryNode(node.Right!, aabb, results);
    }

    private BvhNode Build(int start, int count)
    {
        var node = new BvhNode { Start = start, Count = count };

        // Bounds for this node
        Aabb bounds = Tris[_triOrder[start]].Bounds();
        for (int i = 1; i < count; i++)
            bounds = Aabb.Union(bounds, Tris[_triOrder[start + i]].Bounds());
        node.Bounds = bounds;

        // Leaf threshold
        if (count <= 8)
            return node;

        // Choose axis by bounds extent
        var size = bounds.Max - bounds.Min;
        int axis = (size.X > size.Y && size.X > size.Z) ? 0 : (size.Y > size.Z ? 1 : 2);

        // Sort by centroid along chosen axis
        Array.Sort(_triOrder, start, count, new TriCentroidComparer(Tris, axis));

        int mid = start + count / 2;

        node.Left = Build(start, mid - start);
        node.Right = Build(mid, start + count - mid);

        return node;
    }

    private sealed class TriCentroidComparer : IComparer<int>
    {
        private readonly Tri[] _tris;
        private readonly int _axis;

        public TriCentroidComparer(Tri[] tris, int axis)
        {
            _tris = tris;
            _axis = axis;
        }

        public int Compare(int x, int y)
        {
            double cx = CentroidAxis(_tris[x], _axis);
            double cy = CentroidAxis(_tris[y], _axis);
            return cx.CompareTo(cy);
        }

        private static double CentroidAxis(in Tri t, int axis)
        {
            double ax = (t.A.X + t.B.X + t.C.X) / 3.0;
            double ay = (t.A.Y + t.B.Y + t.C.Y) / 3.0;
            double az = (t.A.Z + t.B.Z + t.C.Z) / 3.0;

            return axis switch
            {
                0 => ax,
                1 => ay,
                _ => az
            };
        }
    }
}
