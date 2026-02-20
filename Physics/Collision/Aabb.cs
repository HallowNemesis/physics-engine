namespace Physics.Collision;

public readonly struct Aabb
{
    public readonly Math.Vector3 Min;
    public readonly Math.Vector3 Max;

    public Aabb(Math.Vector3 min, Math.Vector3 max) { Min = min; Max = max; }

    public bool Overlaps(in Aabb o) =>
        Min.X <= o.Max.X && Max.X >= o.Min.X &&
        Min.Y <= o.Max.Y && Max.Y >= o.Min.Y &&
        Min.Z <= o.Max.Z && Max.Z >= o.Min.Z;

    public static Aabb Union(in Aabb a, in Aabb b) =>
        new(
            new Math.Vector3(
                System.Math.Min(a.Min.X, b.Min.X),
                System.Math.Min(a.Min.Y, b.Min.Y),
                System.Math.Min(a.Min.Z, b.Min.Z)),
            new Math.Vector3(
                System.Math.Max(a.Max.X, b.Max.X),
                System.Math.Max(a.Max.Y, b.Max.Y),
                System.Math.Max(a.Max.Z, b.Max.Z))
        );
}

public readonly struct Tri
{
    public readonly Math.Vector3 A, B, C;
    public Tri(Math.Vector3 a, Math.Vector3 b, Math.Vector3 c) { A = a; B = b; C = c; }

    public Aabb Bounds()
    {
        var min = new Math.Vector3(
            System.Math.Min(A.X, System.Math.Min(B.X, C.X)),
            System.Math.Min(A.Y, System.Math.Min(B.Y, C.Y)),
            System.Math.Min(A.Z, System.Math.Min(B.Z, C.Z)));
        var max = new Math.Vector3(
            System.Math.Max(A.X, System.Math.Max(B.X, C.X)),
            System.Math.Max(A.Y, System.Math.Max(B.Y, C.Y)),
            System.Math.Max(A.Z, System.Math.Max(B.Z, C.Z)));
        return new Aabb(min, max);
    }
}
