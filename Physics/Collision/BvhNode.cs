namespace Physics.Collision;

internal sealed class BvhNode
{
    public Aabb Bounds;
    public BvhNode? Left;
    public BvhNode? Right;
    public int Start, Count; // leaf range into TriIndices

    public bool IsLeaf => Left == null && Right == null;
}
