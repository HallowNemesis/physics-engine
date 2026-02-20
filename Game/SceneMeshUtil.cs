using Assets;
using Physics.Math;

namespace Scene;

public static class SceneMeshUtil
{
    public static Vector3[] ExtractTransformedPositions(ModelData md, Vector3 scale, Vector3 pivot)
    {
        var v = md.Vertices;
        int count = v.Length / 6;
        var outPos = new Vector3[count];

        for (int i = 0; i < count; i++)
        {
            int o = i * 6;
            var p = new Vector3(v[o + 0], v[o + 1], v[o + 2]);

            p += pivot;
            p = new Vector3(p.X * scale.X, p.Y * scale.Y, p.Z * scale.Z);

            outPos[i] = p;  
        }

        return outPos;
    }

    public static double MinY(Vector3[] verts)
    {
        double minY = double.PositiveInfinity;
        for (int i = 0; i < verts.Length; i++)
            if (verts[i].Y < minY) minY = verts[i].Y;
        return minY;
    }
}
