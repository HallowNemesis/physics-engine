using Physics.Dynamics;
using Physics.Math;

namespace Physics.Collision;

internal static class MeshNarrowphase
{
    public static bool SphereStaticMesh(
        RigidBody sphereBody, SphereShape sphere,
        RigidBody meshBody, StaticMeshShape mesh,
        out Manifold m)
    {
        // Convention: Normal is A -> B. Here A = sphere, B = mesh.
        m = new Manifold { A = sphereBody, B = meshBody };

        double r = sphere.Radius;

        // Mesh transform (world)
        Matrix3 R = Matrix3.FromQuaternion(meshBody.Orientation);
        Matrix3 Rt = R.Transposed();

        // Sphere center in mesh-local (mesh assumed rigidly transformed by meshBody)
        Vector3 cWorld = sphereBody.Position;
        Vector3 cLocal = Rt * (cWorld - meshBody.Position);

        // Query BVH with local AABB around sphere
        var q = new Aabb(
            cLocal - new Vector3(r, r, r),
            cLocal + new Vector3(r, r, r));

        var candidates = TempTriList;
        candidates.Clear();
        mesh.Query(q, candidates);

        double bestPen = 0.0;
        Vector3 bestNLocal_AtoB = default; // sphere->mesh in mesh-local
        Vector3 bestPLocal = default;      // point on mesh surface in mesh-local
        bool hit = false;

        foreach (int ti in candidates)
        {
            var tri = mesh.Tris[ti]; // NOTE: Tris are in mesh-local

            Vector3 cp = ClosestPointOnTri(cLocal, tri.A, tri.B, tri.C);

            // d points tri->sphere
            Vector3 d = cLocal - cp;
            double dist2 = Vector3.Dot(d, d);

            if (dist2 <= r * r)
            {
                double dist = System.Math.Sqrt(System.Math.Max(dist2, 1e-18));

                // tri->sphere normal; fallback if center sits exactly on cp
                Vector3 nTriToSphere = dist > 1e-9 ? (d / dist) : Vector3.UnitY;

                // We need Normal A->B (sphere->mesh)
                Vector3 nSphereToMesh = -nTriToSphere;

                double pen = r - dist;

                if (!hit || pen > bestPen)
                {
                    hit = true;
                    bestPen = pen;
                    bestNLocal_AtoB = nSphereToMesh;
                    bestPLocal = cp;
                }
            }
        }

        if (!hit) return false;

        // Convert contact back to world:
        // pointWorld = meshPos + R * pLocal
        // normalWorld = R * nLocal
        Vector3 pWorld = meshBody.Position + (R * bestPLocal);
        Vector3 nWorld = (R * bestNLocal_AtoB).Normalized();

        m.Normal = nWorld;
        m.Penetration = bestPen;
        m.ContactPoint = pWorld;

        m.Restitution = System.Math.Min(sphereBody.Restitution, meshBody.Restitution);
        m.Friction = System.Math.Sqrt(sphereBody.Friction * meshBody.Friction);

        return true;
    }

    // Closest point on triangle (Ericson)
    private static Vector3 ClosestPointOnTri(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
    {
        Vector3 ab = b - a; Vector3 ac = c - a; Vector3 ap = p - a;
        double d1 = Vector3.Dot(ab, ap);
        double d2 = Vector3.Dot(ac, ap);
        if (d1 <= 0 && d2 <= 0) return a;

        Vector3 bp = p - b;
        double d3 = Vector3.Dot(ab, bp);
        double d4 = Vector3.Dot(ac, bp);
        if (d3 >= 0 && d4 <= d3) return b;

        double vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0)
        {
            double v = d1 / (d1 - d3);
            return a + ab * v;
        }

        Vector3 cp = p - c;
        double d5 = Vector3.Dot(ab, cp);
        double d6 = Vector3.Dot(ac, cp);
        if (d6 >= 0 && d5 <= d6) return c;

        double vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0)
        {
            double w = d2 / (d2 - d6);
            return a + ac * w;
        }

        double va = d3 * d6 - d5 * d4;
        if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
        {
            double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return b + (c - b) * w;
        }

        double denom = 1.0 / (va + vb + vc);
        double v2 = vb * denom;
        double w2 = vc * denom;
        return a + ab * v2 + ac * w2;
    }

    [ThreadStatic] private static List<int>? _tempTriList;
    private static List<int> TempTriList => _tempTriList ??= new List<int>(256);
}
