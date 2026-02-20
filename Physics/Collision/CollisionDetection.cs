using Physics.Dynamics;
using Physics.Math;

namespace Physics.Collision;

public static class CollisionDetection
{
    public static void BuildContacts(IReadOnlyList<RigidBody> bodies, List<Manifold> outContacts)
    {
        outContacts.Clear();

        for (int i = 0; i < bodies.Count; i++)
            for (int j = i + 1; j < bodies.Count; j++)
            {
                var a = bodies[i];
                var b = bodies[j];

                if (a.IsStatic && b.IsStatic) continue;

                var sa = a.Shape;
                var sb = b.Shape;

                if (sa is SphereShape sA && sb is SphereShape sB)
                {
                    if (SphereSphere(a, sA, b, sB, out var m)) outContacts.Add(m);
                }
                else if (sa is SphereShape sphA && sb is PlaneShape plB)
                {
                    if (SpherePlane(a, sphA, b, plB, out var m)) outContacts.Add(m);
                }
                else if (sa is PlaneShape plA && sb is SphereShape sphB)
                {
                    if (SpherePlane(b, sphB, a, plA, out var m)) outContacts.Add(m);
                }
                else if (sa is SphereShape sphA2 && sb is BoxShape boxB)
                {
                    if (SphereBox(a, sphA2, b, boxB, out var m)) outContacts.Add(m);
                }
                else if (sa is BoxShape boxA && sb is SphereShape sphB2)
                {
                    if (SphereBox(b, sphB2, a, boxA, out var m)) outContacts.Add(m);
                }
                else if (sa is SphereShape sphA3 && sb is StaticMeshShape ms)
                {
                    if (MeshNarrowphase.SphereStaticMesh(a, sphA3, b, ms, out var m)) outContacts.Add(m);
                }
                else if (sa is StaticMeshShape ms2 && sb is SphereShape sb2)
                {
                    if (MeshNarrowphase.SphereStaticMesh(b, sb2, a, ms2, out var m)) outContacts.Add(m);
                }
                else if (sa is BoxShape boxA2 && sb is BoxShape boxB2)
                {
                    if (BoxBox(a, boxA2, b, boxB2, out var m)) outContacts.Add(m);
                }
            }
    }


    private static bool SphereSphere(RigidBody a, SphereShape sa, RigidBody b, SphereShape sb, out Manifold m)
    {
        m = new Manifold { A = a, B = b };

        Vector3 d = b.Position - a.Position;
        double dist2 = d.LengthSquared();
        double r = sa.Radius + sb.Radius;

        if (dist2 >= r * r)
            return false;

        double dist = double.Sqrt(dist2);

        Vector3 n;
        if (dist <= 1e-8)
        {
            n = Vector3.UnitY; // stable fallback
            m.Penetration = r;
            m.ContactPoint = a.Position + n * sa.Radius;
        }
        else
        {
            n = d / dist;            // A -> B
            m.Penetration = r - dist;
            m.ContactPoint = a.Position + n * sa.Radius; // point on A toward B
        }

        m.Normal = n;

        m.Restitution = double.Min(a.Restitution, b.Restitution);
        m.Friction = double.Sqrt(a.Friction * b.Friction);
        FinalizeManifold(ref m);

        return true;
    }

    private static bool SpherePlane(RigidBody sphereBody, SphereShape sphere, RigidBody planeBody, PlaneShape plane, out Manifold m)
    {
        // Convention: Manifold.Normal points from A -> B.
        // Here A is sphere, B is plane body.
        m = new Manifold { A = sphereBody, B = planeBody };

        Vector3 pn = plane.Normal; // normalized
        double r = sphere.Radius;

        // Signed distance from sphere center to plane: d = Dot(pn, x) - offset
        // Positive means "in direction of plane normal"
        double d = Vector3.Dot(pn, sphereBody.Position) - plane.Offset;

        if (d > r)
            return false;

        m.Penetration = r - d;

        // We want Normal from sphere -> plane.
        // If the sphere is above the plane (d > 0 for ground with pn=+Y),
        // the direction from sphere toward the plane is -pn.
        // If the sphere is below the plane (d < 0), toward the plane is +pn.
        Vector3 nAtoB = (d >= 0.0) ? -pn : pn;
        m.Normal = nAtoB;

        // Contact point on the plane directly under/over the sphere center (projection)
        // pointPlane = x - pn * d
        Vector3 pointOnPlane = sphereBody.Position - pn * d;
        m.ContactPoint = pointOnPlane;

        m.Restitution = double.Min(sphereBody.Restitution, planeBody.Restitution);
        m.Friction = double.Sqrt(sphereBody.Friction * planeBody.Friction);
        FinalizeManifold(ref m);

        return true;
    }

    private static bool SphereBox(RigidBody sphereBody, SphereShape sphere, RigidBody boxBody, BoxShape box, out Manifold m)
    {
        // Manifold expects Normal from A -> B. Here A is sphere, B is box.
        m = new Manifold { A = sphereBody, B = boxBody };

        double r = sphere.Radius;

        Matrix3 R = Matrix3.FromQuaternion(boxBody.Orientation);
        Matrix3 Rt = R.Transposed();

        // sphere center in box local space
        Vector3 rel = sphereBody.Position - boxBody.Position;
        Vector3 pLocal = Rt * rel;

        Vector3 he = box.HalfExtents;

        double cx = Clamp(pLocal.X, -he.X, he.X);
        double cy = Clamp(pLocal.Y, -he.Y, he.Y);
        double cz = Clamp(pLocal.Z, -he.Z, he.Z);

        Vector3 closestLocal = new Vector3(cx, cy, cz);

        Vector3 dLocal = pLocal - closestLocal;
        double dist2 = dLocal.LengthSquared();

        if (dist2 > r * r)
            return false;

        Vector3 nWorld_AtoB;
        double penetration;
        Vector3 contactWorld;

        if (dist2 > 1e-12)
        {
            double dist = double.Sqrt(dist2);
            Vector3 nLocal_boxToSphere = dLocal / dist;
            Vector3 nWorld_boxToSphere = R * nLocal_boxToSphere;

            // A->B (sphere->box) is opposite of box->sphere
            nWorld_AtoB = -nWorld_boxToSphere;

            penetration = r - dist;
            contactWorld = boxBody.Position + (R * closestLocal);
        }
        else
        {
            // inside: pick minimum push-out axis
            double dx = he.X - double.Abs(pLocal.X);
            double dy = he.Y - double.Abs(pLocal.Y);
            double dz = he.Z - double.Abs(pLocal.Z);

            Vector3 nLocal_boxToSphere;
            double push;

            if (dx <= dy && dx <= dz)
            {
                nLocal_boxToSphere = new Vector3((pLocal.X >= 0) ? 1 : -1, 0, 0);
                push = dx;
                closestLocal = new Vector3((pLocal.X >= 0) ? he.X : -he.X, pLocal.Y, pLocal.Z);
            }
            else if (dy <= dz)
            {
                nLocal_boxToSphere = new Vector3(0, (pLocal.Y >= 0) ? 1 : -1, 0);
                push = dy;
                closestLocal = new Vector3(pLocal.X, (pLocal.Y >= 0) ? he.Y : -he.Y, pLocal.Z);
            }
            else
            {
                nLocal_boxToSphere = new Vector3(0, 0, (pLocal.Z >= 0) ? 1 : -1);
                push = dz;
                closestLocal = new Vector3(pLocal.X, pLocal.Y, (pLocal.Z >= 0) ? he.Z : -he.Z);
            }

            Vector3 nWorld_boxToSphere = R * nLocal_boxToSphere;
            nWorld_AtoB = -nWorld_boxToSphere;

            penetration = r + push;
            contactWorld = boxBody.Position + (R * closestLocal);
        }

        m.Normal = nWorld_AtoB;
        m.Penetration = penetration;
        m.ContactPoint = contactWorld;

        m.Restitution = double.Min(sphereBody.Restitution, boxBody.Restitution);
        m.Friction = double.Sqrt(sphereBody.Friction * boxBody.Friction);
        FinalizeManifold(ref m);

        return true;

        static double Clamp(double x, double lo, double hi) => (x < lo) ? lo : (x > hi) ? hi : x;
    }

    private static bool BoxBox(RigidBody A, BoxShape a, RigidBody B, BoxShape b, out Manifold m)
    {
        m = new Manifold { A = A, B = B };

        // Orthonormal rotation matrices
        Matrix3 RA = Matrix3.FromQuaternion(A.Orientation);
        Matrix3 RB = Matrix3.FromQuaternion(B.Orientation);
        Matrix3 RAt = RA.Transposed();

        // R = RA^T * RB (B in A space)
        Matrix3 R = RAt * RB;

        // t = RA^T * (Bpos - Apos) (translation in A space)
        Vector3 t = RAt * (B.Position - A.Position);

        Vector3 aE = a.HalfExtents;
        Vector3 bE = b.HalfExtents;

        // Convenience accessors for R elements
        double R00 = R.M00, R01 = R.M01, R02 = R.M02;
        double R10 = R.M10, R11 = R.M11, R12 = R.M12;
        double R20 = R.M20, R21 = R.M21, R22 = R.M22;

        // AbsR with epsilon to avoid parallel axis issues
        const double eps = 1e-9;
        double AR00 = System.Math.Abs(R00) + eps, AR01 = System.Math.Abs(R01) + eps, AR02 = System.Math.Abs(R02) + eps;
        double AR10 = System.Math.Abs(R10) + eps, AR11 = System.Math.Abs(R11) + eps, AR12 = System.Math.Abs(R12) + eps;
        double AR20 = System.Math.Abs(R20) + eps, AR21 = System.Math.Abs(R21) + eps, AR22 = System.Math.Abs(R22) + eps;

        // Track best (minimum penetration) axis in WORLD space with A->B direction
        double bestPen = double.PositiveInfinity;
        Vector3 bestAxisWorld = Vector3.UnitY; // fallback

        void ConsiderAxisWorld(Vector3 axisWorld, double pen, double sign)
        {
            // axisWorld assumed normalized (or close enough)
            if (pen < bestPen)
            {
                bestPen = pen;
                bestAxisWorld = axisWorld * sign;
            }
        }

        // A's world axes (columns of RA)
        Vector3 Ax0 = new Vector3(RA.M00, RA.M10, RA.M20);
        Vector3 Ax1 = new Vector3(RA.M01, RA.M11, RA.M21);
        Vector3 Ax2 = new Vector3(RA.M02, RA.M12, RA.M22);

        // B's world axes (columns of RB)
        Vector3 Bx0 = new Vector3(RB.M00, RB.M10, RB.M20);
        Vector3 Bx1 = new Vector3(RB.M01, RB.M11, RB.M21);
        Vector3 Bx2 = new Vector3(RB.M02, RB.M12, RB.M22);

        Vector3 AtoB = B.Position - A.Position;

        // ---- 1) Axes A0, A1, A2 (in A space these are x,y,z)
        {
            double ra, rb, dist, pen;

            ra = aE.X;
            rb = bE.X * AR00 + bE.Y * AR01 + bE.Z * AR02;
            dist = System.Math.Abs(t.X);
            pen = (ra + rb) - dist;
            if (pen < 0) return false;
            ConsiderAxisWorld(Ax0, pen, (t.X >= 0 ? 1 : -1));

            ra = aE.Y;
            rb = bE.X * AR10 + bE.Y * AR11 + bE.Z * AR12;
            dist = System.Math.Abs(t.Y);
            pen = (ra + rb) - dist;
            if (pen < 0) return false;
            ConsiderAxisWorld(Ax1, pen, (t.Y >= 0 ? 1 : -1));

            ra = aE.Z;
            rb = bE.X * AR20 + bE.Y * AR21 + bE.Z * AR22;
            dist = System.Math.Abs(t.Z);
            pen = (ra + rb) - dist;
            if (pen < 0) return false;
            ConsiderAxisWorld(Ax2, pen, (t.Z >= 0 ? 1 : -1));
        }

        // ---- 2) Axes B0, B1, B2
        // t in B space = R^T * t
        Vector3 tB = new Vector3(
            t.X * R00 + t.Y * R10 + t.Z * R20,
            t.X * R01 + t.Y * R11 + t.Z * R21,
            t.X * R02 + t.Y * R12 + t.Z * R22
        );

        {
            double ra, rb, dist, pen;

            ra = aE.X * AR00 + aE.Y * AR10 + aE.Z * AR20;
            rb = bE.X;
            dist = System.Math.Abs(tB.X);
            pen = (ra + rb) - dist;
            if (pen < 0) return false;
            ConsiderAxisWorld(Bx0, pen, (tB.X >= 0 ? 1 : -1));

            ra = aE.X * AR01 + aE.Y * AR11 + aE.Z * AR21;
            rb = bE.Y;
            dist = System.Math.Abs(tB.Y);
            pen = (ra + rb) - dist;
            if (pen < 0) return false;
            ConsiderAxisWorld(Bx1, pen, (tB.Y >= 0 ? 1 : -1));

            ra = aE.X * AR02 + aE.Y * AR12 + aE.Z * AR22;
            rb = bE.Z;
            dist = System.Math.Abs(tB.Z);
            pen = (ra + rb) - dist;
            if (pen < 0) return false;
            ConsiderAxisWorld(Bx2, pen, (tB.Z >= 0 ? 1 : -1));
        }

        // ---- 3) Cross axes Ai x Bj (9 axes), explicit Ericson SAT formulas
        // Skip near-parallel axes (length ~ 0)

        // A0 x B0
        {
            double ra = aE.Y * AR20 + aE.Z * AR10;
            double rb = bE.Y * AR02 + bE.Z * AR01;
            double dist = System.Math.Abs(t.Z * R10 - t.Y * R20);
            double pen = (ra + rb) - dist;
            if (pen < 0) return false;

            Vector3 axis = Vector3.Cross(Ax0, Bx0);
            if (axis.LengthSquared() > 1e-18)
            {
                axis = axis.Normalized();
                ConsiderAxisWorld(axis, pen, Vector3.Dot(axis, AtoB) >= 0 ? 1 : -1);
            }
        }

        // A0 x B1
        {
            double ra = aE.Y * AR21 + aE.Z * AR11;
            double rb = bE.X * AR02 + bE.Z * AR00;
            double dist = System.Math.Abs(t.Z * R11 - t.Y * R21);
            double pen = (ra + rb) - dist;
            if (pen < 0) return false;

            Vector3 axis = Vector3.Cross(Ax0, Bx1);
            if (axis.LengthSquared() > 1e-18)
            {
                axis = axis.Normalized();
                ConsiderAxisWorld(axis, pen, Vector3.Dot(axis, AtoB) >= 0 ? 1 : -1);
            }
        }

        // A0 x B2
        {
            double ra = aE.Y * AR22 + aE.Z * AR12;
            double rb = bE.X * AR01 + bE.Y * AR00;
            double dist = System.Math.Abs(t.Z * R12 - t.Y * R22);
            double pen = (ra + rb) - dist;
            if (pen < 0) return false;

            Vector3 axis = Vector3.Cross(Ax0, Bx2);
            if (axis.LengthSquared() > 1e-18)
            {
                axis = axis.Normalized();
                ConsiderAxisWorld(axis, pen, Vector3.Dot(axis, AtoB) >= 0 ? 1 : -1);
            }
        }

        // A1 x B0
        {
            double ra = aE.X * AR20 + aE.Z * AR00;
            double rb = bE.Y * AR12 + bE.Z * AR11;
            double dist = System.Math.Abs(t.X * R20 - t.Z * R00);
            double pen = (ra + rb) - dist;
            if (pen < 0) return false;

            Vector3 axis = Vector3.Cross(Ax1, Bx0);
            if (axis.LengthSquared() > 1e-18)
            {
                axis = axis.Normalized();
                ConsiderAxisWorld(axis, pen, Vector3.Dot(axis, AtoB) >= 0 ? 1 : -1);
            }
        }

        // A1 x B1
        {
            double ra = aE.X * AR21 + aE.Z * AR01;
            double rb = bE.X * AR12 + bE.Z * AR10;
            double dist = System.Math.Abs(t.X * R21 - t.Z * R01);
            double pen = (ra + rb) - dist;
            if (pen < 0) return false;

            Vector3 axis = Vector3.Cross(Ax1, Bx1);
            if (axis.LengthSquared() > 1e-18)
            {
                axis = axis.Normalized();
                ConsiderAxisWorld(axis, pen, Vector3.Dot(axis, AtoB) >= 0 ? 1 : -1);
            }
        }

        // A1 x B2
        {
            double ra = aE.X * AR22 + aE.Z * AR02;
            double rb = bE.X * AR11 + bE.Y * AR10;
            double dist = System.Math.Abs(t.X * R22 - t.Z * R02);
            double pen = (ra + rb) - dist;
            if (pen < 0) return false;

            Vector3 axis = Vector3.Cross(Ax1, Bx2);
            if (axis.LengthSquared() > 1e-18)
            {
                axis = axis.Normalized();
                ConsiderAxisWorld(axis, pen, Vector3.Dot(axis, AtoB) >= 0 ? 1 : -1);
            }
        }

        // A2 x B0
        {
            double ra = aE.X * AR10 + aE.Y * AR00;
            double rb = bE.Y * AR22 + bE.Z * AR21;
            double dist = System.Math.Abs(t.Y * R00 - t.X * R10);
            double pen = (ra + rb) - dist;
            if (pen < 0) return false;

            Vector3 axis = Vector3.Cross(Ax2, Bx0);
            if (axis.LengthSquared() > 1e-18)
            {
                axis = axis.Normalized();
                ConsiderAxisWorld(axis, pen, Vector3.Dot(axis, AtoB) >= 0 ? 1 : -1);
            }
        }

        // A2 x B1
        {
            double ra = aE.X * AR11 + aE.Y * AR01;
            double rb = bE.X * AR22 + bE.Z * AR20;
            double dist = System.Math.Abs(t.Y * R01 - t.X * R11);
            double pen = (ra + rb) - dist;
            if (pen < 0) return false;

            Vector3 axis = Vector3.Cross(Ax2, Bx1);
            if (axis.LengthSquared() > 1e-18)
            {
                axis = axis.Normalized();
                ConsiderAxisWorld(axis, pen, Vector3.Dot(axis, AtoB) >= 0 ? 1 : -1);
            }
        }

        // A2 x B2
        {
            double ra = aE.X * AR12 + aE.Y * AR02;
            double rb = bE.X * AR21 + bE.Y * AR20;
            double dist = System.Math.Abs(t.Y * R02 - t.X * R12);
            double pen = (ra + rb) - dist;
            if (pen < 0) return false;

            Vector3 axis = Vector3.Cross(Ax2, Bx2);
            if (axis.LengthSquared() > 1e-18)
            {
                axis = axis.Normalized();
                ConsiderAxisWorld(axis, pen, Vector3.Dot(axis, AtoB) >= 0 ? 1 : -1);
            }
        }

        // Overlap confirmed
        m.Normal = bestAxisWorld.Normalized();
        m.Penetration = bestPen;

        // Approx contact point: closest point from B center to A (in A space), transformed back
        Vector3 pB_inA = t;
        Vector3 clamped = new Vector3(
            Clamp(pB_inA.X, -aE.X, aE.X),
            Clamp(pB_inA.Y, -aE.Y, aE.Y),
            Clamp(pB_inA.Z, -aE.Z, aE.Z)
        );
        m.ContactPoint = A.Position + (RA * clamped);

        m.Restitution = System.Math.Min(A.Restitution, B.Restitution);
        m.Friction = System.Math.Sqrt(A.Friction * B.Friction);
        FinalizeManifold(ref m);

        return true;

        static double Clamp(double x, double lo, double hi) => x < lo ? lo : (x > hi ? hi : x);
    }

    private static ulong MakeKey(RigidBody a, RigidBody b, Vector3 cp)
    {
        int idA = a.Id, idB = b.Id;
        if (idA > idB) (idA, idB) = (idB, idA);

        // Quantize contact point so key is stable across frames
        long qx = (long)System.Math.Round(cp.X * 1000.0);
        long qy = (long)System.Math.Round(cp.Y * 1000.0);
        long qz = (long)System.Math.Round(cp.Z * 1000.0);

        unchecked
        {
            ulong h = 1469598103934665603UL; // FNV-1a offset
            h = (h ^ (ulong)idA) * 1099511628211UL;
            h = (h ^ (ulong)idB) * 1099511628211UL;
            h = (h ^ (ulong)qx) * 1099511628211UL;
            h = (h ^ (ulong)qy) * 1099511628211UL;
            h = (h ^ (ulong)qz) * 1099511628211UL;
            return h;
        }
    }

    private static void FinalizeManifold(ref Manifold m)
    {
        m.ContactKey = MakeKey(m.A, m.B, m.ContactPoint);
    }

}