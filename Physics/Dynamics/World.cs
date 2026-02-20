using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using Physics.Collision;
using Physics.Math;

namespace Physics.Dynamics;

public sealed class World
{
    public Vector3 Gravity = new(0, -9.81d, 0);

    // Damping
    public double LinearAirDrag = 0.2;      // 1/s
    public double AngularAirDrag = 0.1;     // 1/s
    public double RollingLinearDrag = 2.0;  // 1/s (only when grounded)
    public double RollingAngularDrag = 4.0; // 1/s (only when grounded)

    // Sleeping
    public bool EnableSleeping = true;
    public double SleepLinearThreshold = 0.05;   // m/s
    public double SleepAngularThreshold = 0.10;  // rad/s
    public double TimeToSleep = 0.75;            // seconds

    public readonly List<RigidBody> Bodies = new();
    public readonly HashSet<RigidBody> Grounded = new();

    public readonly List<IConstraint> Constraints = new(); // joints live here
    public int VelocityIterations { get; set; } = 10;

    public bool IsGrounded(RigidBody b) => Grounded.Contains(b);

    // Contact reuse + warmstart cache
    private readonly List<Manifold> _contacts = new(1024);

    private struct ManifoldCacheEntry
    {
        public double Jn, Jt;
        public uint LastSeen;
    }

    private readonly Dictionary<ulong, ManifoldCacheEntry> _contactCache = new();
    private uint _stepId;

    public void Step(double dt)
    {
        if (dt <= 0.0) return;

        // ---- Forces -> Velocities
        foreach (var body in Bodies)
        {
            if (body.IsStatic) continue;
            if (EnableSleeping && !body.IsAwake) continue;

            Vector3 accel = Gravity + body.Force * body.InvMass;
            body.LinearVelocity += accel * dt;

            body.UpdateInvInertiaWorld();
            Vector3 angAccel = body.InvInertiaWorld * body.Torque;
            body.AngularVelocity += angAccel * dt;
        }

        // ---- Contacts (NO allocations)
        _stepId++;
        CollisionDetection.BuildContacts(Bodies, _contacts); // must be a void method that fills out list

        // ---- Warm start from cache (IMPORTANT: write back to list if Manifold is a struct)
        for (int i = 0; i < _contacts.Count; i++)
        {
            var c = _contacts[i];

            ref var entry = ref CollectionsMarshal.GetValueRefOrAddDefault(
                _contactCache, c.ContactKey, out bool exists);

            if (exists)
            {
                c.AccumulatedNormalImpulse = entry.Jn;
                c.AccumulatedTangentImpulse = entry.Jt;
            }
            else
            {
                c.AccumulatedNormalImpulse = 0.0;
                c.AccumulatedTangentImpulse = 0.0;
            }

            entry.LastSeen = _stepId;

            // If Manifold is a struct, this is required. If it's a class, harmless.
            _contacts[i] = c;
        }

        // ---- PreStep constraints
        foreach (var con in Constraints)
            con.PreStep(dt);

        // ---- Solve contacts + constraints
        Solver.Solve(_contacts, Constraints, VelocityIterations, dt);

        // ---- Store solved impulses back to cache (no Clear)
        for (int i = 0; i < _contacts.Count; i++)
        {
            var c = _contacts[i];

            ref var entry = ref CollectionsMarshal.GetValueRefOrAddDefault(
                _contactCache, c.ContactKey, out _);

            entry.Jn = c.AccumulatedNormalImpulse;
            entry.Jt = c.AccumulatedTangentImpulse;
            entry.LastSeen = _stepId;
        }

        // ---- Prune stale cache entries occasionally (avoid per-step Clear)
        if ((_stepId & 63) == 0) // every 64 steps
        {
            var keys = _contactCache.Keys.ToArray();
            for (int k = 0; k < keys.Length; k++)
            {
                ulong key = keys[k];
                if (_contactCache[key].LastSeen != _stepId)
                    _contactCache.Remove(key);
            }
        }

        // ---- Grounded set heuristic
        Grounded.Clear();
        for (int i = 0; i < _contacts.Count; i++)
        {
            var c = _contacts[i];
            var n = c.Normal;

            if (c.A.IsStatic && !c.B.IsStatic && System.Math.Abs(n.Y) > 0.5) Grounded.Add(c.B);
            if (c.B.IsStatic && !c.A.IsStatic && System.Math.Abs(n.Y) > 0.5) Grounded.Add(c.A);
        }

        // ---- Air drag
        double linDamp = System.Math.Exp(-LinearAirDrag * dt);
        double angDamp = System.Math.Exp(-AngularAirDrag * dt);

        foreach (var body in Bodies)
        {
            if (body.IsStatic) continue;
            if (EnableSleeping && !body.IsAwake) continue;

            body.LinearVelocity = body.LinearVelocity * linDamp;
            body.AngularVelocity = body.AngularVelocity * angDamp;
        }

        // ---- Rolling resistance when grounded
        if (Grounded.Count > 0)
        {
            double rollLin = System.Math.Exp(-RollingLinearDrag * dt);
            double rollAng = System.Math.Exp(-RollingAngularDrag * dt);

            foreach (var body in Grounded)
            {
                if (body.IsStatic) continue;
                if (EnableSleeping && !body.IsAwake) continue;

                body.LinearVelocity = new Vector3(
                    body.LinearVelocity.X * rollLin,
                    body.LinearVelocity.Y,
                    body.LinearVelocity.Z * rollLin
                );

                body.AngularVelocity = body.AngularVelocity * rollAng;
            }
        }

        // ---- Integrate
        foreach (var body in Bodies)
        {
            if (body.IsStatic) continue;
            if (EnableSleeping && !body.IsAwake) continue;

            body.Position += body.LinearVelocity * dt;
            body.Orientation = Quaternion.Integrate(body.Orientation, body.AngularVelocity, dt);
            body.Orientation = body.Orientation.Normalized();
            body.UpdateInvInertiaWorld();
        }

        // ---- Positional correction
        Solver.PositionalCorrection(_contacts);

        // ---- Sleep update
        if (EnableSleeping)
        {
            double vThresh2 = SleepLinearThreshold * SleepLinearThreshold;
            double wThresh2 = SleepAngularThreshold * SleepAngularThreshold;

            foreach (var body in Bodies)
            {
                if (body.IsStatic) continue;

                if (body.Force.LengthSquared() > 1e-12 || body.Torque.LengthSquared() > 1e-12)
                {
                    body.Wake();
                    continue;
                }

                if (!body.IsAwake) continue;

                double v2 = body.LinearVelocity.LengthSquared();
                double w2 = body.AngularVelocity.LengthSquared();

                if (v2 < vThresh2 && w2 < wThresh2)
                {
                    body.SleepTimer += dt;
                    if (body.SleepTimer >= TimeToSleep)
                    {
                        body.IsAwake = false;
                        body.SleepTimer = 0.0;
                        body.LinearVelocity = Vector3.Zero;
                        body.AngularVelocity = Vector3.Zero;
                    }
                }
                else
                {
                    body.SleepTimer = 0.0;
                }
            }
        }

        // ---- Clear accumulators
        foreach (var body in Bodies)
            body.ClearAccumulators();
    }
}
