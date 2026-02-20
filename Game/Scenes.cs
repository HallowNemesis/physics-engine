using Assets;
using Game;
using Physics.Collision;
using Physics.Dynamics;
using Physics.Math;
using Scene;
using System.Text.RegularExpressions;

public static class Scenes
{
    public static (World world, RigidBody player, Action<PhysicsWorldAdapter> bindModels) CreateSphereDropScene()
    {
        var w = new World();


        var platform = new RigidBody(
    new BoxShape(new Vector3(6, 0.5, 6)),
    mass: 0f,
    position: new Vector3(0, -0.5, 0)
);
        platform.Restitution = 0.0;
        platform.Friction = 0.9;
        w.Bodies.Add(platform);

        // A few dynamic spheres
//        for (int i = 0; i < 5; i++)
//        {
//            var b = new RigidBody(new SphereShape(0.35), 1f, new Vector3(-1.5 + i * 0.6, 3 + i * 0.5, 0));
//            b.Restitution = 0.1;
//            b.Friction = 0.6;
//            w.Bodies.Add(b);
//        }

//        // One heavier sphere
//        var heavy = new RigidBody(new SphereShape(radius: 0.5), mass: 5f, position: new Vector3(2.0, 6.0, 0));
//        heavy.Restitution = 0.05;
//        heavy.Friction = 0.8;
//        w.Bodies.Add(heavy);

//        var joint = new DistanceJoint(
//    platform, heavy,
//    localAnchorA: new Vector3(0, 4.0, 0),   // in ground local space
//    localAnchorB: new Vector3(0, 0.0, 0),   // center of sphere
//    targetLength: 1.5
//);

        //w.Constraints.Add(joint);

        var path = Path.Combine("Assets", "model.glb");
        var renderScale = new Vector3(5, 5, 5);

        var bounds = GlbLoader.GetLocalBounds(path);
        var pivot = new Physics.Math.Vector3(-bounds.Center.X, -bounds.Center.Y, -bounds.Center.Z);

        // one shared load (CPU)
        var asset = AssetCache.Get(path);
        var localVerts = SceneMeshUtil.ExtractTransformedPositions(asset.Cpu, renderScale, pivot);

        // place rb so mesh bottom touches y=0
        var minY = SceneMeshUtil.MinY(localVerts);
        var buildingPos = new Physics.Math.Vector3(0, -minY, 0);

        var building = new RigidBody(
            shape: new StaticMeshShape(localVerts, asset.Cpu.Indices),
            mass: 0f,
            position: buildingPos
        );
        w.Bodies.Add(building);


        //var Hornet = new RigidBody(
        //    new BoxShape(new Vector3(0.5, 0.5, 0.5)),
        //    mass: 0d,
        //    position: Vector3.UnitY
        //);
        //w.Bodies.Add(Hornet);



        var playerRadius = 0.45;
        var player = new RigidBody(
            shape: new SphereShape(playerRadius),
            mass: 80.0,
            position: new Vector3(0, playerRadius + 10, 2)
        );
        player.Restitution = 0.0;
        player.Friction = 0.2;
        w.Bodies.Add(player);

        void Bind(PhysicsWorldAdapter adapter)
        {
            adapter.AttachModel(
                    building,
                    path,
                    renderScale,
                    pivot
                );
            //adapter.AttachModel(Hornet, Path.Combine("Assets", "HORNET.glb"));
        }

       

        return (w, player, Bind);
    }

}
