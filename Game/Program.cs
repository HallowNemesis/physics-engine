using OpenTK.Windowing.Desktop;
using OpenTK.Mathematics;
namespace Game;

internal static class Program
{
    public static void Main()
    {
        var native = new NativeWindowSettings
        {
            Title = "Physics Debug Viewer (OpenTK)",
            ClientSize = new Vector2i(1280, 720),
            APIVersion = new Version(3, 3),
            Flags = OpenTK.Windowing.Common.ContextFlags.ForwardCompatible
        };

        using var window = new PhysicsViewer(GameWindowSettings.Default, native);
        window.Run();
    }
}
