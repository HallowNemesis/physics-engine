using OpenTK.Graphics.OpenGL4;
using StbImageSharp;

namespace Graphics;

public sealed class Texture2D : IDisposable
{
    public int Handle { get; }

    public Texture2D(byte[] encodedImageBytes, bool srgb)
    {
        var img = ImageResult.FromMemory(encodedImageBytes, ColorComponents.RedGreenBlueAlpha);

        Handle = GL.GenTexture();
        GL.BindTexture(TextureTarget.Texture2D, Handle);

        var internalFormat = srgb ? PixelInternalFormat.Srgb8Alpha8 : PixelInternalFormat.Rgba8;

        GL.TexImage2D(TextureTarget.Texture2D, 0, internalFormat, img.Width, img.Height, 0,
            PixelFormat.Rgba, PixelType.UnsignedByte, img.Data);

        GL.GenerateMipmap(GenerateMipmapTarget.Texture2D);

        GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.LinearMipmapLinear);
        GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Linear);
        GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapS, (int)TextureWrapMode.Repeat);
        GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapT, (int)TextureWrapMode.Repeat);

        GL.BindTexture(TextureTarget.Texture2D, 0);
    }

    public void Bind(TextureUnit unit)
    {
        GL.ActiveTexture(unit);
        GL.BindTexture(TextureTarget.Texture2D, Handle);
    }

    public void Dispose() => GL.DeleteTexture(Handle);
}
