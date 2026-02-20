using System;
using OpenTK.Graphics.OpenGL4;
namespace Graphics;



public sealed class Mesh : IDisposable
{
    private readonly int _vao;
    private readonly int _vbo;
    private readonly int _ebo;
    private readonly int _indexCount;

    public Mesh(float[] interleavedPosNormal, uint[] indices)
    {
        _indexCount = indices.Length;

        _vao = GL.GenVertexArray();
        _vbo = GL.GenBuffer();
        _ebo = GL.GenBuffer();

        GL.BindVertexArray(_vao);

        GL.BindBuffer(BufferTarget.ArrayBuffer, _vbo);
        GL.BufferData(BufferTarget.ArrayBuffer, interleavedPosNormal.Length * sizeof(float), interleavedPosNormal, BufferUsageHint.StaticDraw);

        GL.BindBuffer(BufferTarget.ElementArrayBuffer, _ebo);
        GL.BufferData(BufferTarget.ElementArrayBuffer, indices.Length * sizeof(uint), indices, BufferUsageHint.StaticDraw);

        // layout: vec3 pos, vec3 normal
        // layout: vec3 pos, vec3 normal, vec2 uv
        int stride = 8 * sizeof(float);

        GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, stride, 0);
        GL.EnableVertexAttribArray(0);

        GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, stride, 3 * sizeof(float));
        GL.EnableVertexAttribArray(1);

        GL.VertexAttribPointer(2, 2, VertexAttribPointerType.Float, false, stride, 6 * sizeof(float));
        GL.EnableVertexAttribArray(2);

        GL.BindVertexArray(0);
    }

    public void Draw(PrimitiveType primitive)
    {
        GL.BindVertexArray(_vao);
        GL.DrawElements(primitive, _indexCount, DrawElementsType.UnsignedInt, 0);
    }

    public void DrawRange(PrimitiveType primitive, int indexStart, int indexCount)
    {
        GL.BindVertexArray(_vao);
        GL.DrawElements(primitive, indexCount, DrawElementsType.UnsignedInt, indexStart * sizeof(uint));
    }

    public void Dispose()
    {
        GL.DeleteBuffer(_ebo);
        GL.DeleteBuffer(_vbo);
        GL.DeleteVertexArray(_vao);
    }
}

