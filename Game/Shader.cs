
using System;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;

namespace Assets;


public sealed class Shader : IDisposable
{
    public int Handle { get; }

    public Shader(string vertexSrc, string fragmentSrc)
    {
        int v = GL.CreateShader(ShaderType.VertexShader);
        GL.ShaderSource(v, vertexSrc);
        GL.CompileShader(v);
        ThrowIfShaderFailed(v, "VERTEX");

        int f = GL.CreateShader(ShaderType.FragmentShader);
        GL.ShaderSource(f, fragmentSrc);
        GL.CompileShader(f);
        ThrowIfShaderFailed(f, "FRAGMENT");

        Handle = GL.CreateProgram();
        GL.AttachShader(Handle, v);
        GL.AttachShader(Handle, f);
        GL.LinkProgram(Handle);

        GL.GetProgram(Handle, GetProgramParameterName.LinkStatus, out int ok);
        if (ok == 0)
            throw new Exception("Program link failed: " + GL.GetProgramInfoLog(Handle));

        GL.DetachShader(Handle, v);
        GL.DetachShader(Handle, f);
        GL.DeleteShader(v);
        GL.DeleteShader(f);
    }

    public void Use() => GL.UseProgram(Handle);

    public void SetMat4(string name, Matrix4 mat)
    {
        int loc = GL.GetUniformLocation(Handle, name);
        GL.UniformMatrix4(loc, false, ref mat);
    }

    public void SetVec3(string name, Vector3 v)
    {
        int loc = GL.GetUniformLocation(Handle, name);
        GL.Uniform3(loc, v);
    }

    public void SetInt(string name, int v)
    {
        int loc = GL.GetUniformLocation(Handle, name);
        GL.Uniform1(loc, v);
    }
    public void SetBool(string name, bool v)
    {
        int loc = GL.GetUniformLocation(Handle, name);
        GL.Uniform1(loc, v ? 1 : 0);
    }


    private static void ThrowIfShaderFailed(int shader, string stage)
    {
        GL.GetShader(shader, ShaderParameter.CompileStatus, out int ok);
        if (ok == 0)
            throw new Exception($"{stage} compile failed: {GL.GetShaderInfoLog(shader)}");
    }

    public void Dispose()
    {
        GL.DeleteProgram(Handle);
    }
}
