
using Physics.Math;

namespace Physics.Collision;

public interface IShape
{
    Matrix3 ComputeInvInertiaLocal(double mass);
}
