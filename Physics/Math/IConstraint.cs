using Physics.Math;

namespace Physics.Dynamics;

public interface IConstraint
{
    // Called once per step to compute cached values (effective mass, bias, etc.)
    void PreStep(double dt);

    // Apply cached accumulated impulses from last step
    void WarmStart();

    // One sequential-impulse iteration
    void Solve();
}
