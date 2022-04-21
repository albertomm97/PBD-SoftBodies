#ifndef PBD_SOLVER_H
#define PBD_SOLVER_H

#include <Constraint.h>
#include <ParticleSystem.h>

#include <vector>

namespace utad
{
class PbdSolver
{
public:
    PbdSolver(ParticleSystem *particleSystem);

    void Integrate(const float dt);
    void SolveConstraints();
    void Update(const float dt);

    uint m_Iterations{10};
    std::vector<Constraint *> m_FixedConstraints;
    std::vector<Constraint *> m_CollisionConstraints;

private:
    void InitConstraints();
    void ProjectConstraints();

    ParticleSystem *m_ParticleSystem;

    std::vector<Constraint *> m_Constraints;
};
}  // namespace utad

#endif