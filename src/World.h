#ifndef WORLD_H
#define WORLD_H

#include <Containers/MemoryPool.h>
#include <ParticleParticleCollisionConstraint.h>
#include <ParticleSphereCollisionConstraint.h>
#include <ParticleSystem.h>
#include <PbdSolver.h>
#include <NeighborsSearch.h>
#include <SoftBody.h>

#include <vector>

namespace utad
{
class World
{
public:
    World();
    ~World();

    SoftBody *CreateSoftBody();

    void Draw();

    void Simulate(const float dt);

    Vector3f m_gravity{0.0F, -2.0F, 0.0F};
    float m_radius{0.01F};

    ParticleSystem m_ParticleSystem;

    std::vector<SoftBody *> m_SoftBodies;

    NeighborsSearch m_NeighborsSearch;
    PbdSolver m_Solver;

private:
    void ClearCollisionConstraints();
    void UpdateFixedConstraints();

    void ClearForces();
    void ComputeExternalForces();

    void GenerateCollisionConstraints();

    void EndStep();

    bool m_FixedConstraintsDirty{true};

    std::vector<ParticleSphereCollisionConstraint *> m_ParticleSphereCollisionConstraints;
    std::vector<ParticleParticleCollisionConstraint *> m_ParticleParticleCollisionConstraints;
    MemoryPool<ParticleSphereCollisionConstraint> m_ParticleSphereCollisionConstraintsPool;
    MemoryPool<ParticleParticleCollisionConstraint> m_ParticleParticleCollisionConstraintsPool;
};
}  // namespace utad

#endif