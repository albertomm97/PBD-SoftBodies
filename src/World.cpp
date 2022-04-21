#include "World.h"

#include <Gfx/Gfx.h>
#include <ParticleParticleCollisionConstraint.h>
#include <ParticleSphereCollisionConstraint.h>

namespace utad
{
static const Vector3f gSphereOrigin{0.0F, 2.0F, 0.0};
static const float gSphereRadius{2.0F};

World::World()
    : m_NeighborsSearch(&m_ParticleSystem)
    , m_Solver(&m_ParticleSystem)
{
}

World::~World()
{
    ClearCollisionConstraints();

    for (auto sb : m_SoftBodies) {
        delete sb;
    }
}

SoftBody *World::CreateSoftBody()
{
    auto sb = new SoftBody(&m_ParticleSystem);
    m_SoftBodies.push_back(sb);

    m_FixedConstraintsDirty = true;

    return sb;
}

void World::Simulate(const float dt)
{
    // Clear collision constraints
    ClearCollisionConstraints();

    // update fixed constraints
    UpdateFixedConstraints();

    // Clear forces
    ClearForces();

    // Compute external forces applied
    ComputeExternalForces();

    // PBD solver integrate
    m_Solver.Integrate(dt);

    // Generate collision constraints
    GenerateCollisionConstraints();

    // PBD solver constraint projection
    m_Solver.SolveConstraints();

    /*std::printf("Particle-Particle: %zd, Particle-Sphere: %zd\n",
                m_ParticleParticleCollisionConstraints.size(),
                m_ParticleSphereCollisionConstraints.size());*/

    // PBD solver update
    m_Solver.Update(dt);

    // End step
    EndStep();
}

void World::Draw()
{
    // Draw bounds
    Gfx::setColor(Vector3f(0.0F, 0.0F, 0.0F));
    Gfx::drawSphere(gSphereOrigin, gSphereRadius, false, true);

    // Draw particles
    m_ParticleSystem.Draw(true, true);
}

void World::ClearCollisionConstraints()
{
    for (auto constraint : m_ParticleParticleCollisionConstraints) {
        m_ParticleParticleCollisionConstraintsPool.deleteElement(constraint);
    }
    for (auto constraint : m_ParticleSphereCollisionConstraints) {
        m_ParticleSphereCollisionConstraintsPool.deleteElement(constraint);
    }
    m_ParticleParticleCollisionConstraints.clear();
    m_ParticleSphereCollisionConstraints.clear();

    m_Solver.m_CollisionConstraints.clear();
}

void World::UpdateFixedConstraints()
{
    if (!m_FixedConstraintsDirty) {
        return;
    }

    const uint iterations = m_Solver.m_Iterations;

    // Update fixed constraints
    auto &fixedConstraints = m_Solver.m_FixedConstraints;

    fixedConstraints.clear();

    for (auto sb : m_SoftBodies) {
        for (auto constraint : sb->m_Constraints) {
            constraint->UpdateStiffnessPerIteration(iterations);

            fixedConstraints.push_back(constraint);
        }
    }

    m_FixedConstraintsDirty = false;
}

void World::ClearForces()
{
    m_ParticleSystem.ClearForces();
}

void World::ComputeExternalForces()
{
    // TODO: Add gravity force to particles
    auto &particles = m_ParticleSystem.m_Particles;
    for (auto *particle : particles) {
        particle->m_F += m_gravity;
    }
}

void World::GenerateCollisionConstraints()
{
    m_NeighborsSearch.Update();

    auto &particles = m_ParticleSystem.m_Particles;
    auto radius = m_ParticleSystem.m_Radius;

    
    // Particle sphere constraints
    for (auto* particle : particles) {
        const float particleSphereDistance = (gSphereOrigin - particle->m_X).norm();
        if (particleSphereDistance > (gSphereRadius - radius)) {
            auto* C = m_ParticleSphereCollisionConstraintsPool.newElement(particle, radius);
            m_ParticleSphereCollisionConstraints.push_back(C);
        }
    }
    
    // Particle particle constraints
    for (auto *particle : particles) {
        std::vector<Particle *> neighbors;
        m_NeighborsSearch.QueryNeighbors(particle, neighbors);
        for (auto *neighbor : neighbors) {
            auto *c = m_ParticleParticleCollisionConstraintsPool.newElement(particle, neighbor, radius);
            m_ParticleParticleCollisionConstraints.push_back(c);
        }
    }

    // Add all collision constraints to the solver
    for (auto C : m_ParticleSphereCollisionConstraints) {
        m_Solver.m_CollisionConstraints.push_back(C);
    }
    for (auto C : m_ParticleParticleCollisionConstraints) {
        m_Solver.m_CollisionConstraints.push_back(C);
    }

    
}

void World::EndStep()
{
    for (auto sb : m_SoftBodies) {
        sb->EndStep();
    }
}
}  // namespace utad
