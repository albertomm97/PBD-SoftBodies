#include "PbdSolver.h"

namespace utad
{
PbdSolver::PbdSolver(ParticleSystem *particleSystem)
    : m_ParticleSystem(particleSystem)
{
}

void PbdSolver::SolveConstraints()
{
    InitConstraints();

    for (uint iter = 0; iter < m_Iterations; ++iter) {
        ProjectConstraints();
    }
}

void PbdSolver::Integrate(const float dt)
{
    // TODO: Integrate particles
    auto &particles = m_ParticleSystem->m_Particles;
    for (auto *particle : particles) {
        Vector3f acceleration = particle->m_F * particle->m_MassInv;
        particle->m_V += acceleration * dt;
        particle->m_P = (particle->m_X + particle->m_V * dt);
    }

}

void PbdSolver::Update(const float dt)
{
    // TODO: Update particles state after constraints projection
    auto &particles = m_ParticleSystem->m_Particles;
    const float invDt = 1 / dt;
    for (auto *particle : particles) {
        particle->m_V = (particle->m_P - particle->m_X) * invDt;
        particle->m_X = particle->m_P;
    }
}

void PbdSolver::InitConstraints()
{
    const size_t n = m_FixedConstraints.size() + m_CollisionConstraints.size();

    m_Constraints.clear();
    m_Constraints.reserve(n);

    for (auto constraint : m_FixedConstraints) {
        m_Constraints.push_back(constraint);
    }
    for (auto constraint : m_CollisionConstraints) {
        m_Constraints.push_back(constraint);
    }
}

void PbdSolver::ProjectConstraints()
{
    for (auto constraint : m_Constraints) {
        constraint->Project();
    }
}
}