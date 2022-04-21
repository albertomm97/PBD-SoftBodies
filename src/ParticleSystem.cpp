#include "ParticleSystem.h"

#include <Gfx/Gfx.h>

namespace utad
{
ParticleSystem::~ParticleSystem()
{
    for (auto particle : m_Particles) {
        delete particle;
    }
}

Particle *ParticleSystem::CreateParticle(const uint body, const float mass)
{
    auto particle = new Particle(body, mass);
    m_Particles.push_back(particle);
    return particle;
}

void ParticleSystem::Draw(const bool drawSurf, const bool drawWire)
{
    for (auto particle : m_Particles) {
        const uint body = particle->m_Body;

        Gfx::setColor(particle->m_Color);
        Gfx::drawSphere(particle->m_X, m_Radius, drawSurf, drawWire);
    }
}

void ParticleSystem::ClearForces()
{
    for (auto particle : m_Particles) {
        particle->m_F = Vector3f(0.0f, 0.0f, 0.0f);
    }
}
}  // namespace utad