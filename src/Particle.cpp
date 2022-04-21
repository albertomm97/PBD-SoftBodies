#include "Particle.h"

namespace utad
{

uint Particle::id_gen = 0;

Particle::Particle(uint body, float mass)
    : m_Body(body)
    , m_Mass(mass)
    , m_Next(nullptr)
{
    m_X = Vector3f(0.0F, 0.0F, 0.0F);
    m_V = Vector3f(0.0F, 0.0F, 0.0F);
    m_F = Vector3f(0.0F, 0.0F, 0.0F);

    m_MassInv = 1.0F / m_Mass;

    id = id_gen;
    id_gen += 1;
}
}  // namespace utad