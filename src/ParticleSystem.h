#ifndef PARTICLE_SYSTEM_H
#define PARTICLE_SYSTEM_H

#include <Particle.h>
#include <vector>

namespace utad
{
class ParticleSystem
{
public:
    ~ParticleSystem();

    Particle *CreateParticle(const uint body, const float mass);

    void Draw(const bool drawSurf, const bool drawWire);

    void ClearForces();

    float m_Radius{0.05F};
    std::vector<Particle *> m_Particles;
};
}  // namespace utad

#endif