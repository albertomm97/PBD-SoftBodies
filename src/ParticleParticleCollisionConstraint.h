#ifndef PARTICLE_PARTICLE_COLLISION_CONSTRAINT_H
#define PARTICLE_PARTICLE_COLLISION_CONSTRAINT_H

#include <Constraint.h>
#include <Particle.h>

namespace utad
{
class ParticleParticleCollisionConstraint : public Constraint
{
public:
    ParticleParticleCollisionConstraint(Particle *p1, Particle *p2, const float radius);

    void Draw();

    void Project();

    Particle *m_P1;
    Particle *m_P2;
    float m_Diameter;
};
}  // namespace utad

#endif