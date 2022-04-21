#ifndef PARTICLE_SPHERE_COLLISION_CONSTRAINT_H
#define PARTICLE_SPHERE_COLLISION_CONSTRAINT_H

#include <Constraint.h>
#include <Particle.h>

namespace utad
{
class ParticleSphereCollisionConstraint : public Constraint
{
public:
    ParticleSphereCollisionConstraint(Particle *p1, const float radius);

    void Draw();

    void Project();

    Particle *m_P1;
    float m_Radius;
};
}  // namespace utad

#endif