#ifndef DISTANCE_CONSTRAINT_H
#define DISTANCE_CONSTRAINT_H

#include <Constraint.h>
#include <Particle.h>

namespace utad
{
class DistanceConstraint : public Constraint
{
public:
    DistanceConstraint(Particle *p1, Particle *p2, const float restLength);

    void Draw();

    void Project();

    Particle *m_P1;
    Particle *m_P2;
    float m_RestLength;
};
}  // namespace utad

#endif