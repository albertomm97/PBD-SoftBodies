#include "ParticleParticleCollisionConstraint.h"

#include <Gfx/Gfx.h>
#include <iostream>
namespace utad
{
ParticleParticleCollisionConstraint::ParticleParticleCollisionConstraint(Particle *p1, Particle *p2, const float radius)
    : m_P1(p1)
    , m_P2(p2)
    , m_Diameter(radius * 2.0F)
{
}

void ParticleParticleCollisionConstraint::Draw()
{
}

void ParticleParticleCollisionConstraint::Project()
{
    // TODO: Apply constraint projection
    // NOTE: C = ||P1 - P2|| - r * 2
    // NOTE: Use the mass dependent formulation
    // NOTE: Compute m_Lambda first
    // NOTE: Collision constraints are inequality constraints, return without projection if m_Lambda >= 0
    Vector3f p1p2Vector = m_P1->m_P - m_P2->m_P;
    float p1p2Distance = p1p2Vector.norm();  // n
    Vector3f p1p2Direction = p1p2Vector / p1p2Distance;

    float constraint = p1p2Distance - m_Diameter;
  
    m_Lambda = constraint / (m_P1->m_MassInv + m_P2->m_MassInv);
    if (m_Lambda >= 0.0f)  // In this case the particles are far enough so they are not colliding
        return;
    
    //std::cout << "Procesamos correcion\n";
    Vector3f deltap1 = -m_Lambda * m_P1->m_MassInv * p1p2Direction;
    m_P1->m_P += deltap1;

    Vector3f deltap2 = m_Lambda * m_P2->m_MassInv * p1p2Direction;
    m_P2->m_P += deltap2;
}
}  // namespace utad