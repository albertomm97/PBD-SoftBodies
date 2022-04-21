#include "ParticleSphereCollisionConstraint.h"

#include <Gfx/Gfx.h>
#include <iostream>
namespace utad
{
static const Vector3f gSphereOrigin{0.0F, 2.0F, 0.0};
static const float gSphereRadius{2.0F};

ParticleSphereCollisionConstraint::ParticleSphereCollisionConstraint(Particle *p1, const float radius)
    : m_P1(p1)
    , m_Radius(radius)
{
}

void ParticleSphereCollisionConstraint::Draw()
{
}

void ParticleSphereCollisionConstraint::Project()
{
    // TODO: Apply constraint projection
    // NOTE: C = ||P - P0|| - r
    // NOTE: Compute m_Lambda first
    // NOTE: Collision constraints are inequality constraints, return without projection if m_Lambda >= 0
    
    Vector3f particleSphereVector = m_P1->m_P - gSphereOrigin;
    float particleSphereDistance = particleSphereVector.norm();
    Vector3f particleSphereDirection = particleSphereVector / particleSphereDistance;
    
    // Si no hago la resta asi, cuando la particula este dentro de la esfera m_Lambda < 0.0 y cuando este fuera m_lambda
    // > 0.0 por lo que cuando este en rango maximo no se produce colision segun arriba seria c = particleSphereDistance
    // -
    float constraint = gSphereRadius - particleSphereDistance;
    
    // En este caso no se que otro valor puede tener lambda que no sea constraint, he buscado en las diapos y no he encontrado
    // la forma (siempre hay dos particulas y en este caso 1)
    m_Lambda = constraint;
    if (m_Lambda >= 0.0f) // In this case the particle is inside the sphere
        return;

    Vector3f delta = m_Lambda * particleSphereDirection;
    m_P1->m_P += delta;
}
}  // namespace utad