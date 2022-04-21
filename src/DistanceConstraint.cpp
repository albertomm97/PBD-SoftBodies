#include "DistanceConstraint.h"

#include <GL/glut.h>
#include <iostream>
#include <limits>
namespace utad
{
DistanceConstraint::DistanceConstraint(Particle *p1, Particle *p2, float restLength)
    : m_RestLength(restLength)
{
    m_P1 = p1;
    m_P2 = p2;
}

void DistanceConstraint::Draw()
{
    glBegin(GL_LINES);
    glVertex3f(m_P1->m_X.x(), m_P1->m_X.y(), m_P1->m_X.z());
    glVertex3f(m_P2->m_X.x(), m_P2->m_X.y(), m_P2->m_X.z());
    glEnd();
}

void DistanceConstraint::Project()
{
    // TODO: Apply constraint projection
    // NOTE: C = ||P1 - P2|| - L0
    // NOTE: Use the mass dependent formulation
    // NOTE: Compute m_Lambda first

    // NOTE: Multiply final position corrections by m_StiffnessIter before applying to control the rigidity
    // NOTE: https://matthias-research.github.io/pages/publications/posBasedDyn.pdf
    float p1p2Distance = (m_P1->m_P - m_P2->m_P).norm();
    Vector3f p1p2Direction = (m_P1->m_P - m_P2->m_P) / p1p2Distance;

    float constraint = p1p2Distance - m_RestLength;

    if (constraint > -0.0001f && constraint < 0.0001f) {
        return;
    }
        
    m_Lambda = constraint / (m_P1->m_MassInv + m_P2->m_MassInv);

    Vector3f deltap1 = -m_Lambda * m_P1->m_MassInv * p1p2Direction;
    m_P1->m_P += deltap1 * m_StiffnessIter;

    Vector3f deltap2 = m_Lambda * m_P2->m_MassInv * p1p2Direction;
    m_P2->m_P += deltap2 * m_StiffnessIter;
}
}  // namespace utad