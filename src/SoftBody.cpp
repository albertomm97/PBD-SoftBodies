#include "SoftBody.h"

#include <Gfx/Gfx.h>
#include <cmath>
#include <algorithm>

namespace utad
{
static uint gBody = 0;

static int Rand(int max)
{
    return rand() % max;
}

SoftBody::SoftBody(ParticleSystem *paricleSystem)
    : m_ParticleSystem(paricleSystem)
    , m_Body(gBody++)
{
    m_Color = Vector3f(Rand(11) * 0.1F, Rand(11) * 0.1F, Rand(11) * 0.1F);
}

SoftBody::~SoftBody()
{
    ReleaseConstraints();
}

void SoftBody::InitParticleField(const uint ni, const uint nj, const uint nk)
{
    m_ParticleField.resize(ni, nj, nk);
}

void SoftBody::AddParticle(const uint i, const uint j, const uint k, const Vector3f &position)
{
    auto particle = m_ParticleSystem->CreateParticle(m_Body, m_ParticleMass);
    particle->m_Color = m_Color;
    particle->m_X = position;

    m_ParticleField(i, j, k) = particle;
}

void SoftBody::UpdateConstraints()
{
    ReleaseConstraints();

    // TODO: Create structural and shear DistanceConstraints
    // NOTE: m_ParticleSystem holds the particle radius
    // NOTE: Use m_ParticleField to setup the required connectivity easily
    auto nk = m_ParticleField.m_NK;
    auto ni = m_ParticleField.m_NI;
    auto nj = m_ParticleField.m_NJ;

    const float diameter = m_ParticleSystem->m_Radius*2.0f;
    //const float dist = std::sqrt(diameter * diameter);
    const float distance = diameter + 0.5f;
    /**/
    for (uint i = 0; i < ni; ++i) {
        
        int top = i - 1;
        if (top < 0)
            top = top * -1;
        top = top % ni;
        
        int bottom = (i + 1) % ni;
        if (bottom == 0)
            bottom = i;

        for (uint j = 0; j < nj; ++j) {

            int left = j - 1;
            if (left < 0)
                left = left * -1;
            left = left % nj;

            int right = (j + 1) % nj;
            if (right == 0)
                right = j;

            for (uint k = 0; k < nk; ++k) {

                int back = k - 1;
                if (back < 0)
                    back = back * -1;
                back = back % nk;

                int front = (k + 1) % nk;
                if (front == 0)
                    front = k;

                Particle *p;
                Particle *p2;

                p = m_ParticleField(i, j, k);
                
                p2 = m_ParticleField(bottom, j, k); // Abajo
                if (checkDuplicated(p, p2) && p->id != p2->id) {
                    float distance = (p->m_X - p2->m_X).norm();
                    m_Constraints.push_back(new DistanceConstraint(p, p2, distance));
                }
                
                p2 = m_ParticleField(top, j, k); // Arriba
                if (checkDuplicated(p, p2) && p->id != p2->id) {
                    float distance = (p->m_X - p2->m_X).norm();
                    m_Constraints.push_back(new DistanceConstraint(p, p2, distance));
                }

                p2 = m_ParticleField(i, right, k); // Derecha
                if (checkDuplicated(p, p2) && p->id != p2->id) {
                    float distance = (p->m_X - p2->m_X).norm();
                    m_Constraints.push_back(new DistanceConstraint(p, p2, distance));
                }

                p2 = m_ParticleField(i, left, k); // Izquierda
                if (checkDuplicated(p, p2) && p->id != p2->id) {
                    float distance = (p->m_X - p2->m_X).norm();
                    m_Constraints.push_back(new DistanceConstraint(p, p2, distance));
                }

                p2 = m_ParticleField(i, j, front); // Delante
                if (checkDuplicated(p, p2) && p->id != p2->id) {
                    float distance = (p->m_X - p2->m_X).norm();
                    m_Constraints.push_back(new DistanceConstraint(p, p2, distance));
                }
                
                p2 = m_ParticleField(i, j, back); // Detras
                if (checkDuplicated(p, p2) && p->id != p2->id) {
                    float distance = (p->m_X - p2->m_X).norm();
                    m_Constraints.push_back(new DistanceConstraint(p, p2, distance));
                }

                // Shear (todas las posibles combinaciones)
                p2 = m_ParticleField(bottom, right, k); // Abajo derecha
                if (checkDuplicated(p, p2) && p->id != p2->id) {
                    float distance = (p->m_X - p2->m_X).norm();
                    m_Constraints.push_back(new DistanceConstraint(p, p2, distance));
                }

                p2 = m_ParticleField(bottom, right, front);  // Abajo derecha
                if (checkDuplicated(p, p2) && p->id != p2->id) {
                    float distance = (p->m_X - p2->m_X).norm();
                    m_Constraints.push_back(new DistanceConstraint(p, p2, distance));
                }

                p2 = m_ParticleField(bottom, right, back);  // Abajo derecha
                if (checkDuplicated(p, p2) && p->id != p2->id) {
                    float distance = (p->m_X - p2->m_X).norm();
                    m_Constraints.push_back(new DistanceConstraint(p, p2, distance));
                }

                p2 = m_ParticleField(bottom, left, k);  // Abajo izquierda
                if (checkDuplicated(p, p2) && p->id != p2->id) {
                    float distance = (p->m_X - p2->m_X).norm();
                    m_Constraints.push_back(new DistanceConstraint(p, p2, distance));
                }

                p2 = m_ParticleField(bottom, left, front);  // Abajo izquierda
                if (checkDuplicated(p, p2) && p->id != p2->id) {
                    float distance = (p->m_X - p2->m_X).norm();
                    m_Constraints.push_back(new DistanceConstraint(p, p2, distance));
                }

                p2 = m_ParticleField(bottom, left, back);  // Abajo izquierda
                if (checkDuplicated(p, p2) && p->id != p2->id) {
                    float distance = (p->m_X - p2->m_X).norm();
                    m_Constraints.push_back(new DistanceConstraint(p, p2, distance));
                }

                p2 = m_ParticleField(top, right, k);  // Arriba derecha
                if (checkDuplicated(p, p2) && p->id != p2->id) {
                    float distance = (p->m_X - p2->m_X).norm();
                    m_Constraints.push_back(new DistanceConstraint(p, p2, distance));
                }

                p2 = m_ParticleField(top, right, front);  // Arriba derecha
                if (checkDuplicated(p, p2) && p->id != p2->id) {
                    float distance = (p->m_X - p2->m_X).norm();
                    m_Constraints.push_back(new DistanceConstraint(p, p2, distance));
                }

                p2 = m_ParticleField(top, right, back);  // Arriba derecha
                if (checkDuplicated(p, p2) && p->id != p2->id) {
                    float distance = (p->m_X - p2->m_X).norm();
                    m_Constraints.push_back(new DistanceConstraint(p, p2, distance));
                }

                p2 = m_ParticleField(top, left, k);  // Arriba izquierda
                if (checkDuplicated(p, p2) && p->id != p2->id) {
                    float distance = (p->m_X - p2->m_X).norm();
                    m_Constraints.push_back(new DistanceConstraint(p, p2, distance));
                }

                p2 = m_ParticleField(top, left, front);  // Arriba izquierda
                if (checkDuplicated(p, p2) && p->id != p2->id) {
                    float distance = (p->m_X - p2->m_X).norm();
                    m_Constraints.push_back(new DistanceConstraint(p, p2, distance));
                }

                p2 = m_ParticleField(top, left, back);  // Arriba izquierda
                if (checkDuplicated(p, p2) && p->id != p2->id) {
                    float distance = (p->m_X - p2->m_X).norm();
                    m_Constraints.push_back(new DistanceConstraint(p, p2, distance));
                }
            }
        }
    }
}

void SoftBody::ReleaseConstraints()
{
    for (auto constraint : m_Constraints) {
        delete constraint;
    }
}

void SoftBody::Draw()
{
    Gfx::pushTransform();
    Gfx::popTransform();
}

void SoftBody::EndStep()
{

}

bool SoftBody::checkDuplicated(Particle *p1, Particle *p2)
{
    auto e = std::find_if(m_Constraints.begin(), m_Constraints.end(), [p1, p2](DistanceConstraint *c) {
        return p2->id == c->m_P2->id && c->m_P1->id == p1->id;
    });
    
    return e == m_Constraints.end();
}

void initSoftBodyFromCube(SoftBody *sb, const Vector3f &x0, const Quaternionf &q0, const Vector3f &size)
{
    const Matrix3f r0 = Matrix3f(q0);

    const float h = sb->m_ParticleSystem->m_Radius * 2.0F;

    const Vector3f n = size * (1.0F / h);
    const uint ni = std::ceil(n.x());
    const uint nj = std::ceil(n.y());
    const uint nk = std::ceil(n.z());

    sb->InitParticleField(ni, nj, nk);

    for (uint i = 0; i < ni; ++i) {
        for (uint j = 0; j < nj; ++j) {   
            for (uint k = 0; k < nk; ++k) {
                Vector3f position = x0 + r0 * Vector3f(h * i, h * j, h * k);
                sb->AddParticle(i, j, k, position);
            }
        }
    }

    sb->UpdateConstraints();
}
}  // namespace utad