#ifndef SOFT_BODY_H
#define SOFT_BODY_H

#include <Containers/Field3.h>
#include <Math/Quaternionf.h>
#include <Math/Matrix3f.h>
#include <Math/Vector3f.h>
#include <DistanceConstraint.h>
#include <ParticleSystem.h>

#include <iostream>

namespace utad
{
class SoftBody
{
public:
    using ParticleField = Field3<Particle *>;

    SoftBody(ParticleSystem *particleSystem);
    ~SoftBody();

    void InitParticleField(const uint ni, const uint nj, const uint nk);

    void AddParticle(const uint i, const uint j, const uint k, const Vector3f &position);

    void UpdateConstraints();
    void ReleaseConstraints();

    void Draw();

    void EndStep();
    
    bool checkDuplicated(Particle *p1, Particle *p2);

    ParticleSystem *m_ParticleSystem;
    uint m_Body;

    float m_ParticleMass{1.0F};
    float m_ShearStiffness{1.0F};
    float m_StructuralStiffness{1.0F};

    Vector3f m_Color;

    ParticleField m_ParticleField;

    std::vector<DistanceConstraint *> m_Constraints;
};

void initSoftBodyFromCube(SoftBody *sb, const Vector3f &x0, const Quaternionf &q0, const Vector3f &size);
}  // namespace utad

#endif