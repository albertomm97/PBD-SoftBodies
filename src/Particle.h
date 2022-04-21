#ifndef PARTICLE_H
#define PARTICLE_H

#include <Math/Vector3f.h>

namespace utad
{
class Particle
{
public:
    Particle(const uint body, const float mass = 1.0F);
    static uint id_gen;
    uint id;

    float m_Mass;
    float m_MassInv;

    Vector3f m_X;
    Vector3f m_V;
    Vector3f m_F;

    uint m_Body;
    Vector3f m_Color{1.0F, 1.0F, 1.0F};

    // PBD
    Vector3f m_P;
 
    // Voxelization
    Particle *m_Next;
};
}  // namespace utad

#endif