#ifndef NEIGHBORS_SEARCH_H
#define NEIGHBORS_SEARCH_H

#include <Containers/Field3.h>
#include <ParticleSystem.h>

namespace utad
{
class NeighborsSearch
{
public:
    using ParticleGrid = Field3<Particle *>;

    NeighborsSearch(ParticleSystem *particleSystem, const uint nx = 32, const uint ny = 32, const uint nz = 32);

    void Update();

    void QueryNeighbors(const Particle *particle, std::vector<Particle *> &neighbors);

private:
    void getIndexFromPosition(const Vector3f &position, int &i, int &j, int &k);

    ParticleSystem *m_ParticleSystem;

    float m_Dx;
    float m_DxInv;
    float m_DxSquared;

    ParticleGrid m_Voxelization;
};
}  // namespace utad

#endif
