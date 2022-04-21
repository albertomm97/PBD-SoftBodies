#include "NeighborsSearch.h"

namespace utad
{
NeighborsSearch::NeighborsSearch(ParticleSystem *particleSystem, const uint nx, const uint ny, const uint nz)
    : m_ParticleSystem(particleSystem)
    , m_Voxelization(nx, ny, nz)
{
    m_Dx = m_ParticleSystem->m_Radius * 3.0F;
    m_DxInv = 1.0F / m_Dx;
    m_DxSquared = m_Dx * m_Dx;
}

void NeighborsSearch::Update()
{
    m_Voxelization.clear();

    // TODO: Insert all particles in the voxelization
    // NOTE: getIndexFromPosition already implements the hashing function
    auto &particles = m_ParticleSystem->m_Particles;

    for (auto *particle : particles) {
        
        int i, j, k;
        getIndexFromPosition(particle->m_X, i, j, k);
        
        // Get first particle in slot
        auto *p = m_Voxelization(i, j, k);
        
        // If particle not null 
        if (p) {  

            // Look for the next element where we can store another particle
            while (p) {
                p = p->m_Next;
            }

            // Store particle
            p = particle;
            
        } else {
            // If particle is null, then add particle
            m_Voxelization(i, j, k) = particle;
        }
    }
}

void NeighborsSearch::QueryNeighbors(const Particle *particle, std::vector<Particle *> &neighbors)
{
    neighbors.clear();

    auto position = particle->m_X;
    int x, y, z;
    getIndexFromPosition(position, x, y, z);
    
    auto ni = m_Voxelization.m_NI;
    auto nj = m_Voxelization.m_NJ;
    auto nk = m_Voxelization.m_NK;
    
    Particle *p2;

    for (int i = x - 1; i < x + 2; ++i) {
        
        int ri = i % ni;

        for (int j = y - 1; j < y + 2; ++j) {
            
            int rj = j % nj;

            for (int k = z - 1; k < z + 2; ++k) {

                int rk = k % nk;

                // Check requeriments
                p2 = m_Voxelization(ri, rj, rk);
                if (p2 == nullptr || (p2->m_X - particle->m_X).norm() > m_Dx || (particle->m_Body == p2->m_Body && p2->id != particle->id)) {
                    continue;
                }
                
                // if its the same
                if (particle->id == p2->id) {
                    while (p2->m_Next) {
                        if (p2->m_Next->m_Body != particle->m_Body && (p2->m_Next->m_X - particle->m_X).norm() <= m_Dx) {
                            neighbors.push_back(p2->m_Next);
                        }
                        p2 = p2->m_Next;
                    }
                    continue;
                }

                // Otherwise we have a particle with different m_body that meets all requirements
                neighbors.push_back(p2);
                while (p2->m_Next) {
                    if (p2->m_Next->m_Body != particle->m_Body && (p2->m_Next->m_X - particle->m_X).norm() <= m_Dx) {
                        neighbors.push_back(p2->m_Next);
                    }
                    p2 = p2->m_Next;
                }            
            }
        }
    }
}

void NeighborsSearch::getIndexFromPosition(const Vector3f &position, int &i, int &j, int &k)
{
    const uint ni = m_Voxelization.m_NI;
    const uint nj = m_Voxelization.m_NJ;
    const uint nk = m_Voxelization.m_NK;

    const Vector3f indexSpace = position * m_DxInv;
    i = static_cast<int>(std::floor(indexSpace.x())) % ni;
    j = static_cast<int>(std::floor(indexSpace.y())) % nj;
    k = static_cast<int>(std::floor(indexSpace.z())) % nk;
}
}  // namespace utad
