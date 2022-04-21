#include "Constraint.h"

namespace utad
{
    void Constraint::UpdateStiffnessPerIteration(const uint iterations) {
        m_StiffnessIter = 1.0F - std::pow(1.0F - m_Stiffness, 1.0F / iterations);
    }
}