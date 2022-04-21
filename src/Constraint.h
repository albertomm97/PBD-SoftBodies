#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <Math/Matrix3f.h>
#include <Math/Vector3f.h>

#include <vector>

namespace utad
{
class Constraint
{
public:
    virtual ~Constraint() {}

    void UpdateStiffnessPerIteration(const uint iterations);

    virtual void Draw() = 0;

    virtual void Project() = 0;

    float m_Stiffness{1.0F};

    float m_Lambda{0.0F};

protected:
    float m_StiffnessIter{1.0F};
};
}  // namespace utad

#endif