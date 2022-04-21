#ifndef FIELD_3_H
#define FIELD_3_H

#include <Std/CStdInt.h>

#include <vector>
#include <iostream>
namespace utad
{
template <class T>
class Field3
{
public:
    Field3() {}

    Field3(const uint ni, const uint nj, const uint nk) { resize(ni, nj, nk); }

    size_t size() const { return m_V.size(); }

    const T *data() const { return m_V.data(); }

    T *data() { return m_V.data(); }

    void clear()
    {
        for (auto &entry : m_V) {
           entry = T();
        }
    }

    void resize(const uint ni, const uint nj, const uint nk)
    {
        const uint n = ni * nj * nk;
        m_NI = ni;
        m_NJ = nj;
        m_NK = nk;
        m_V.resize(n);
    }

    const T &operator[](const uint i) const { return m_V[i]; }

    T &operator[](uint i) { return m_V[i]; }

    const T &operator()(const int i, const int j, const int k) const
    {
        const uint idx = index1(i, j, k);
        return m_V[idx];
    }

    T &operator()(const int i, const int j, const int k)
    {
        const int idx = index1(i, j, k);
        return m_V[idx];
    }

    uint index1(const uint i, const uint j, const uint k) const { return k * m_NI * m_NJ + j * m_NI + i; }

    uint m_NI;
    uint m_NJ;
    uint m_NK;

private:
    std::vector<T> m_V;
};
};  // namespace utad

#endif
