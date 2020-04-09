#ifndef MPM_H
#define MPM_H

#include <vector>
#include <iostream>
#include "mesh.h"
#include "grid.h"

using namespace std;

class MPM
{
public:
    MPM(Mesh snowMesh, int numParticles);

private:
    Mesh m_snowMesh;

    bool m_firstStep;
    int m_numParticles;
    float m_timer;

    Grid* m_grid;

};

#endif // MPM_H
