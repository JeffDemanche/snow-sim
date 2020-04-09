#include "mpm.h"

MPM::MPM(Mesh snowMesh, int numParticles):
    m_snowMesh(snowMesh),
    m_numParticles(numParticles)
{
    m_grid = new Grid(snowMesh, numParticles);
}

