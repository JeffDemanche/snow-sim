#include "mpm.h"

MPM::MPM(Mesh snowMesh, int numParticles):
    m_snowMesh(snowMesh),
    m_numParticles(numParticles)
{
    m_grid = new Grid(snowMesh, numParticles);
    m_particlePositions = m_grid->getPoints();
}

MPM::MPM() {
}

// Right now this is just moving all particles down to confirm visualization
std::vector<Vector3f> MPM::update(float seconds) {
    std::vector<Vector3f> newPositions;
    for (int i = 0; i < m_particlePositions.size(); i++) {
        newPositions.push_back(m_particlePositions[i] + Vector3f(0, -0.1 * seconds, 0));
        m_particlePositions[i] = m_particlePositions[i] + Vector3f(0, -0.1 * seconds, 0);
    }
    return newPositions;
}

std::vector<Vector3f> MPM::getPositions() {
    return m_particlePositions;
}

float MPM::randomNumber(float Min, float Max) {
    return ((float(rand()) / float(RAND_MAX)) * (Max - Min)) + Min;
}
