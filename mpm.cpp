#include "mpm.h"

// Define parameters here
const float _gridSpacing = 0.1; // units for this would be cm (group that did this a few years ago used 0.001 m)

MPM::MPM(Mesh snowMesh, int numParticles):
    m_snowMesh(snowMesh),
    m_numParticles(numParticles)
{
    m_grid = new Grid(snowMesh, numParticles, _gridSpacing);
    m_particlePositions = m_grid->getPoints();
}

MPM::MPM() {
}

// Right now this is just moving all particles down to confirm visualization
std::vector<Vector3f> MPM::update(float seconds) {
//    std::vector<Vector3f> newPositions;
//    for (int i = 0; i < m_particlePositions.size(); i++) {
//        newPositions.push_back(m_particlePositions[i] + Vector3f(0, -0.1 * seconds, 0));
//        m_particlePositions[i] = m_particlePositions[i] + Vector3f(0, -0.1 * seconds, 0);
//    }
//    return newPositions;


    // Step 1
    m_grid->computeGridMass();
    m_grid->computeGridVelocity();

    // Step 2
    m_grid->computeParticleVolumes();

    // Step 3
    m_grid->computeGridForces();

    // Step 4
    m_grid->updateGridVelocities();

    // Step 5
    m_grid->gridCollision();

    // Step 6
    m_grid->implicitSolver();

    // Step 7
    m_grid->calculateDeformationGradient();

    // Step 8
    m_grid->updateParticleVelocities();

    // Step 9
    // Maybe this should be a separate method because it's acting on particles instead of grid nodes?
    m_grid->gridCollision();

    m_grid->updateParticlePositions();

    // Or other way to send updated positions to GLWidget
    std::vector<Vector3f> newPositions = m_grid->getPoints();
    return newPositions;
}

std::vector<Vector3f> MPM::getPositions() {
    return m_particlePositions;
}

std::pair<Vector3f, Vector3f> MPM::getGridBounds() {
    return m_grid->getGridBounds();
}

float MPM::randomNumber(float Min, float Max) {
    return ((float(rand()) / float(RAND_MAX)) * (Max - Min)) + Min;
}
