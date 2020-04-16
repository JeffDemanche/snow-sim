#include "mpm.h"

MPM::MPM(Mesh snowMesh, int numParticles):
    m_snowMesh(snowMesh),
    m_numParticles(numParticles)
{
    m_grid = new Grid(snowMesh, numParticles);
    m_particlePositions = m_grid->getPoints();
    m_firstStep = true;
}

MPM::MPM() {
}

std::vector<Vector3f> MPM::update(float seconds) {

    // Step 1
    m_grid->computeGridMass();
    m_grid->computeGridVelocity();

    // Step 2
    if (m_firstStep) {
        m_grid->computeParticleVolumes();
        m_firstStep = false;
    }

    // Step 3
    m_grid->computeGridForces();

    // Step 4
    m_grid->updateGridVelocities();

    // Step 5
    m_grid->gridCollision();

    // Step 6
    m_grid->explicitSolver();
    //m_grid->implicitSolver();

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
