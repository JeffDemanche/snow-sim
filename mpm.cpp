#include "mpm.h"
#include <iostream>
#include <fstream>

using namespace std;

MPM::MPM(Mesh snowMesh, QString outDir, int numParticles, int numFrames, float stepLength):
    m_snowMesh(snowMesh),
    m_outDir(outDir),
    m_numParticles(numParticles),
    m_numFrames(numFrames),
    m_stepLength(stepLength)
{
    m_grid = new Grid(snowMesh, numParticles);
    m_particlePositions = m_grid->getPoints();
    m_firstStep = true;
}

MPM::MPM() {

}

void MPM::runSimulation() {
    for (int i = 0; i < m_numFrames; i++) {
        cout << "Frame: " << i << endl;
        update(m_stepLength);
        writeFrameToFile(i);
    }
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
    m_grid->updateGridVelocities(seconds);

    // Step 5
    m_grid->gridCollision();


    // Step 6
    m_grid->explicitSolver();
    //m_grid->implicitSolver();


    // Step 7
    m_grid->calculateDeformationGradient(seconds);

    // Step 8
    m_grid->updateParticleVelocities();

    // Step 9
    m_grid->particleCollision();

    m_grid->updateParticlePositions(seconds);

    // Or other way to send updated positions to GLWidget
    std::vector<Vector3f> newPositions = m_grid->getPoints();

    m_grid->reset();

    return newPositions;
}

void MPM::writeFrameToFile(int frameNum) {
    QString paddedNumber = QString("%4").arg(frameNum, 5, 10, QChar('0'));
    ofstream particlesFile((m_outDir + "\\particles." + paddedNumber).toUtf8());
    for (unsigned int p = 0; p < m_grid->getParticles().size(); p++) {
        Particle part = *m_grid->getParticles()[p];
        QString position = QString("[%1,%2,%3]").arg(part.getPosition()[0]).arg(part.getPosition()[1]).arg(part.getPosition()[2]);
        QString volume = QString("%1").arg(part.getVolume());
        QString mass = QString("%1").arg(part.getMass());
        particlesFile << "p:" << position.toStdString() << " V:" << volume.toStdString() << " m:" << mass.toStdString() << endl;
    }
    particlesFile.close();

    ofstream gridNodesFile((m_outDir + "\\grid." + paddedNumber).toUtf8());
    for (unsigned int n = 0; n < m_grid->getGridNodes().size(); n++) {
        GridNode node = *m_grid->getGridNodes()[n];
        QString index = QString("[%1,%2,%3]").arg(node.getIndex()[0]).arg(node.getIndex()[1]).arg(node.getIndex()[2]);
        QString mass = QString("%1").arg(node.getMass());
        gridNodesFile << "i:" << index.toStdString() << " m:" << mass.toStdString() << endl;
    }
    gridNodesFile.close();
}

std::vector<Vector3f> MPM::getPositions() {
    return m_particlePositions;
}

std::pair<Vector3f, Vector3f> MPM::getGridBounds() {
    return m_grid->getGridBounds();
}

std::vector<CollisionObject*> MPM::getColliders() {
    return m_grid->getColliders();
}

float MPM::randomNumber(float Min, float Max) {
    return ((float(rand()) / float(RAND_MAX)) * (Max - Min)) + Min;
}
