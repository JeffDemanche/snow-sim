#include "mpm.h"
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <Eigen/Core>

using namespace std;
using namespace std::chrono;

MPM::MPM(Mesh snowMesh, QString outDir, int numParticles, int numFrames, float stepLength, bool debugStepTimes):
    m_snowMesh(snowMesh),
    m_outDir(outDir),
    m_numParticles(numParticles),
    m_numFrames(numFrames),
    m_stepLength(stepLength),
    m_debugStepTimes(debugStepTimes)
{
    m_grid = new Grid(snowMesh, numParticles);
    m_particlePositions = m_grid->getPoints();
    m_firstStep = true;
}

MPM::MPM() {

}

void MPM::runSimulation() {
    Eigen::initParallel();
    for (int i = 0; i < m_numFrames; i++) {
        cout << "Frame: " << i << endl;
        auto t0 = high_resolution_clock::now();

        update(m_stepLength, i);

        auto t1 = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(t1-t0).count();

        cout << "Frame simulation took " << duration << "ms" << endl;
    }
}

std::vector<Vector3f> MPM::update(float seconds, int currentFrame) {

    thread simThreads[4];

    m_grid->setCurrentFrame(currentFrame);

    // Step 1
    doStep(1, seconds, "compute grid mass/velocity");

    // Step 2
    if (m_firstStep) {
        doStep(2, seconds, "compute particle volumes");
        m_firstStep = false;
    }

    // Step 3
    doStep(3, seconds, "compute grid forces");

    // Step 4
    doStep(4, seconds, "update grid velocities");

    // Step 5
    doStep(5, seconds, "compute grid collisions");

    // Step 6
    doStep(6, seconds, "explicit solver");
    //m_grid->implicitSolver();

    // Step 7
    doStep(7, seconds, "calculate deformation gradient");

    // Step 8
    doStep(8, seconds, "update particle velocities");

    // Step 9
    doStep(9, seconds, "particle collisions");

    // Step 10
    doStep(10, seconds, "update particle positions");

    // Or other way to send updated positions to GLWidget
    std::vector<Vector3f> newPositions = m_grid->getPoints();

    writeFrameToFile(currentFrame);

    m_grid->reset();

    return newPositions;
}

void MPM::doStep(int step, float delta_t, string description) {
    auto t0 = high_resolution_clock::now();

    assert(step >= 1 && step <= 10);
    switch(step) {
    case 1:
        m_grid->computeGridMass();
        m_grid->computeGridVelocity();
        break;
    case 2:
        m_grid->computeParticleVolumes();
        break;
    case 3: {
        thread t[8];
        for (unsigned int i = 0; i < 8; i++) {
            t[i] = m_grid->computeGridForcesThread(i, 8);
        }
        for (unsigned int i = 0; i < 8; i++) {
            t[i].join();
        }
        break;
    }
    case 4:
        m_grid->updateGridVelocities(delta_t);
        break;
    case 5:
        m_grid->gridCollision();
        break;
    case 6:
        m_grid->explicitSolver();
        break;
    case 7:
        m_grid->calculateDeformationGradient(delta_t);
        break;
    case 8:
        m_grid->updateParticleVelocities();
        break;
    case 9:
        m_grid->particleCollision();
        break;
    case 10:
        m_grid->updateParticlePositions(delta_t);
        break;
    }

    auto t1 = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(t1-t0).count();
    if (m_debugStepTimes)
        cout << "\tStep " << step << " (" << description << ") took " << duration << "ms" << endl;
}

void MPM::writeFrameToFile(int frameNum) {
    QString paddedNumber = QString("%4").arg(frameNum, 5, 10, QChar('0'));
    //ofstream particlesFile((m_outDir + "\\particles." + paddedNumber).toUtf8());
    ofstream particlesFile((m_outDir + "/particles." + paddedNumber).toUtf8());
    for (unsigned int p = 0; p < m_grid->getParticles().size(); p++) {
        Particle part = *m_grid->getParticles()[p];
        QString position = QString("[%1,%2,%3]").arg(part.getPosition()[0]).arg(part.getPosition()[1]).arg(part.getPosition()[2]);
        QString volume = QString("%1").arg(part.getVolume());
        QString mass = QString("%1").arg(part.getMass());
        particlesFile << "p:" << position.toStdString() << " V:" << volume.toStdString() << " m:" << mass.toStdString() << endl;
    }
    particlesFile.close();

    //ofstream gridNodesFile((m_outDir + "\\grid." + paddedNumber).toUtf8());
    ofstream gridNodesFile((m_outDir + "/grid." + paddedNumber).toUtf8());
    for (unsigned int n = 0; n < m_grid->getGridNodes().size(); n++) {
        GridNode node = *m_grid->getGridNodes()[n];
        QString index = QString("[%1,%2,%3]").arg(node.getIndex()[0]).arg(node.getIndex()[1]).arg(node.getIndex()[2]);
        QString mass = QString("%1").arg(node.getMass());
        QString velocity = QString("[%1,%2,%3]").arg(node.getVelocity()[0]).arg(node.getVelocity()[1]).arg(node.getVelocity()[2]);
        gridNodesFile << "i:" << index.toStdString() << " density:" << mass.toStdString() << " v:" << velocity.toStdString() << endl;
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
