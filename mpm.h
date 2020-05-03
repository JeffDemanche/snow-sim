#ifndef MPM_H
#define MPM_H

#include <vector>
#include <iostream>
#include <QString>
#include "mesh.h"
#include "grid.h"

using namespace std;


class MPM
{
public:
    MPM(Mesh snowMesh, QString outDir, int numParticles, int numFrames, float stepLength, bool debugStepTimes = true, int debugParticles = 1);

    MPM(string sceneFile);

    MPM();

    /**
     * Does the whole thing, including simulating and exporting every frame.
     */
    void runSimulation();

    vector<Eigen::Vector3f> update(float seconds, int currentFrame);

    void exportFrame();

    float randomNumber(float Min, float Max);
    vector<Eigen::Vector3f> getPositions();
    pair<Vector3f, Vector3f> getGridBounds();
    vector<CollisionObject*> getColliders();

private:
    void writeFrameToFile(int frameNum);

    void doStep(int step, float delta_t, string descipription);

    Mesh m_snowMesh;
    QString m_outDir;

    bool m_firstStep;
    int m_numParticles;
    int m_numFrames;
    float m_stepLength;
    float m_timer;
    bool m_debugStepTimes;
    int m_debugParticles;

    Grid* m_grid;
    vector<Eigen::Vector3f> m_particlePositions;

};

#endif // MPM_H
