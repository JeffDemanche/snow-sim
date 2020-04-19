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
    MPM(Mesh snowMesh, int numParticles, int numFrames, float stepLength);

    MPM();

    /**
     * Does the whole thing, including simulating and exporting every frame.
     */
    void runSimulation();

    vector<Eigen::Vector3f> update(float seconds);

    void exportFrame();

    float randomNumber(float Min, float Max);
    vector<Eigen::Vector3f> getPositions();
    pair<Vector3f, Vector3f> getGridBounds();
    vector<CollisionObject*> getColliders();

private:
    Mesh m_snowMesh;

    bool m_firstStep;
    int m_numParticles;
    int m_numFrames;
    float m_stepLength;
    float m_timer;

    Grid* m_grid;
    vector<Eigen::Vector3f> m_particlePositions;

};

#endif // MPM_H
