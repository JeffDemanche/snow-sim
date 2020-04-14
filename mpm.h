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
    MPM();

    std::vector<Eigen::Vector3f> update(float seconds);
    float randomNumber(float Min, float Max);
    std::vector<Eigen::Vector3f> getPositions();
    std::pair<Vector3f, Vector3f> getGridBounds();

private:
    Mesh m_snowMesh;

    bool m_firstStep;
    int m_numParticles;
    float m_timer;

    Grid* m_grid;
    std::vector<Eigen::Vector3f> m_particlePositions;

};

#endif // MPM_H
