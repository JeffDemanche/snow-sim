#ifndef GRID_H
#define GRID_H

#include "particle.h"
#include "gridnode.h"
#include "mesh.h"

using namespace std;

class Grid
{
public:
    Grid(Mesh snowMesh, size_t numParticles);

private:
    vector<Particle> m_particles;
    vector<GridNode> m_nodes;

    /**
     * This is run before the simulation even begins. It loads the particles
     * taken from the mesh data into the particle vector with initial states.
     */
    void initParticles(vector<Vector3f> points);

    /**
     * Generates a list of points positions inside the snow mesh. The
     * amount of particles is determined by m_numParticles.
     */
    vector<Vector3f> pointsFromMesh(Mesh mesh, size_t numParticles);

};

#endif // GRID_H
