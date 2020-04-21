#ifndef GRID_H
#define GRID_H

#include "particle.h"
#include "gridnode.h"
#include "mesh.h"
#include "collisionobject.h"
#include "ground.h"

using namespace std;

class Grid
{
public:
    Grid(Mesh snowMesh, size_t numParticles);
    ~Grid();

    vector<Vector3f> getPoints();
    pair<Vector3f, Vector3f> getGridBounds();
    std::vector<CollisionObject*> getColliders();

    /**
     * Step 1. Calculates the grid mass values based on particles.
     */
    void computeGridMass();

    /**
     * Step 1. Calculates the grid velocity values based on particles.
     */
    void computeGridVelocity();

    /**
     * Step 2. Calculates the volumes for each particle.
     */
    void computeParticleVolumes();

    /**
     * Step 3. Calculates forces acting at grid points.
     */
    void computeGridForces();

    /**
     * Step 4. Calculates updated grid velocities.
     */
    void updateGridVelocities(float delta_t);

    /**
     * Step 5. Temporary grid node velocity is updated with grid-based body collision.
     */
    void gridCollision();

    /**
     * Step 6: Explicit integrator
     */
    void explicitSolver();

    /**
     * Step 6. Implicit integrator.
     */
    void implicitSolver();

    /**
     * Step 7. Update deformation gradient for particles.
     */
    void calculateDeformationGradient(float delta_t);

    /**
     * Step 8. Update particle velocities after doing the grid work.
     */
    void updateParticleVelocities();

    /**
     * Step 9. Particle-based body collisions
     */
    void particleCollision();

    /**
     * Step 10. Update particle positions.
     */
    void updateParticlePositions(float delta_t);

    /**
     * Clears all GridNode info except position
     */
    void reset();

private:
    vector<Particle> m_particles;
    vector<GridNode> m_nodes;
    vector<Vector3f> m_points;
    pair<Vector3f, Vector3f> m_gridBounds;
    float m_gridWidth;
    float m_gridHeight;
    float m_gridDepth;
    float m_gridSpacing;
    std::vector<CollisionObject*> m_colliders;

    /**
     * Initializes all colliding objects in the scene
     */
    void initColliders();

    /**
     * This is run before the simulation even begins. It loads the particles
     * taken from the mesh data into the particle vector with initial states.
     */
    void initParticles(vector<Vector3f> points);

    /**
     * Takes minimum and maximum corners of the grid and creates the GridNodes
     */
    void initGrid(Vector3f min, Vector3f max);

    /**
     * Generates a list of points positions inside the snow mesh. The
     * amount of particles is determined by m_numParticles.
     */
    vector<Vector3f> pointsFromMesh(Mesh mesh, size_t numParticles);

    /**
     * Finds bounds of grid based on mesh bounding box and lowest y point in scene (the ground)
     */
    pair<Vector3f, Vector3f> findGridBoundaries(Vector3f bbMin, Vector3f bbMax, float lowestY);

    std::vector<int> getNeighboringGridNodes(Vector3i gridNodeOrigin, Vector3f particlePos);
    float weightFunctionN(float x);
    float weightGradientFunctionDelN(float x);
    float weightN(Vector3f particlePos, Vector3f nodePos);
    Vector3f weightGradientDelOmega(Vector3f particlePos, Vector3f nodePos);

    Matrix3f velocityGradient(Particle particle);

};

#endif // GRID_H
