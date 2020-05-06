#ifndef GRID_H
#define GRID_H

#include "particle.h"
#include "gridnode.h"
#include "mesh.h"
#include "collisionobject.h"
#include "ground.h"
#include <unordered_map>
#include <thread>
#include "gridboundcollider.h"
#include "scenefile.h"

using namespace std;

class Grid
{
public:
    Grid(Mesh snowMesh, size_t numParticles, GridInfo gridInfo, HyperparameterInfo hyperparamaterInfo);
    ~Grid();

    void setCurrentFrame(int currentFrame);

    vector<Particle*> getParticles();
    vector<GridNode*> getGridNodes();

    vector<Vector3f> getPoints();
    pair<Vector3f, Vector3f> getGridBounds();
    std::vector<CollisionObject*> getColliders();

    GridNode* getNodeAt(int x, int y, int z);
    int getNodeIndexAt(int x, int y, int z);

    /**
     * Step 1. Calculates the grid mass values based on particles.
     */
    void computeGridMass(int thread, int numThreads);
    thread computeGridMassThread(int t, int numThreads);

    /**
     * Step 1. Calculates the grid velocity values based on particles.
     */
    void computeGridVelocity(int thread, int numThreads);
    thread computeGridVelocityThread(int t, int numThreads);

    /**
     * Step 2. Calculates the volumes for each particle.
     */
    void computeParticleVolumes(int t, int numThreads);
    thread computeParticleVolumesThread(int t, int numThreads);

    /**
     * Step 3. Calculates forces acting at grid points.
     */
    void computeGridForces(int thread, int numThreads);
    thread computeGridForcesThread(int thread, int numThreads);

    /**
     * Step 4. Calculates updated grid velocities.
     */
    void updateGridVelocities(float delta_t, int thread, int numThreads);
    thread updateGridVelocitiesThread(float delta_t, int t, int numThreads);

    /**
     * Step 5. Temporary grid node velocity is updated with grid-based body collision.
     */
    void gridCollision(int thread, int numThreads);
    thread gridCollisionThread(int t, int numThreads);

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
    void calculateDeformationGradient(float delta_t, int thread, int numThreads);
    thread calculateDeformationGradientThread(float delta_t, int t, int numThreads);

    /**
     * Step 8. Update particle velocities after doing the grid work.
     */
    void updateParticleVelocities(int thread, int numThreads);
    thread updateParticleVelocitiesThread(int t, int numThreads);

    /**
     * Step 9. Particle-based body collisions
     */
    void particleCollision(int thread, int numThreads);
    thread particleCollisionThread(int t, int numThreads);

    /**
     * Step 10. Update particle positions.
     */
    void updateParticlePositions(float delta_t);

    /**
     * Clears all GridNode info except position
     */
    void reset();

    void debugGridNodes(int num, int afterStep);

    void debugParticles(int num, int afterStep);

private:
    vector<Particle*> m_particles;
    vector<GridNode*> m_nodes;
    vector<Vector3f> m_points;
    pair<Vector3f, Vector3f> m_gridBounds;
    float m_gridWidth;
    float m_gridHeight;
    float m_gridDepth;
    float m_gridSpacing;
    std::vector<CollisionObject*> m_colliders;
    int m_currentFrame;
    float m_particleMass;

    Vector3f _initParticleVelocity = Vector3f(-0.5,0,0);
    Vector3f _gravity = Vector3f(0, -1, 0); // Should be -10 for Earth gravity
    float _groundHeight = -0.2; // Location of the ground plane (in meters)

    float _criticalCompression = 2.5E-2;
    float _criticalStretch = 7.5E-3;
    float _Eo = 1.4E5; // Initial Young's Modulus
    float _v = 0.2; // Poisson's ratio
    float _hardening = 10; // Hardening coefficient
    float _targetDensity = 400.f;

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

    float weightFunctionN(float x);
    float weightGradientFunctionDelN(float x);
    float weightN(Vector3f particlePos, Vector3f nodePos);
    Vector3f weightGradientDelOmega(Vector3f particlePos, Vector3f nodePos);

    Matrix3f velocityGradient(Particle* particle);

    Matrix3f computeStress(Matrix3f Fe, Matrix3f Fp);
    float lambda(Matrix3f Fp, float Jp);
    float mu(Matrix3f Fp, float Jp);
    bool outOfBounds(Particle* p);

    int closestGridNode(Particle* particle);

    void debug(string label, MatrixXf m);
    void debug(string label, float m);

};

#endif // GRID_H
