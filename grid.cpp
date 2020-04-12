#include "grid.h"
#include <chrono>
#include <iostream>

Grid::Grid(Mesh snowMesh, size_t numParticles)
{
    vector<Vector3f> points = pointsFromMesh(snowMesh, numParticles);
    initParticles(points);
}


void Grid::initParticles(vector<Vector3f> points)
{
    for(Vector3f p : points) {
        Particle particle = Particle(p);
        m_particles.push_back(particle);
        m_points.push_back(p);
    }
}

vector<Vector3f> Grid::pointsFromMesh(Mesh mesh, size_t numParticles)
{
    auto t0 = chrono::high_resolution_clock::now();

    vector<Vector3f> points = vector<Vector3f>();
    for (size_t i = 0; i < numParticles; i++) {
        points.push_back(mesh.randPosition());
    }

    auto t1 = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(t1-t0).count();
    cout << "Generating points: " << duration << " milliseconds" << endl;
    return points;
}

vector<Vector3f> Grid::getPoints()
{
    return m_points;
}

void Grid::computeGridMass()
{
    // TODO Step 1. See eq. 3.1 from the masters doc
}

void Grid::computeGridVelocity()
{
    // TODO Step 1. See eq. 3.6 from the masters doc (this requires using the calculated mass, so do that first).
}

void Grid::computeParticleVolumes()
{
    // TODO Step 2. See eq. 3.8. I think this only needs to be done once for the whole simulation.
}

void Grid::computeGridForces()
{
    // TODO Step 3. See eq. 3.9. Do we need to compute the deformation gradient prior to this happening?
}

void Grid::updateGridVelocities()
{
    // TODO Step 4. See eq. 3.16. This updates velocities using forces we calculated last step.
}

void Grid::gridCollision()
{
    // TODO Steps 5 and 9. See eq. 3.17.
}

void Grid::implicitSolver()
{
    // TODO Step 6. See Algorithm 1. This will likely require a lot of calculating grid node values.
}

void Grid::calculateDeformationGradient()
{
    // TODO Step 7. See eq. 3.33 / 3.34
}

void Grid::updateParticleVelocities()
{
    // TODO Step 8. See eq. 3.35.
}

void Grid::updateParticlePositions()
{
    // TODO Step 10. Eq. 3.36. Simple time step to finish.
}
