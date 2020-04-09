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
