#include "grid.h"
#include <chrono>
#include <iostream>

Grid::Grid(Mesh snowMesh, size_t numParticles, float gridSpacing)
{
    snowMesh.buildBoundingBox();
    vector<Vector3f> points = pointsFromMesh(snowMesh, numParticles);
    initParticles(points);
    pair<Vector3f, Vector3f> boundingPoints = snowMesh.boundingBoxCorners();
    pair<Vector3f, Vector3f> gridBounds = findGridBoundaries(boundingPoints.first, boundingPoints.second, -2);
    initGrid(gridBounds.first, gridBounds.second, gridSpacing);
}

void Grid::initParticles(vector<Vector3f> points)
{
    for(Vector3f p : points) {
        Particle particle = Particle(p);
        m_particles.push_back(particle);
        m_points.push_back(p);
    }
}

pair<Vector3f, Vector3f> Grid::findGridBoundaries(Vector3f bbMin, Vector3f bbMax, float lowestY) {
    float meshWidth = fabs(bbMin.x() - bbMax.x());
    float meshDepth = fabs(bbMin.z() - bbMax.z());
    float spaceBelowMesh = fabs(bbMin.y() - lowestY);

    // Add entire mesh width and entire mesh depth on each side of the mesh, take height of mesh + space below + wiggle room
    Vector3f gridMin(bbMin.x() - meshWidth, bbMin.y() - (spaceBelowMesh * 1.5), bbMin.z() - meshDepth);
    Vector3f gridMax(bbMax.x() + meshWidth, bbMax.y(), bbMax.z() + meshDepth);

    return pair<Vector3f, Vector3f>(gridMin, gridMax);
}

void Grid::initGrid(Vector3f min, Vector3f max, float gridSpacing) {
    // Find how many cells in each dimension grid should be
    float gridWidth = int(fabs(max.x() - min.x()) / gridSpacing);
    float gridHeight = int(fabs(max.y() - min.y()) / gridSpacing);
    float gridDepth = int(fabs(max.z() - min.z()) / gridSpacing);

    // Loop through each cell and create a GridNode at given position
    // (Currently one GridNode is created for bottom left corner of each grid cell.
    // Not sure if it's supposed to be one node at every single intersection of not?)
    for (int w = 0; w < gridWidth; w++) {
        for (int h = 0; h < gridHeight; h++) {
            for (int d = 0; d < gridDepth; d++) {
                float x = min.x() + gridSpacing * w;
                float y = min.y() + gridSpacing * h;
                float z = min.z() + gridSpacing * d;
                Vector3f position(x, y, z);
                GridNode node = GridNode(position);
                m_nodes.push_back(node);
            }
        }
    }
}

void Grid::reset() {
    vector<GridNode> newNodes;
    // Create new GridNodes with only position
    for (int i = 0; i < m_nodes.size(); i++) {
        Vector3f position = m_nodes[i].getPosition();
        GridNode node = GridNode(position);
        newNodes.push_back(node);
    }
    // Replace old nodes with new nodes
    m_nodes = newNodes;
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
