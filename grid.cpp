#include "grid.h"
#include <chrono>
#include <iostream>
#include <set>
#include <math.h>

// DEFINE PARAMETERS HERE
const float _particleMass = 0.05;
const Vector3f _initParticleVelocity(0,0,0);
const float _gridSpacing = 0.5; // Shouldn't be less than 0.1

Grid::Grid(Mesh snowMesh, size_t numParticles)
{
    m_gridSpacing = _gridSpacing;
    snowMesh.buildBoundingBox();
    vector<Vector3f> points = pointsFromMesh(snowMesh, numParticles);
    initParticles(points);
    pair<Vector3f, Vector3f> boundingPoints = snowMesh.boundingBoxCorners();
    pair<Vector3f, Vector3f> gridBounds = findGridBoundaries(boundingPoints.first, boundingPoints.second, -2);
    initGrid(gridBounds.first, gridBounds.second);

    std::cout << "Number of grid nodes: " << m_nodes.size() << std::endl;
}

void Grid::initParticles(vector<Vector3f> points)
{
    for(Vector3f p : points) {
        Particle particle = Particle(p, _particleMass, _initParticleVelocity);
        m_particles.push_back(particle);
        m_points.push_back(p);
    }
}

pair<Vector3f, Vector3f> Grid::findGridBoundaries(Vector3f bbMin, Vector3f bbMax, float lowestY) {
    float meshWidth = fabs(bbMin.x() - bbMax.x());
    float meshDepth = fabs(bbMin.z() - bbMax.z());
    float meshHeight = fabs(bbMin.y() - bbMax.y());

    float spaceBelowMesh = fabs(bbMin.y() - lowestY);
    float gridHeight = int((meshHeight + spaceBelowMesh * 2.f) / m_gridSpacing);

    float gridWidth = int((meshWidth * 2.f) / m_gridSpacing); //number of cells across grid should be
    float cellsOnEachSide = (gridWidth - (meshWidth / m_gridSpacing)) / 2.f; //number of cells to add on each side of mesh
    float toAddX = cellsOnEachSide * m_gridSpacing;

    float gridDepth = int((meshDepth * 2.f) / m_gridSpacing); //number of cells across grid should be
    cellsOnEachSide = (gridDepth - (meshDepth / m_gridSpacing)) / 2.f; //number of cells to add on each side of mesh
    float toAddZ = cellsOnEachSide * m_gridSpacing;

    Vector3f gridMin(bbMin.x() - toAddX, bbMax.y() - (gridHeight * m_gridSpacing), bbMin.z() - toAddZ);
    Vector3f gridMax(bbMax.x() + toAddX, bbMax.y(), bbMax.z() + toAddZ);

    pair<Vector3f, Vector3f> result = pair<Vector3f, Vector3f>(gridMin, gridMax);
    m_gridBounds = result;
    m_gridWidth = fabs(gridMax.x() - gridMin.x()) / m_gridSpacing;
    m_gridHeight = fabs(gridMax.y() - gridMin.y()) / m_gridSpacing;
    m_gridDepth = fabs(gridMax.z() - gridMin.z()) / m_gridSpacing;

    return result;
}

void Grid::initGrid(Vector3f min, Vector3f max) {
    // Loop through each cell and create a GridNode at given position
    // (Currently one GridNode is created for bottom left corner of each grid cell.
    // Not sure if it's supposed to be one node at every single intersection of not?)
    for (int w = 0; w < m_gridWidth; w++) {
        for (int h = 0; h < m_gridHeight; h++) {
            for (int d = 0; d < m_gridDepth; d++) {
                float x = min.x() + m_gridSpacing * w;
                float y = min.y() + m_gridSpacing * h;
                float z = min.z() + m_gridSpacing * d;
                Vector3f position(x, y, z);
                Vector3i index(w, h, d);
                GridNode node = GridNode(position, index);
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
        Vector3i index = m_nodes[i].getIndex();
        GridNode node = GridNode(position, index);
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

pair<Vector3f, Vector3f> Grid::getGridBounds() {
    return m_gridBounds;
}

void Grid::computeGridMass()
{
    // TODO Step 1. See eq. 3.1 from the masters doc
    for (int p = 0; p < m_particles.size(); p++) {
        Vector3f particlePos = m_particles[p].getPosition();

        // Find any gridNode in range of the particle
        int originIndex = 0;
        int closestIndex = 0;

        // Loop backwards through m_nodes since most mass is at top of grid (IDEA: keep track of biggest y in particles and resize grid height accordingly with each timestep?)
        for (int n = m_nodes.size() - 1; n > -1; n--) {
            if ((m_nodes[n].getPosition() - particlePos).norm() <= m_gridSpacing) {
                originIndex = n;
                break;
            } else {
                if ((m_nodes[n].getPosition() - particlePos).norm() <= (m_nodes[closestIndex].getPosition() - particlePos).norm()) {
                    closestIndex = n;
                }
            }
        }
        if (originIndex == 0) {
            originIndex = closestIndex;
        }
        // Remember the originIndex
        m_particles[p].closestGridNode(originIndex);

        // Get indices of neighbors
        std::vector<int> inRange = getNeighboringGridNodes(m_nodes[originIndex].getIndex(), particlePos);

        // For grid nodes within range
        for (int n = 0; n < inRange.size(); n++) {
            // Calculate weight
            int nodeIndex = inRange[n];
            Vector3f gridNodePos = m_nodes[nodeIndex].getPosition();

            float w_x = weightFunctionN(1.f / m_gridSpacing * (particlePos.x() - gridNodePos.x()));
            float w_y = weightFunctionN(1.f / m_gridSpacing * (particlePos.y() - gridNodePos.y()));
            float w_z = weightFunctionN(1.f / m_gridSpacing * (particlePos.z() - gridNodePos.z()));

            // Calculate final weight function
            float w = w_x * w_y * w_z;

            // Summation equation
            float particleContribution = m_particles[p].getMass() * w;
            m_nodes[nodeIndex].setMass(m_nodes[nodeIndex].getMass() + particleContribution);
        }
    }
}

float Grid::weightFunctionN(float x) {
    if (fabs(x) >= 0 && fabs(x) < 1) {
       return  (0.5 * fabs(x*x*x) - (x*x) + 2.f/3.f);
    } else if (fabs(x) < 2 && fabs(x) >= 1) {
        return (-1 * 1.f/6.f * fabs(x*x*x) + (x*x) - 2.f*fabs(x) + 4.f/3.f);
    } else {
        return 0;
    }
}

float Grid::finalWeightN(Vector3f particlePos, Vector3f gridNodePos) {
    float w_x = weightFunctionN(1.f / m_gridSpacing * (particlePos.x() - gridNodePos.x()));
    float w_y = weightFunctionN(1.f / m_gridSpacing * (particlePos.y() - gridNodePos.y()));
    float w_z = weightFunctionN(1.f / m_gridSpacing * (particlePos.z() - gridNodePos.z()));

    return w_x * w_y * w_z;
}

std::vector<int> Grid::getNeighboringGridNodes(Vector3i gridNodeOrigin, Vector3f particlePos) {
    // return list of m_nodes indices of grid nodes within 2*gridSpacing neighborhood of particlePos
    // Check neighborhood of size 3*gridspacing since the origin grid node might be shifted
    std::set<int> neighbors;

        int i_start = max(0, gridNodeOrigin(0) - 1);
        float i_end = fmin(m_gridWidth, gridNodeOrigin(0) + 2);
        int j_start = max(0, gridNodeOrigin(1) - 1);
        float j_end = fmin(m_gridHeight, gridNodeOrigin(1) + 2);
        int k_start = max(0, gridNodeOrigin(2) - 1);
        float k_end = fmin(m_gridDepth, gridNodeOrigin(2) + 2);

        // Loop through local neighboorhood and search for gridNodes that are within range of the particle
        for (int i = i_start; i < i_end; i++) {
            for (int j = j_start; j < j_end; j++) {
                for (int k = k_start; k < k_end; k++) {
                    int listIndex = i + j * m_gridWidth + (k * m_gridWidth * m_gridHeight);
                    Vector3f neighborPos = m_nodes[listIndex].getPosition();
                    if (fabs(neighborPos.x() - particlePos.x()) <= m_gridSpacing || fabs(neighborPos.y() - particlePos.y()) <= m_gridSpacing || fabs(neighborPos.z() - particlePos.z()) <= m_gridSpacing) {
                        neighbors.insert(listIndex);
                    }
                }
            }
        }
    // Convert set to list and return
    std::vector<int> result;
    for(auto it = neighbors.begin(); it != neighbors.end(); ++it) {
        result.push_back(*it);
    }
    return result;
}

void Grid::computeGridVelocity()
{
    // TODO Step 1. See eq. 3.6 from the masters doc (this requires using the calculated mass, so do that first).
    for (int p = 0; p < m_particles.size(); p++) {

        int originIndex = m_particles[p].closestGridNode();
        Vector3f particlePos = m_particles[p].getPosition();

        // Get indices of neighbors
        std::vector<int> inRange = getNeighboringGridNodes(m_nodes[originIndex].getIndex(), particlePos);

        // For grid nodes within range
        for (int n = 0; n < inRange.size(); n++) {
            // Calculate weight function
            int nodeIndex = inRange[n];
            Vector3f gridNodePos = m_nodes[nodeIndex].getPosition();

            // Calculate final weight function
            float w = finalWeightN(particlePos, gridNodePos);

            Vector3f particleContribution;
            if (m_nodes[nodeIndex].getMass() == 0) {
                particleContribution = Vector3f(0,0,0);
            } else {
                particleContribution = m_particles[p].getVelocity() * m_particles[p].getMass() * w / m_nodes[nodeIndex].getMass();
            }
            m_nodes[nodeIndex].setVelocity(m_nodes[nodeIndex].getVelocity() + particleContribution);
        }
    }
}

void Grid::computeParticleVolumes()
{
    // TODO Step 2. See eq. 3.8. I think this only needs to be done once for the whole simulation.
    for (size_t p = 0; p < m_particles.size(); p++) {
        Particle particle = m_particles[p];

        float particleDensity = 0;
        for (size_t i = 0; i < m_nodes.size(); i++) {
            GridNode node = m_nodes[i];
            particleDensity += particle.getMass() * finalWeightN(particle.getPosition(), node.getPosition()) / pow(m_gridSpacing, 3);
        }

        particle.setVolume(particle.getMass() / particleDensity);
    }
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

void Grid::explicitSolver()
{
    // TODO: Start with this before implicit solver. Just update velocity
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
