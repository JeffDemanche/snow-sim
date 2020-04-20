#include "grid.h"
#include <chrono>
#include <iostream>
#include <set>
#include <math.h>
#include "cubecollider.h"

// DEFINE PARAMETERS HERE
const float _particleMass = 0.05;
const Vector3f _initParticleVelocity(0,0,0);
const float _gridSpacing = 0.5; // Shouldn't be less than 0.1
const Vector3f _gravity(0, -1, 0); // Haven't tuned this yet
const float _groundHeight = -2; // Location of the ground plane

Grid::Grid(Mesh snowMesh, size_t numParticles)
{
    m_gridSpacing = _gridSpacing;
    snowMesh.buildBoundingBox();
    vector<Vector3f> points = pointsFromMesh(snowMesh, numParticles);
    initParticles(points);
    pair<Vector3f, Vector3f> boundingPoints = snowMesh.boundingBoxCorners();
    pair<Vector3f, Vector3f> gridBounds = findGridBoundaries(boundingPoints.first, boundingPoints.second, _groundHeight);
    initGrid(gridBounds.first, gridBounds.second);
    initColliders();

    std::cout << "Number of grid nodes: " << m_nodes.size() << std::endl;
}

Grid::~Grid() {
    for (int i = 0; i < m_colliders.size(); i++) {
        delete m_colliders[i];
    }
}

void Grid::initColliders() {
    Ground* g = new Ground(-2, 0.6);
    m_colliders.push_back(g);
}

std::vector<CollisionObject *> Grid::getColliders() {
    return m_colliders;
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
    // Step 1. See eq. 3.1 from the masters doc
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

            // Calculate weight function
            float w = weightN(particlePos, gridNodePos);

            // Summation equation
            float particleContribution = m_particles[p].getMass() * w;
            m_nodes[nodeIndex].setMass(m_nodes[nodeIndex].getMass() + particleContribution);
        }
    }
}

float Grid::weightFunctionN(float x) {
    // Eq. 3.3
    if (fabs(x) >= 0 && fabs(x) < 1) {
       return  (0.5 * fabs(x*x*x) - (x*x) + 2.f/3.f);
    } else if (fabs(x) < 2 && fabs(x) >= 1) {
        return (-1 * 1.f/6.f * fabs(x*x*x) + (x*x) - 2.f*fabs(x) + 4.f/3.f);
    } else {
        return 0;
    }
}

float Grid::weightGradientFunctionDelN(float x) {
    // Eq. 3.5
    // x here is the distance between a particle and a grid node.
    if (fabs(x) >= 0 && fabs(x) < 1) {
       return  (x * x * 3.0 / 2.0) - (2 * fabs(x));
    } else if (fabs(x) < 2 && fabs(x) >= 1) {
        return (-0.5 * x * x) + (2 * fabs(x)) - 2;
    } else {
        return 0;
    }
}

float Grid::weightN(Vector3f particlePos, Vector3f gridNodePos) {
    // Eq. 3.2
    // The weight some particle position has on some grid node position.
    float w_x = weightFunctionN(1.f / m_gridSpacing * (particlePos.x() - gridNodePos.x()));
    float w_y = weightFunctionN(1.f / m_gridSpacing * (particlePos.y() - gridNodePos.y()));
    float w_z = weightFunctionN(1.f / m_gridSpacing * (particlePos.z() - gridNodePos.z()));

    return w_x * w_y * w_z;
}

float Grid::weightGradientDelOmega(Vector3f particlePos, Vector3f nodePos) {
    float x_dist = particlePos.x() - nodePos.x();
    float y_dist = particlePos.y() - nodePos.y();
    float z_dist = particlePos.z() - nodePos.z();

    // Eq 3.4
    float N_ix = weightGradientFunctionDelN(x_dist) * weightFunctionN(y_dist) * weightFunctionN(z_dist);
    float N_iy = weightFunctionN(x_dist) * weightGradientFunctionDelN(y_dist) * weightFunctionN(z_dist);
    float N_iz = weightFunctionN(x_dist) * weightFunctionN(y_dist) * weightGradientFunctionDelN(z_dist);

    return N_ix * N_iy * N_iz;
}

std::vector<int> Grid::getNeighboringGridNodes(Vector3i gridNodeOrigin, Vector3f particlePos) {
    // return list of m_nodes indices of grid nodes within 2*gridSpacing neighborhood of particlePos
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
    // Step 1. See eq. 3.6 from the masters doc (this requires using the calculated mass, so do that first).
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
            float w = weightN(particlePos, gridNodePos);

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
    // Step 2. See eq. 3.8. I think this only needs to be done once for the whole simulation.
    for (size_t p = 0; p < m_particles.size(); p++) {
        Particle particle = m_particles[p];

        float particleDensity = 0;
        for (size_t i = 0; i < m_nodes.size(); i++) {
            GridNode node = m_nodes[i];
            particleDensity += particle.getMass() * weightN(particle.getPosition(), node.getPosition()) / pow(m_gridSpacing, 3);
        }

        particle.setVolume(particle.getMass() / particleDensity);
    }
}

void Grid::computeGridForces()
{
    // TODO Step 3. See eq. 3.9. Do we need to compute the deformation gradient prior to this happening?
}

void Grid::updateGridVelocities(float delta_t)
{
    // TODO: TEST THIS
    // Step 4. See eq. 3.16. This updates velocities using forces we calculated last step.
    for (int i = 0; i < m_nodes.size(); i++) {
        GridNode curr = m_nodes[i];
        Vector3f v_star = curr.getVelocity() + delta_t * (1.f / curr.getMass()) * (curr.getForce() + _gravity * curr.getMass());
        curr.setNewVelocity(v_star);
    }
}

Vector3f Grid::velocityGradient(Particle particle) {
    float sum = 0;
    for (int i = 0; i < m_nodes.size(); i++) {
        //GridNode node = m_nodes[i];
        //sum +=
    }
}

void Grid::gridCollision()
{
    // TODO: TEST THIS
    // Step 5. See eq. 3.17.
    for (int i = 0; i < m_nodes.size(); i++) {
        // Loop through all colliders in the scene
        for (int c = 0; c < m_colliders.size(); c++) {
            CollisionObject* collider = m_colliders[c];

            // Check whether gridNode is intersecting with collider
            if (collider->insideObject(m_nodes[i].getPosition())) {
                Vector3f v_rel = m_nodes[i].getVelocity() - collider->getVelocity();
                Vector3f n = collider->normalAt(m_nodes[i].getPosition());
                float u = collider->coefficientOfFriction();
                float v_n = v_rel.dot(n);

                Vector3f v_rel_prime = v_rel;
                if (v_n < 0) { // Collision only applied if objects are not separating
                    Vector3f v_t = v_rel - n * v_n;
                    if (v_t.norm() <= (-u * v_n)) { // If sticking impulse is required
                        v_rel_prime = Vector3f(0,0,0);
                    } else { // Otherwise apply dynamic friction
                        v_rel_prime = v_t + u * v_n * v_t / v_t.norm();
                    }
                }
                Vector3f v_prime = v_rel_prime + collider->getVelocity(); // Transform relative velocity back to world coords
                m_nodes[i].setNewVelocity(v_prime);
            }
        }
    }
}

void Grid::explicitSolver()
{
    // TODO: Start with this before implicit solver. Just update velocity
    for (int i = 0; i < m_nodes.size(); i++) {
        m_nodes[i].setVelocity(m_nodes[i].getNewVelocity());
    }
}

void Grid::implicitSolver()
{
    // TODO Step 6. See Algorithm 1. This will likely require a lot of calculating grid node values.
}

void Grid::calculateDeformationGradient(float delta_t)
{
    // TODO Step 7. See eq. 3.33 / 3.34
    //Matrix3f tempElastic = (Matrix3f::Identity() + delta_t * weightGradientDelOmega()) *
}

void Grid::updateParticleVelocities()
{
    // TODO Step 8. See eq. 3.35.
}

void Grid::particleCollision()
{
    // TODO Step 9. See eq. 3.17.
}

void Grid::updateParticlePositions()
{
    // TODO Step 10. Eq. 3.36. Simple time step to finish.
}
