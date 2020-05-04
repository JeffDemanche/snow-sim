#include "grid.h"
#include <chrono>
#include <iostream>
#include <set>
#include <math.h>
#include "cubecollider.h"
#include <Eigen/SVD>

// DEFINE PARAMETERS HERE
const float _targetDensity = 400.f;
const Vector3f _initParticleVelocity(0,0,0);
const float _gridSpacing = 0.035; // Shouldn't be less than 0.001 (in meters)
const Vector3f _gravity(0, -1, 0); // Should be -10 for Earth gravity
const float _groundHeight = -1; // Location of the ground plane (in meters)

// TODO Make these optional command line arguments
const float criticalCompression = 2.5E-2;
const float criticalStretch = 7.5E-3;
const float _Eo = 1.4E5; // Initial Young's Modulus
const float _v = 0.2; // Poisson's ratio
const float _hardening = 10; // Hardening coefficient

Grid::Grid(Mesh snowMesh, size_t numParticles)
{
    m_gridSpacing = _gridSpacing;
    snowMesh.buildBoundingBox();
    vector<Vector3f> points = pointsFromMesh(snowMesh, numParticles);
    m_particleMass = _targetDensity / points.size();
    initParticles(points);
    pair<Vector3f, Vector3f> boundingPoints = snowMesh.boundingBoxCorners();
    pair<Vector3f, Vector3f> gridBounds = findGridBoundaries(boundingPoints.first, boundingPoints.second, _groundHeight);

    initGrid(gridBounds.first, gridBounds.second);
    initColliders();

    std::cout << "Number of particles: " << m_particles.size() << std::endl;
    std::cout << "Number of grid nodes: " << m_nodes.size() << std::endl;
}

Grid::~Grid() {
    for (unsigned int i = 0; i < m_colliders.size(); i++) {
        delete m_colliders[i];
    }
    for (unsigned int i = 0; i < m_nodes.size(); i++) {
        delete m_nodes[i];
    }
    for (unsigned int i = 0; i < m_particles.size(); i++) {
        delete m_particles[i];
    }
}

void Grid::setCurrentFrame(int currentFrame) {
    m_currentFrame = currentFrame;
}

vector<Particle*> Grid::getParticles() {
    return m_particles;
}

vector<GridNode*> Grid::getGridNodes() {
    return m_nodes;
}

void Grid::initColliders() {
    Ground* g = new Ground(_groundHeight, 0.6);
    m_colliders.push_back(g);
}

std::vector<CollisionObject *> Grid::getColliders() {
    return m_colliders;
}

void Grid::initParticles(vector<Vector3f> points)
{
    for(int i = 0; i < points.size(); i++) {
        Vector3f p = points[i];
        Particle* particle = new Particle(p, i, m_particleMass, _initParticleVelocity);
        m_particles.push_back(particle);
        m_points.push_back(p);
    }
}

pair<Vector3f, Vector3f> Grid::findGridBoundaries(Vector3f bbMin, Vector3f bbMax, float lowestY) {
    float meshWidth = fabs(bbMin.x() - bbMax.x());
    float meshDepth = fabs(bbMin.z() - bbMax.z());
    float meshHeight = fabs(bbMin.y() - bbMax.y());

    float spaceBelowMesh = fabs(bbMin.y() - lowestY);
    float gridHeight = int((meshHeight + spaceBelowMesh * 1.5) / m_gridSpacing);

    float gridWidth = int((meshWidth * 2.f) / m_gridSpacing); //number of cells across grid should be
    float cellsOnEachSide = (gridWidth - (meshWidth / m_gridSpacing)) / 2.f; //number of cells to add on each side of mesh
    float toAddX = cellsOnEachSide * m_gridSpacing;

    float gridDepth = int((meshDepth * 2.f) / m_gridSpacing); //number of cells across grid should be
    cellsOnEachSide = (gridDepth - (meshDepth / m_gridSpacing)) / 2.f; //number of cells to add on each side of mesh
    float toAddZ = cellsOnEachSide * m_gridSpacing;

    Vector3f gridMin(bbMin.x() - toAddX, bbMax.y() - (gridHeight * m_gridSpacing), bbMin.z() - toAddZ);
    Vector3f gridMax(bbMax.x() + toAddX, bbMax.y() + 2*m_gridSpacing, bbMax.z() + toAddZ);

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
    cout << "Grid minimum: " << min.transpose() << endl;
    cout << "Grid maximum: " << max.transpose() << endl;

    for (int w = 0; w < m_gridWidth; w++) {
        for (int h = 0; h < m_gridHeight; h++) {
            for (int d = 0; d < m_gridDepth; d++) {
                float x = min.x() + m_gridSpacing * w;
                float y = min.y() + m_gridSpacing * h;
                float z = min.z() + m_gridSpacing * d;
                Vector3f position(x, y, z);
                Vector3i index(w, h, d);
                GridNode* node = new GridNode(position, index);
                m_nodes.push_back(node);
            }
        }
    }
}

GridNode* Grid::getNodeAt(int x, int y, int z) {
    return m_nodes[getNodeIndexAt(x, y, z)];
}

int Grid::getNodeIndexAt(int x, int y, int z) {
    int a = 1;
    int b = (int) m_gridWidth;
    int c = ((int) m_gridWidth) * ((int) m_gridHeight);
    return a * x + b * y + c * z;
}

void Grid::reset() {
    vector<GridNode*> newNodes;
    // Create new GridNodes with only position
    for (unsigned int i = 0; i < m_nodes.size(); i++) {
        Vector3f position = m_nodes[i]->getPosition();
        Vector3i index = m_nodes[i]->getIndex();
        GridNode* node = new GridNode(position, index);
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
    for (unsigned int n = 0; n < m_nodes.size(); n++) {
        Vector3f gridNodePos = m_nodes[n]->getPosition();
        for (unsigned int p = 0; p < m_particles.size(); p++) {
            Vector3f particlePos = m_particles[p]->getPosition();

            // Calculate weight function
            float w = weightN(particlePos, gridNodePos);

            // Summation equation
            float particleContribution = m_particles[p]->getMass() * w;
            m_nodes[n]->setMass(m_nodes[n]->getMass() + particleContribution);
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

Vector3f Grid::weightGradientDelOmega(Vector3f particlePos, Vector3f nodePos) {
    float x_dist = 1.f / m_gridSpacing * (particlePos.x() - nodePos.x());
    float y_dist = 1.f / m_gridSpacing * (particlePos.y() - nodePos.y());
    float z_dist = 1.f / m_gridSpacing * (particlePos.z() - nodePos.z());

    // Eq 3.4
    float N_ix = weightGradientFunctionDelN(x_dist) * weightFunctionN(y_dist) * weightFunctionN(z_dist);
    float N_iy = weightFunctionN(x_dist) * weightGradientFunctionDelN(y_dist) * weightFunctionN(z_dist);
    float N_iz = weightFunctionN(x_dist) * weightFunctionN(y_dist) * weightGradientFunctionDelN(z_dist);

    Vector3f weightGradient(N_ix, N_iy, N_iz);
    return weightGradient;
}

Matrix3f Grid::velocityGradient(Particle* p) {
    // Eq. 3.23, this assumes that whatever the current velocity is at a grid node has been calculated for the next time step.
    // Which should occur in step 4.

    // TODO unsure on the shape of this
    Matrix3f velGrad = Matrix3f::Zero();

    for (unsigned int i = 0; i < m_nodes.size(); i++) {
//        if (p->getIndex() == 1000 && weightGradientDelOmega(p->getPosition(), m_nodes[i]->getPosition()).norm() > 0)
//            debug("new vel: ", m_nodes[i]->getNewVelocity());
        velGrad += m_nodes[i]->getNewVelocity() * weightGradientDelOmega(p->getPosition(), m_nodes[i]->getPosition()).transpose();
    }
    return velGrad;
}

void Grid::computeGridVelocity()
{
    // Step 1. See eq. 3.6 from the masters doc (this requires using the calculated mass, so do that first).
    bool nanCheck = false;
    for (unsigned int p = 0; p < m_particles.size(); p++) {

        Vector3f particlePos = m_particles[p]->getPosition();

        for (unsigned int n = 0; n < m_nodes.size(); n++) {
            Vector3f gridNodePos = m_nodes[n]->getPosition();

            // Calculate final weight function
            float w = weightN(particlePos, gridNodePos);

            Vector3f particleContribution;
            if (m_nodes[n]->getMass() == 0) {
                particleContribution = Vector3f(0,0,0);
            } else {
                particleContribution = m_particles[p]->getVelocity() * m_particles[p]->getMass() * w / m_nodes[n]->getMass();
            }
            Vector3f newVelocity = m_nodes[n]->getVelocity() + particleContribution;
            m_nodes[n]->setVelocity(newVelocity);

            if (isnan(newVelocity.x()) || isnan(newVelocity.y()) || isnan(newVelocity.z()))
                cerr << "Step 1: GRID VELOCITY IS NAN" << endl;
        }
    }
}

void Grid::computeParticleVolumes()
{
    // Step 2. See eq. 3.8. I think this only needs to be done once for the whole simulation.
    for (size_t p = 0; p < m_particles.size(); p++) {
        Particle* particle = m_particles[p];

        float particleDensity = 0;
        for (size_t i = 0; i < m_nodes.size(); i++) {
            GridNode* node = m_nodes[i];
            particleDensity += node->getMass() * weightN(particle->getPosition(), node->getPosition()) / pow(m_gridSpacing, 3);
        }

        particle->setVolume(particle->getMass() / particleDensity);
    }
}

void Grid::computeGridForces(int thread, int numThreads)
{
    // Step 3. See eq. 3.9. Do we need to compute the deformation gradient prior to this happening?
    // For grid nodes within range
    bool nanCheck = false;
    for (unsigned int n = thread; n < m_nodes.size(); n += numThreads) {
        GridNode* curr = m_nodes[n];
        Vector3f sum = Vector3f::Zero();
        for (unsigned int p = 0; p < m_particles.size(); p++) {

            Matrix3f F_p = m_particles[p]->getPlasticDeformation();

            //float J_p = F_p.determinant();
            Matrix3f F_e = m_particles[p]->getElasticDeformation();

            //float V_p = J_p * m_particles[p]->getVolume()
            float V_p = m_particles[p]->getVolume();
            Vector3f del_w = weightGradientDelOmega(m_particles[p]->getPosition(), curr->getPosition());
            Matrix3f stress = computeStress(F_e, F_p);

//            if (p == 1000 && curr->getVelocity().norm() > 0)
//                debug("Stress: ", stress);
            Vector3f force = V_p * stress * del_w;
            sum += force;
        }
        nanCheck = (isnan(sum.x()) || isnan(sum.y()) || isnan(sum.z()));
        Vector3f force = -1 * sum;

        curr->setForce(force);
    }
    if (nanCheck)
        cerr << "Step 3: GRID FORCE IS NAN" << endl;
}

thread Grid::computeGridForcesThread(int t, int numThreads) {
    return thread(&Grid::computeGridForces, this, t, numThreads);
}

Matrix3f Grid::computeStress(Matrix3f Fe, Matrix3f Fp) {
    float Jp = Fp.determinant();
    float Je = Fe.determinant();
    float lambda_Fp = lambda(Fp, Jp);
    float mu_Fp = mu(Fp, Jp);
    Matrix3f I = Matrix3f::Identity();

    assert(!isinf(lambda_Fp));
    assert(!isinf(mu_Fp));

    assert(Fe.determinant() != 0);

    // Compute Re from Fe using polar decomposition
    JacobiSVD<MatrixXf> svd(Fe, ComputeFullU | ComputeFullV);
    Matrix3f U = svd.matrixU();
    Matrix3f V = svd.matrixV();
    Matrix3f Re = U * V.transpose();

    Matrix3f stress = ((2.f * mu_Fp) * (Fe - Re) * Fe.transpose()) + ((lambda_Fp) * (Je - 1.f) * Je * I);
    return stress;
}

float Grid::lambda(Matrix3f Fp, float Jp) {
    float lambda_o = _Eo * _v / ((1.f + _v) * (1.f - 2.f*_v));
    float result = (lambda_o * exp(_hardening * (1.f - Jp)));
    return fmin(result, lambda_o);
}

float Grid::mu(Matrix3f Fp, float Jp) {
    float mu_o = _Eo / (2.f * (1.f + _v));
    float result = (mu_o * exp(_hardening * (1.f - Jp)));
    return fmin(result, mu_o);
}

void Grid::updateGridVelocities(float delta_t)
{
    // Step 4. See eq. 3.16. This updates velocities using forces we calculated last step.
    bool nanCheck = false;
    for (unsigned int i = 0; i < m_nodes.size(); i++) {
        GridNode* curr = m_nodes[i];
        Vector3f v_star(0, 0, 0);
        if (curr->getMass() > 0) {
            v_star = curr->getVelocity() + delta_t * (1.f / curr->getMass()) * (curr->getForce() + _gravity * curr->getMass());
        }

        nanCheck = (isnan(v_star.x()) || isnan(v_star.y()) || isnan(v_star.z()));
        curr->setNewVelocity(v_star);
    }
    if (nanCheck)
        cerr << "Step 4: UPDATED GRID VELOCITY IS NAN" << endl;
}

void Grid::gridCollision()
{
    // Step 5. See eq. 3.17.
    bool nanCheck = false;
    for (unsigned int i = 0; i < m_nodes.size(); i++) {
        if (m_nodes[i]->getMass() > 0) {
            // Loop through all colliders in the scene
            for (unsigned int c = 0; c < m_colliders.size(); c++) {
                CollisionObject* collider = m_colliders[c];

                // Check whether gridNode is intersecting with collider
                if (collider->insideObject(m_nodes[i]->getPosition())) {
                    Vector3f v_rel = m_nodes[i]->getVelocity() - collider->getVelocity();
                    Vector3f n = collider->normalAt(m_nodes[i]->getPosition());
                    float u = collider->coefficientOfFriction();
                    float v_n = v_rel.dot(n);

                    Vector3f v_rel_prime = v_rel;
                    if (v_n < 0) { // Collision only applied if objects are not separating
                        Vector3f v_t = v_rel - n * v_n;
                        if (v_t.norm() <= (-u * v_n)) { // If sticking impulse is required
                            v_rel_prime = Vector3f(0, 0, 0);
                        } else { // Otherwise apply dynamic friction
                            v_rel_prime = v_t + (u * v_n * v_t / v_t.norm());
                        }
                    }
                    Vector3f v_prime = v_rel_prime + collider->getVelocity(); // Transform relative velocity back to world coords
                    nanCheck = (isnan(v_prime.x()) || isnan(v_prime.y()) || isnan(v_prime.z()));
                    m_nodes[i]->setNewVelocity(v_prime);
                }
            }
        }
    }
    if (nanCheck)
        cerr << "Step 5: UPDATED GRID VELOCITY (AFTER COLILISION) IS NAN" << endl;
}

void Grid::explicitSolver()
{
    // TODO: Start with this before implicit solver. Just update velocity
    for (unsigned int i = 0; i < m_nodes.size(); i++) {
        m_nodes[i]->setVelocity(m_nodes[i]->getNewVelocity());
    }
}

void Grid::implicitSolver()
{
    // TODO Step 6. See Algorithm 1. This will likely require a lot of calculating grid node values.
}

void Grid::calculateDeformationGradient(float delta_t)
{
    // Step 7. See eq. 3.33 / 3.34
    for (unsigned int p = 0; p < m_particles.size(); p++) {
        Particle* particle = m_particles[p];
        Matrix3f tempElastic = (Matrix3f::Identity() + delta_t * velocityGradient(particle)) * particle->getElasticDeformation();
        Matrix3f tempF = (Matrix3f::Identity() + delta_t * velocityGradient(particle)) * particle->getDeformationGradient();

        // Compute the singular value decomposition.
        JacobiSVD<Matrix3f> svd(tempElastic, ComputeFullU | ComputeFullV);
        Matrix3f U = svd.matrixU();
        Matrix3f V = svd.matrixV();

        // singularValues() returns an array of the singular values.
        Vector3f sigma = svd.singularValues();
        for (int j = 0; j < 3; j++) {   //Clamp Sigma's diagonal values
           sigma[j] = std::min(std::max(sigma[j], 1-criticalCompression), 1+criticalStretch);
        }
        // Turn sigma into a matrix
        Matrix3f matSigma = sigma.asDiagonal();
        particle->setElasticDeformation(U * matSigma * V.transpose());
        particle->setPlasticDeformation(V * matSigma.inverse() * U.transpose() * tempF);
        particle->setDeformationGradient(tempF);
    }
}

void Grid::updateParticleVelocities()
{
    // TODO Step 8. See eq. 3.35.
    // In the paper they say "we typically used alpha=0.95" so IDK.
    float alpha = 0.95;

    for (unsigned int p = 0; p < m_particles.size(); p++) {
        Particle* particle = m_particles[p];

        Vector3f v_PIC = Vector3f::Zero();
        Vector3f v_FLIP = particle->getVelocity();

        for (unsigned int i = 0; i < m_nodes.size(); i++) {
            float weight = weightN(particle->getPosition(), m_nodes[i]->getPosition());
            v_PIC += m_nodes[i]->getNewVelocity() * weight;
            v_FLIP += (m_nodes[i]->getNewVelocity() - m_nodes[i]->getVelocity()) * weight;
        }
        particle->setVelocity((1.f - alpha) * v_PIC + alpha * v_FLIP);
    }
}

void Grid::particleCollision()
{
    // Step 9. See eq. 3.17.
    for (unsigned int i = 0; i < m_particles.size(); i++) {
        // Loop through all colliders in the scene
        for (unsigned int c = 0; c < m_colliders.size(); c++) {
            CollisionObject* collider = m_colliders[c];

            // Check whether gridNode is intersecting with collider
            if (collider->insideObject(m_particles[i]->getPosition())) {
                Vector3f v_rel = m_particles[i]->getVelocity() - collider->getVelocity();
                Vector3f n = collider->normalAt(m_particles[i]->getPosition());
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
                m_particles[i]->setVelocity(v_prime);
            }
        }
    }
}

void Grid::updateParticlePositions(float delta_t)
{
    // Step 10. Eq. 3.36. Simple time step to finish.
    bool nanCheck = false;
    m_points.clear();
    for (unsigned int p = 0; p < m_particles.size(); p++) {
        if (!outOfBounds(m_particles[p])) {
            m_particles[p]->setPosition(m_particles[p]->getPosition() + delta_t * m_particles[p]->getVelocity());
        }
        m_points.push_back(m_particles[p]->getPosition());
        nanCheck = (isnan(m_particles[p]->getPosition()[0]) || isnan(m_particles[p]->getPosition()[0]) || isnan(m_particles[p]->getPosition()[0]));
    }
    if (nanCheck)
        cerr << "Step 10: PARTICLE POSITION IS NAN" << endl;
}

bool Grid::outOfBounds(Particle* p) {
    Vector3f pos = p->getPosition();
    Vector3f min = m_gridBounds.first;
    Vector3f max = m_gridBounds.second;
    bool out = false;
    if (pos.x() > max.x() || pos.x() < min.x()) {
        if (pos.y() > max.y() || pos.y() < min.y()) {
            if (pos.z() > max.z() || pos.z() < min.z()) {
                out = true;
            }
        }
    }
    return out;
}

int Grid::closestGridNode(Particle* particle) {
    float max_weight = 0.f;
    int max_index = 0;
    for (unsigned int i = 0; i < m_nodes.size(); i++) {
        float weight = weightN(particle->getPosition(), m_nodes[i]->getPosition());
        if (weight > max_weight) {
            max_weight = weight;
            max_index = i;
        }
    }
    return max_index;
}

void Grid::debugGridNodes(int num, int afterStep) {
    for (int i = 0; i < num; i++) {
        int p_index = ((i + 1.f) / (num + 1)) * (1.f * m_particles.size());
        int g_index = closestGridNode(m_particles[p_index]);
        cout << "\t\tDebug for grid node " << g_index << " (closest to particle " << p_index << ") after step  " << afterStep << ":" << endl;
        m_nodes[g_index]->debug();
    }
}

void Grid::debugParticles(int num, int afterStep) {
    for (int i = 0; i < num; i++) {
        int index = ((i + 1.f) / (num + 1)) * (1.f * m_particles.size());
        cout << "\t\tDebug for particle " << index << " after step " << afterStep << ":" << endl;
        m_particles[index]->debug();
    }
}

void Grid::debug(string label, MatrixXf m) {
    cout << label << endl;
    cout << m << endl << endl;
}

void Grid::debug(string label, float m) {
    cout << label << endl;
    cout << m << endl << endl;
}
