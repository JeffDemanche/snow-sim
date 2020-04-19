#include "ground.h"

using namespace Eigen;

Ground::Ground(float groundHeight, float u) : CollisionObject()
{
    m_groundHeight = groundHeight;
    m_u = u;
}

bool Ground::insideObject(Vector3f pos) {
    return (pos.y() <= m_groundHeight);
}

Vector3f Ground::normalAt(Vector3f pos) {
    return Vector3f(0, 1, 0); // Normal is always upward facing
}

Vector3f Ground::getVelocity() {
    return Vector3f(0, 0, 0); // Ground is stationary
}

float Ground::coefficientOfFriction() {
    return m_u;
}

std::vector<glm::vec3> Ground::drawingData() {
    int numVertices = 6;
    std::vector<glm::vec3> data(numVertices);

    data[0] = glm::vec3(5, m_groundHeight, 5);
    data[1] = glm::vec3(5, m_groundHeight, -5);
    data[2] = glm::vec3(-5, m_groundHeight, 5);
    data[3] = glm::vec3(-5, m_groundHeight, -5);
    data[4] = glm::vec3(-5, m_groundHeight, 5);
    data[5] = glm::vec3(5, m_groundHeight, -5);

    return data;
}
