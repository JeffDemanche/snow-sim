#include "planecollider.h"

using namespace Eigen;

PlaneCollider::PlaneCollider(Vector3f point, Vector3f normal, float u) : CollisionObject()
{
    m_point = point;
    m_normal = normal;
    m_u = u;
}

bool PlaneCollider::insideObject(Vector3f pos) {
    if (m_normal.x() != 0) {
        if (m_normal.x() > 0) { // Left wall
            return (pos.x() <= m_point.x());
        }
        if (m_normal.x() < 0) { // Right wall
            return (pos.x() >= m_point.x());
        }
    }

    if (m_normal.y() != 0) {
        if (m_normal.y() > 0) { // Floor
            return (pos.y() <= m_point.y());
        }
        if (m_normal.y() < 0) { // Ceiling
            return (pos.y() >= m_point.y());
        }
    }

    if (m_normal.z() != 0) {
        if (m_normal.z() > 0) {
            return (pos.z() <= m_point.z());
        }
        if (m_normal.z() < 0) {
            return (pos.z() >= m_point.z());
        }
    }
}

Vector3f PlaneCollider::normalAt(Vector3f pos, Vector3f vel) {
    return m_normal;
}

Vector3f PlaneCollider::getVelocity() {
    return Vector3f(0, 0, 0); // Plane is stationary
}

float PlaneCollider::coefficientOfFriction() {
    return m_u;
}

std::vector<glm::vec3> PlaneCollider::drawingData() {
    int numVertices = 6;
    std::vector<glm::vec3> data(2*numVertices);

    return data;
}
