#include "gridboundcollider.h"

GridBoundCollider::GridBoundCollider(Vector3f minBB, Vector3f maxBB, float u) : CollisionObject()
{
    m_u = u;
    m_minBB = minBB;
    m_maxBB = maxBB;
}

bool GridBoundCollider::insideObject(Vector3f pos) {
    if (pos.x() < m_maxBB.x() && pos.x() > m_minBB.x()) {
        if (pos.y() < m_maxBB.y() && pos.y() > m_minBB.y()) {
            if (pos.z() < m_maxBB.z() && pos.z() > m_minBB.z()) {
                return false;
            }
        }
    }
    return true;
}

Vector3f GridBoundCollider::normalAt(Vector3f pos) {
    float distToTop = fabs(pos.y() - m_maxBB.y());
    float distToBottom = fabs(pos.y() - m_minBB.y());
    float distToFront = fabs(pos.z() - m_maxBB.z());
    float distToBack = fabs(pos.z() - m_minBB.z());
    float distToRight = fabs(pos.x() - m_maxBB.x());
    float distToLeft = fabs(pos.x() - m_minBB.x());
    std::vector<float> dists = {distToTop, distToBottom, distToFront, distToBack, distToRight, distToLeft};

    int minIndex = 0;
    for (unsigned int i = 0; i < dists.size(); i++) {
        if (dists[i] < dists[minIndex]) {
            minIndex = 0;
        }
    }

    switch(minIndex) {
    case 0:
        return Vector3f(0, -1, 0); // Top
    case 1:
        return Vector3f(0, 1, 0); // Bottom
    case 2:
        return Vector3f(0, 0, -1); // Front
    case 3:
        return Vector3f(0, 0, 1); // Back
    case 4:
        return Vector3f(-1, 0, 0); // Right
    case 5:
        return Vector3f(1, 0, 0); // Left
    }
}

Vector3f GridBoundCollider::getVelocity() {
    return Vector3f(0, 0, 0);
}

float GridBoundCollider::coefficientOfFriction() {
    return m_u;
}

std::vector<glm::vec3> GridBoundCollider::drawingData() {
    int numTriangles = 12;
    std::vector<glm::vec3> data(2 * numTriangles * 3);

    // TODO

    return data;
}

