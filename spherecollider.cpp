#include "spherecollider.h"

using namespace Eigen;

SphereCollider::SphereCollider(Vector3f center, float u, float z_rot, float radius, Vector3f velocity) : CollisionObject()
{
    m_center = center;
    m_u = u;
    Matrix4f R;
    R << cos(z_rot), -sin(z_rot), 0, 0,
            sin(z_rot), cos(z_rot), 0 , 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    Matrix4f T;
    T << 1, 0, 0, center.x(),
            0, 1, 0, center.y(),
            0, 0, 1, center.z(),
            0, 0, 0, 1;
    Matrix4f S;
    S << radius, 0, 0, 0,
            0, radius, 0, 0,
            0, 0, radius, 0,
            0, 0, 0, 1;
    m_R = R;
    m_T = T;
    m_S = S;
    m_transform = T * R * S;
    m_inverseTransform = m_transform.inverse();
    m_velocity = velocity;
}

void SphereCollider::updatePosition(float delta_t) {
        m_center = m_center + m_velocity * delta_t;
        m_T <<  1, 0, 0, m_center.x(),
                0, 1, 0, m_center.y(),
                0, 0, 1, m_center.z(),
                0, 0, 0, 1;
        m_transform = m_T * m_R * m_S;
        m_inverseTransform = m_transform.inverse();
}

bool SphereCollider::insideObject(Vector3f pos) {
    // Transform pos to object space
    Vector4f obj_pos = m_inverseTransform * Vector4f(pos.x(), pos.y(), pos.z(), 1);
    return ((obj_pos - Vector4f(0, 0, 0, 1)).norm() <= 0.5);
}

Vector3f SphereCollider::normalAt(Vector3f pos, Vector3f vel) {
    // Transform pos to object space
    Vector4f obj_pos = m_inverseTransform * Vector4f(pos.x(), pos.y(), pos.z(), 1);
    Vector3f obj_normal(obj_pos.x(), obj_pos.y(), obj_pos.z());
    obj_normal.normalize();

    Vector4f ws_normal = m_transform * Vector4f(obj_normal.x(), obj_normal.y(), obj_normal.z(), 0);
    Vector3f normal = Vector3f(ws_normal.x(), ws_normal.y(), ws_normal.z());
    normal.normalize();
    return normal;
}

Vector3f SphereCollider::getVelocity() {
    return m_velocity;
}

float SphereCollider::coefficientOfFriction() {
    return m_u;
}

std::vector<glm::vec3> SphereCollider::drawingData() {
    int numTriangles = 12;
    std::vector<glm::vec3> data(2 * numTriangles * 3);
    return data;
}

