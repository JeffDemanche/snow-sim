#include "cubecollider.h"

using namespace Eigen;

CubeCollider::CubeCollider(float height, float u) : CollisionObject()
{
    m_u = u;
    Matrix4f R;
    R << cos(M_PI / 4.f), -sin(M_PI / 4.f), 0, 0,
            sin(M_PI / 4.f), cos(M_PI / 4.f), 0 , 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    Matrix4f T;
    T << 1, 0, 0, 0,
            0, 1, 0, height,
            0, 0, 1, 0,
            0, 0, 0, 1;
    m_transform = T * R;
    m_inverseTransform = m_transform.inverse();
}

bool CubeCollider::insideObject(Vector3f pos) {
    // Transform pos to object space
    Vector4f obj_pos = m_inverseTransform * Vector4f(pos.x(), pos.y(), pos.z(), 1);

    // Check whether position is within bounds of cube
    if (obj_pos.x() <= 1 && obj_pos.x() >= -1) {
        if (obj_pos.y() <= 1 && obj_pos.y() >= -1) {
            if (obj_pos.z() <= 1 && obj_pos.z() >= -1) {
                return true;
            }
        }
    }
    return false;
}

Vector3f CubeCollider::normalAt(Vector3f pos) {
    // Transform pos to object space
    Vector4f obj_pos = m_inverseTransform * Vector4f(pos.x(), pos.y(), pos.z(), 1);

    // Determine which face pos is closest to and return normal to that face
    float distToTop = fabs(obj_pos.y() - 1);
    float distToBottom = fabs(obj_pos.y() + 1);
    float distToFront = fabs(obj_pos.z() - 1);
    float distToBack = fabs(obj_pos.z() + 1);
    float distToRight = fabs(obj_pos.x() - 1);
    float distToLeft = fabs(obj_pos.x() + 1);
    std::vector<float> dists = {distToTop, distToBottom, distToFront, distToBack, distToRight, distToLeft};

    int minIndex = 0;
    for (int i = 0; i < dists.size(); i++) {
        if (dists[i] < dists[minIndex]) {
            minIndex = 0;
        }
    }

    Vector3f obj_normal;
    switch(minIndex) {
    case 0:
        obj_normal = Vector3f(0, 1, 0); // Top
        break;
    case 1:
        obj_normal = Vector3f(0, -1, 0); // Bottom
        break;
    case 2:
        obj_normal = Vector3f(0, 0, 1); // Front
        break;
    case 3:
        obj_normal = Vector3f(0, 0, -1); // Back
        break;
    case 4:
        obj_normal = Vector3f(1, 0, 0); // Right
        break;
    case 5:
        obj_normal = Vector3f(-1, 0, 0); // Left
        break;
    }

    Vector4f ws_normal = m_transform * Vector4f(obj_normal.x(), obj_normal.y(), obj_normal.z(), 1);
    return Vector3f(ws_normal.x(), ws_normal.y(), ws_normal.z());
}

Vector3f CubeCollider::getVelocity() {
    return Vector3f(0, 0, 0);  // Cube is stationary
}

float CubeCollider::coefficientOfFriction() {
    return m_u;
}

std::vector<glm::vec3> CubeCollider::drawingData() {
    int numTriangles = 12;
    std::vector<glm::vec3> data(2 * numTriangles * 3);

    int index = 0;

    for (int i = 0; i < 6; i++) {
        Vector4f v1_prime;
        Vector4f v2_prime;
        Vector4f v3_prime;
        Vector4f v4_prime;
        Vector4f n_prime;
        // Determine 4 verts and normal for the face in world space
        switch(i) {
        case 0: // Top face
            v3_prime = m_transform * Vector4f(-0.5, 0.5, -0.5, 1);
            v4_prime = m_transform * Vector4f(0.5, 0.5, -0.5, 1);
            v2_prime = m_transform * Vector4f(0.5, 0.5, 0.5, 1);
            v1_prime = m_transform * Vector4f(-0.5, 0.5, 0.5, 1);
            n_prime = m_transform * Vector4f(0, 1, 0, 0);
            break;
        case 1: // Right face
            v1_prime = m_transform * Vector4f(0.5, 0.5, -0.5, 1);
            v2_prime = m_transform * Vector4f(0.5, 0.5, 0.5, 1);
            v4_prime = m_transform * Vector4f(0.5, -0.5, 0.5, 1);
            v3_prime = m_transform * Vector4f(0.5, -0.5, -0.5, 1);
            n_prime = m_transform * Vector4f(1, 0, 0, 0);
            break;
        case 2: // Bottom face
            v1_prime = m_transform * Vector4f(-0.5, -0.5, -0.5, 1);
            v2_prime = m_transform * Vector4f(0.5, -0.5, -0.5, 1);
            v3_prime = m_transform * Vector4f(-0.5, -0.5, 0.5, 1);
            v4_prime = m_transform * Vector4f(0.5, -0.5, 0.5, 1);
            n_prime = m_transform * Vector4f(0, -1, 0, 0);
            break;
        case 3: // Left face
            v2_prime = m_transform * Vector4f(-0.5, -0.5, -0.5, 1);
            v1_prime = m_transform * Vector4f(-0.5, 0.5, -0.5, 1);
            v3_prime = m_transform * Vector4f(-0.5, 0.5, 0.5, 1);
            v4_prime = m_transform * Vector4f(-0.5, -0.5, 0.5, 1);
            n_prime = m_transform * Vector4f(-1, 0, 0, 0);
            break;
        case 4: // Front face
            v2_prime = m_transform * Vector4f(-0.5, -0.5, 0.5, 1);
            v1_prime = m_transform * Vector4f(0.5, -0.5, 0.5, 1);
            v3_prime = m_transform * Vector4f(0.5, 0.5, 0.5, 1);
            v4_prime = m_transform * Vector4f(-0.5, 0.5, 0.5, 1);
            n_prime = m_transform * Vector4f(0, 0, 1, 0);
            break;
        case 5: // Back face
            v1_prime = m_transform * Vector4f(-0.5, -0.5, -0.5, 1);
            v2_prime = m_transform * Vector4f(0.5, -0.5, -0.5, 1);
            v3_prime = m_transform * Vector4f(-0.5, 0.5, -0.5, 1);
            v4_prime = m_transform * Vector4f(0.5, 0.5, -0.5, 1);
            n_prime = m_transform * Vector4f(0, 0, -1, 0);
            break;
        }

        glm::vec3 v1 = glm::vec3(v1_prime.x(), v1_prime.y(), v1_prime.z());
        glm::vec3 v2 = glm::vec3(v2_prime.x(), v2_prime.y(), v2_prime.z());
        glm::vec3 v3 = glm::vec3(v3_prime.x(), v3_prime.y(), v3_prime.z());
        glm::vec3 v4 = glm::vec3(v4_prime.x(), v4_prime.y(), v4_prime.z());
        glm::vec3 n = glm::vec3(n_prime.x(), n_prime.y(), n_prime.z());

        data[index] = v1;
        data[index+1] = n;
        data[index+2] = v2;
        data[index+3] = n;
        data[index+4] = v3;
        data[index+5] = n;
        data[index+6] = v3;
        data[index+7] = n;
        data[index+8] = v2;
        data[index+9] = n;
        data[index+10] = v4;
        data[index+11] = n;

        index = index + 12;
    }
    return data;
}
