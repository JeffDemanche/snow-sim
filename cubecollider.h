#ifndef CUBECOLLIDER_H
#define CUBECOLLIDER_H

#include "collisionobject.h"

class CubeCollider : public CollisionObject
{
public:
    CubeCollider(Eigen::Vector3f center, float u, float z_rot, float scale);

    bool insideObject(Eigen::Vector3f pos);
    Eigen::Vector3f normalAt(Eigen::Vector3f pos, Eigen::Vector3f vel);
    Eigen::Vector3f getVelocity();
    float coefficientOfFriction();
    std::vector<glm::vec3> drawingData();

private:
    float m_u; // Coeffecient of friction
    Eigen::Matrix4f m_transform;
    Eigen::Matrix4f m_inverseTransform;
    float m_scale;
    Eigen::Vector3f m_center;
};

#endif // CUBECOLLIDER_H
