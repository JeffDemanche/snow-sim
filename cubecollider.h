#ifndef CUBECOLLIDER_H
#define CUBECOLLIDER_H

#include "collisionobject.h"

class CubeCollider : public CollisionObject
{
public:
    CubeCollider(Eigen::Vector3f center, float u, float z_rot, Eigen::Vector3f scale, Eigen::Vector3f velocity);

    bool insideObject(Eigen::Vector3f pos);
    Eigen::Vector3f normalAt(Eigen::Vector3f pos, Eigen::Vector3f vel);
    Eigen::Vector3f getVelocity();
    float coefficientOfFriction();
    std::vector<glm::vec3> drawingData();
    void updatePosition(float delta_t);

private:
    float m_u; // Coeffecient of friction
    Eigen::Matrix4f m_transform;
    Eigen::Matrix4f m_inverseTransform;
    Eigen::Vector3f m_scale;
    Eigen::Vector3f m_center;
    Eigen::Matrix4f m_S;
    Eigen::Matrix4f m_T;
    Eigen::Matrix4f m_R;
    Eigen::Vector3f m_velocity;
};

#endif // CUBECOLLIDER_H
