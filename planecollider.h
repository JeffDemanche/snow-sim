#ifndef PLANECOLLIDER_H
#define PLANECOLLIDER_H

#include "collisionobject.h"

class PlaneCollider : public CollisionObject
{
public:
    PlaneCollider(Eigen::Vector3f point, Eigen::Vector3f normal, float u);

    bool insideObject(Eigen::Vector3f pos);
    Eigen::Vector3f normalAt(Eigen::Vector3f pos, Eigen::Vector3f vel);
    Eigen::Vector3f getVelocity();
    float coefficientOfFriction();
    std::vector<glm::vec3> drawingData();

private:
    Eigen::Vector3f m_point;
    Eigen::Vector3f m_normal;
    float m_u; // Coefficient of friction
};

#endif // PLANECOLLIDER_H
