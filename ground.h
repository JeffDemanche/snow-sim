#ifndef GROUND_H
#define GROUND_H

#include "collisionobject.h"

class Ground : public CollisionObject
{
public:
    Ground(float groundHeight, float u);

    bool insideObject(Eigen::Vector3f pos);
    Eigen::Vector3f normalAt(Eigen::Vector3f pos, Eigen::Vector3f vel);
    Eigen::Vector3f getVelocity();
    float coefficientOfFriction();
    std::vector<glm::vec3> drawingData();
    void updatePosition(float delta_t);

private:
    float m_groundHeight;
    float m_u; // Coefficient of friction
};

#endif // GROUND_H
