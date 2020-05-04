#ifndef GRIDBOUNDCOLLIDER_H
#define GRIDBOUNDCOLLIDER_H

#include "collisionobject.h"

using namespace Eigen;

class GridBoundCollider : public CollisionObject
{
public:
    GridBoundCollider(Vector3f minBB, Vector3f maxBB, float u);

    bool insideObject(Vector3f pos);
    Vector3f normalAt(Vector3f pos);
    Vector3f getVelocity();
    float coefficientOfFriction();
    std::vector<glm::vec3> drawingData();

private:
    float m_u;

    Vector3f m_minBB;
    Vector3f m_maxBB;

};

#endif // GRIDBOUNDCOLLIDER_H
