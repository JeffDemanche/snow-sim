#ifndef COLLISIONOBJECT_H
#define COLLISIONOBJECT_H

#include <memory>
#include <iostream>
#include <vector>
#include "glm/glm.hpp"
#include <Eigen/Dense>

class CollisionObject
{
public:
    CollisionObject();
    virtual bool insideObject(Eigen::Vector3f pos){};
    virtual Eigen::Vector3f normalAt(Eigen::Vector3f pos, Eigen::Vector3f vel){};
    virtual Eigen::Vector3f getVelocity(){};
    virtual float coefficientOfFriction(){};
    virtual std::vector<glm::vec3> drawingData(){};
};

#endif // COLLISIONOBJECT_H
