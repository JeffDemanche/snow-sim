#include "gridnode.h"

GridNode::GridNode(Vector3f position, Vector3i index):
    m_position(position), m_index(index)
{
    m_mass = 0;
}

Vector3f GridNode::getPosition() {
    return m_position;
}

Vector3i GridNode::getIndex() {
    return m_index;
}

void GridNode::setMass(float mass) {
    m_mass = mass;
}

float GridNode::getMass() {
    return m_mass;
}

void GridNode::setVelocity(Vector3f velocity) {
    m_velocity = velocity;
}

Vector3f GridNode::getVelocity() {
    return m_velocity;
}
