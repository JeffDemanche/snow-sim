#include "gridnode.h"

GridNode::GridNode(Vector3f position, Vector3i index):
    m_position(position), m_index(index)
{
    m_mass = 0;
    m_force = Vector3f(0, 0, 0);
    m_velocity = Vector3f(0, 0, 0);
    m_newVelocity = Vector3f(0, 0, 0);
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

void GridNode::setNewVelocity(Vector3f v_star) {
    m_newVelocity = v_star;
}

Vector3f GridNode::getNewVelocity() {
    return m_newVelocity;
}

Vector3f GridNode::getForce() {
    return m_force;
}

void GridNode::setForce(Vector3f force) {
    m_force = force;
}

void GridNode::debug() {
    std::cout << "\t\t\t       Index: " << m_index.transpose() << std::endl;
    std::cout << "\t\t\t    Position: " << m_position.transpose() << std::endl;
    std::cout << "\t\t\t        Mass: " << m_mass << std::endl;
    std::cout << "\t\t\t    Velocity: " << m_velocity.transpose() << std::endl;
    std::cout << "\t\t\tNew Velocity: " << m_newVelocity.transpose() << std::endl;
    std::cout << "\t\t\t       Force: " << m_force.transpose() << std::endl;
}
