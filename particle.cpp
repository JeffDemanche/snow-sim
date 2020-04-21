#include "particle.h"

Particle::Particle(Vector3f centerPos, float mass, Vector3f velocity):
    m_position(centerPos), m_mass(mass), m_velocity(velocity)
{

}

Matrix3f Particle::getElasticDeformation() {
    return m_elasticDeformation;
}

void Particle::setElasticDeformation(Matrix3f elastic) {
    m_elasticDeformation = elastic;
}

Matrix3f Particle::getPlasticDeformation() {
    return m_plasticDeformation;
}

void Particle::setPlasticDeformation(Matrix3f plastic) {
    m_plasticDeformation = plastic;
}

Matrix3f Particle::getDeformationGradient() {
    return m_deformationGradient;
}

void Particle::setDeformationGradient(Matrix3f deformation) {
    m_deformationGradient = deformation;
}

Vector3f Particle::getPosition() {
    return m_position;
}

void Particle::setPosition(Vector3f position) {
    m_position = position;
}

float Particle::getMass() {
    return m_mass;
}

Vector3f Particle::getVelocity() {
    return m_velocity;
}

void Particle::setVelocity(Vector3f velocity) {
    m_velocity = velocity;
}

float Particle::getVolume() {
    return m_volume;
}

void Particle::setVolume(float volume) {
    m_volume = volume;
}

// Methods to keep track of which gridNode index the particle is closest to; used to avoid looping over gridNodes repeatedly
void Particle::closestGridNode(int i) {
    m_closestGridNode = i;
}

int Particle::closestGridNode() {
    return m_closestGridNode;
}
