#include "particle.h"

Particle::Particle(Vector3f centerPos, float mass, Vector3f velocity):
    m_position(centerPos), m_mass(mass), m_velocity(velocity)
{
    m_elasticDeformation = Matrix3f::Identity();
    m_plasticDeformation = Matrix3f::Identity();
    m_deformationGradient = Matrix3f::Identity();

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

void Particle::debug() {
    std::cout << "\t\t\t   Position: " << m_position.transpose() << std::endl;
    std::cout << "\t\t\t       Mass: " << m_mass << std::endl;
    std::cout << "\t\t\t     Volume: " << m_volume << std::endl;
    std::cout << "\t\t\t   Velocity: " << m_velocity.transpose() << std::endl;
    std::cout << "\t\t\tDeformation: " << m_deformationGradient.row(0) << std::endl;
    std::cout << "\t\t\t             " << m_deformationGradient.row(1) << std::endl;
    std::cout << "\t\t\t             " << m_deformationGradient.row(2) << std::endl;
    std::cout << "\t\t\t    Elastic: " << m_elasticDeformation.row(0) << std::endl;
    std::cout << "\t\t\t             " << m_elasticDeformation.row(1) << std::endl;
    std::cout << "\t\t\t             " << m_elasticDeformation.row(2) << std::endl;
    std::cout << "\t\t\t    Plastic: " << m_plasticDeformation.row(0) << std::endl;
    std::cout << "\t\t\t             " << m_plasticDeformation.row(1) << std::endl;
    std::cout << "\t\t\t             " << m_plasticDeformation.row(2) << std::endl;
}
