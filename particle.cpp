#include "particle.h"

Particle::Particle(Vector3f centerPos, float mass, Vector3f velocity):
    m_position(centerPos), m_mass(mass), m_velocity(velocity)
{

}

Vector3f Particle::getPosition() {
    return m_position;
}

float Particle::getMass() {
    return m_mass;
}

Vector3f Particle::getVelocity() {
    return m_velocity;
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
