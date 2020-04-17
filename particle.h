#ifndef PARTICLE_H
#define PARTICLE_H

#include <Eigen/StdVector>

using namespace Eigen;

class Particle
{
public:
    Particle(Vector3f centerPos, float mass, Vector3f velocity);

    void updatePosition(float timeStep);
    Vector3f getPosition();
    Vector3f getVelocity();
    float getMass();
    float getVolume();

    void setVolume(float volume);

    void closestGridNode(int i);
    int closestGridNode();

private:
    Matrix3f m_elasticDeformation;
    Matrix3f m_plasticDeformation;

    Matrix3f m_deformationGradient;

    Vector3f m_position;

    Vector3f m_velocity;

    float m_mass;

    float m_volume;

    int m_closestGridNode;

};

#endif // PARTICLE_H
