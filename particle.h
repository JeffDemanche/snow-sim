#ifndef PARTICLE_H
#define PARTICLE_H

#include <Eigen/StdVector>

using namespace Eigen;

class Particle
{
public:
    Particle(Vector3f centerPos);

    void updatePosition(float timeStep);

private:
    Matrix3f m_elasticDeformation;
    Matrix3f m_plasticDeformation;

    Matrix3f m_deformationGradient;

    Vector3f m_position;

    float m_velocity;

    float m_mass;

    float m_volume;

};

#endif // PARTICLE_H
