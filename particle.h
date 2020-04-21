#ifndef PARTICLE_H
#define PARTICLE_H

#include <Eigen/StdVector>

using namespace Eigen;

class Particle
{
public:
    Particle(Vector3f centerPos, float mass, Vector3f velocity);

    void updatePosition(float timeStep);

    Matrix3f getElasticDeformation();
    void setElasticDeformation(Matrix3f elastic);
    Matrix3f getPlasticDeformation();
    void setPlasticDeformation(Matrix3f plastic);
    Matrix3f getDeformationGradient();
    void setDeformationGradient(Matrix3f deformation);

    Vector3f getPosition();
    void setPosition(Vector3f position);
    Vector3f getVelocity();
    void setVelocity(Vector3f velocity);
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
