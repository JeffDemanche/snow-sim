#ifndef GRIDNODE_H
#define GRIDNODE_H

#include <Eigen/StdVector>

using namespace Eigen;

class GridNode
{
public:
    GridNode(Vector3f position);

    Vector3f getPosition();

private:
    float m_mass;

    Vector3f m_velocity;
    Vector3f m_newVelocity;

    Vector3f m_force;
    Vector3f m_position;

    Vector3f m_r;
    Vector3f m_s;
    Vector3f m_p;
    Vector3f m_v;
    Vector3f m_df;
    Vector3f m_dFE;
    Vector3f m_gamma;
    Vector3f m_Ar;
    Vector3f m_Ap;
    float m_beta;
    float m_alpha;
    float m_residual;
    bool m_implicitSolved;

};

#endif // GRIDNODE_H
