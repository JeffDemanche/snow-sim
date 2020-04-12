#ifndef MESH_H
#define MESH_H

#include <vector>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3i)

using namespace Eigen;

class Mesh
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void initFromVectors(const std::vector<Eigen::Vector3f> &vertices,
         const std::vector<Eigen::Vector3i> &faces);
    void loadFromFile(const std::string &filePath);
    void saveToFile(const std::string &filePath);

    /**
     * Generates a point randomly within the bounds of the mesh.
     */
    Vector3f randPosition();

private:
    std::vector<Eigen::Vector3f> _vertices;
    std::vector<Eigen::Vector3i> _faces;

    bool pointInMesh(Vector3f point);

    float triSign(Vector2f a, Vector2f b, Vector2f c);
    float randomNumber(float Min, float Max);

};

#endif // MESH_H
