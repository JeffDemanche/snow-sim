#include "mesh.h"

#include <iostream>
#include <fstream>

#include <QFileInfo>
#include <QString>

#define TINYOBJLOADER_IMPLEMENTATION
#include "util/tiny_obj_loader.h"

using namespace Eigen;
using namespace std;

void Mesh::initFromVectors(const std::vector<Vector3f> &vertices,
           const std::vector<Vector3i> &faces)
{
    _vertices = vertices;
    _faces = faces;
}

void Mesh::loadFromFile(const std::string &filePath)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    QFileInfo info(QString(filePath.c_str()));
    std::string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err,
                                info.absoluteFilePath().toStdString().c_str(), (info.absolutePath().toStdString() + "/").c_str(), true);
    if(!err.empty()) {
        std::cerr << err << std::endl;
    }

    if(!ret) {
        std::cerr << "Failed to load/parse .obj file" << std::endl;
        return;
    }

    for(size_t s = 0; s < shapes.size(); s++) {
        size_t index_offset = 0;
        for(size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            unsigned int fv = shapes[s].mesh.num_face_vertices[f];

            Vector3i face;
            for(size_t v = 0; v < fv; v++) {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

                face[v] = idx.vertex_index;

            }
            _faces.push_back(face);

            index_offset += fv;
        }
    }
    for(size_t i = 0; i < attrib.vertices.size(); i += 3) {
	_vertices.emplace_back(attrib.vertices[i], attrib.vertices[i + 1], attrib.vertices[i + 2]);
    }

    std::cout << "Loaded " << _faces.size() << " faces and " << _vertices.size() << " vertices" << std::endl;
}

void Mesh::saveToFile(const std::string &filePath)
{
    std::ofstream outfile;
    outfile.open(filePath);

    // Write vertices
    for (size_t i = 0; i < _vertices.size(); i++)
    {
        const Vector3f &v = _vertices[i];
        outfile << "v " << v[0] << " " << v[1] << " " << v[2] << std::endl;
    }

    // Write faces
    for (size_t i = 0; i < _faces.size(); i++)
    {
        const Vector3i &f = _faces[i];
        outfile << "f " << (f[0]+1) << " " << (f[1]+1) << " " << (f[2]+1) << std::endl;
    }

    outfile.close();
}

void Mesh::buildBoundingBox() {

    if (_vertices.size() == 0) {
        std::cerr << "Can't generate points in mesh volume, no verts on mesh" << std::endl;
    }

    // Generate bounding box for mesh.
    Vector3f bbMin = _vertices[0];
    Vector3f bbMax = _vertices[0];

    for (unsigned int i = 1; i < _vertices.size(); i++) {
        if (_vertices[i].x() < bbMin.x())
            bbMin.x() = _vertices[i].x();
        if (_vertices[i].x() > bbMax.x())
            bbMax.x() = _vertices[i].x();

        if (_vertices[i].y() < bbMin.y())
            bbMin.y() = _vertices[i].y();
        if (_vertices[i].y() > bbMax.y())
            bbMax.y() = _vertices[i].y();

        if (_vertices[i].z() < bbMin.z())
            bbMin.z() = _vertices[i].z();
        if (_vertices[i].z() > bbMax.z())
            bbMax.z() = _vertices[i].z();
    }
    m_bbMin = bbMin;
    m_bbMax = bbMax;
}

pair<Vector3f, Vector3f> Mesh::boundingBoxCorners() {
    return pair<Vector3f, Vector3f>(m_bbMin, m_bbMax);
}

Vector3f Mesh::randPosition()
{
    if (_vertices.size() == 0) {
        std::cerr << "Can't generate points in mesh volume, no verts on mesh" << std::endl;
    }

    // Keep generating random points within the bounding box until we get one
    // within the actual mesh.
    float randX = randomNumber(m_bbMin.x(), m_bbMax.x());
    float randY = randomNumber(m_bbMin.y(), m_bbMax.y());
    float randZ = randomNumber(m_bbMin.z(), m_bbMax.z());

    Vector3f randPoint = Vector3f(randX, randY, randZ);

    while(!pointInMesh(randPoint)) {
        float randX = randomNumber(m_bbMin.x(), m_bbMax.x());
        float randY = randomNumber(m_bbMin.y(), m_bbMax.y());
        float randZ = randomNumber(m_bbMin.z(), m_bbMax.z());

        randPoint = Vector3f(randX, randY, randZ);
    }

    return randPoint;
}

float Mesh::signedTriVolume(Vector3f p1, Vector3f p2, Vector3f p3) {
    float v321 = p3.x() * p2.y() * p1.z();
    float v231 = p2.x() * p3.y() * p1.z();
    float v312 = p3.x() * p1.y() * p2.z();
    float v132 = p1.x() * p3.y() * p2.z();
    float v213 = p2.x() * p1.y() * p3.z();
    float v123 = p1.x() * p2.y() * p3.z();
    return (1.0f / 6.0f) * (-v321 + v231 + v312 - v132 - v213 + v123);
}

float Mesh::volume() {
    // https://stackoverflow.com/questions/1406029/how-to-calculate-the-volume-of-a-3d-mesh-object-the-surface-of-which-is-made-up
    float volSum = 0;
    for (size_t i = 0; i < _faces.size(); i++) {
        volSum += signedTriVolume(_vertices[_faces[i].x()], _vertices[_faces[i].y()], _vertices[_faces[i].z()]);
    }
    return fabs(volSum);
}

bool Mesh::pointInMesh(Vector3f point)
{
    int numIntersections = 0;

    // Arbitrary.
    Vector3f rayVector = Vector3f(0, 1, 0);

    for (Vector3i face : _faces) {
        // Moller-Tumbore intersection from here:
        // https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
        const float EPSILON = 0.0000001;
        Vector3f vertex0 = _vertices[face[0]];
        Vector3f vertex1 = _vertices[face[1]];
        Vector3f vertex2 = _vertices[face[2]];
        Vector3f edge1, edge2, h, s, q;
        float a,f,u,v;
        edge1 = vertex1 - vertex0;
        edge2 = vertex2 - vertex0;
        h = rayVector.cross(edge2);
        a = edge1.dot(h);
        if (a > -EPSILON && a < EPSILON)
            continue;    // This ray is parallel to this triangle.
        f = 1.0/a;
        s = point - vertex0;
        u = f * s.dot(h);
        if (u < 0.0 || u > 1.0)
            continue;
        q = s.cross(edge1);
        v = f * rayVector.dot(q);
        if (v < 0.0 || u + v > 1.0)
            continue;
        // At this stage we can compute t to find out where the intersection point is on the line.
        float t = f * edge2.dot(q);
        if (t > EPSILON) // ray intersection
        {
            // We don't need the intersection locations for this purpose.
            // outIntersectionPoint = point + rayVector * t;
            numIntersections++;
        }
        else // This means that there is a line intersection but not a ray intersection.
            continue;
    }

    // An odd number of intersections implies the point is inside the possibly-convex mesh.
    return numIntersections % 2 != 0;
}

float Mesh::triSign(Vector2f a, Vector2f b, Vector2f c)
{
    return (a.x() - c.x()) * (b.y() - c.y()) - (b.x() - c.x()) * (a.y()- c.y());
}

float Mesh::randomNumber(float Min, float Max) {
    return ((float(rand()) / float(RAND_MAX)) * (Max - Min)) + Min;
}
