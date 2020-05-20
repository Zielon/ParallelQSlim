#include "vertex.h"

#include <utility>

garland::Vertex::Vertex(double x, double y, double z, int vid) {
    position << x, y, z;
    id = vid;
}

garland::Vertex::Vertex(double x, double y, double z, double r, double g, double b, int vid) {
    position << x, y, z;
    color << r, g, b;
    id = vid;
}

garland::Vertex::Vertex(double x, double y, double z, double r, double g, double b, double nx, double ny, double nz,
                        int vid) {
    position << x, y, z;
    color << r, g, b;
    normal << nx, ny, nz;
    id = vid;
}

garland::Vertex::Vertex(Eigen::Vector3d _position, Eigen::Vector3d _normal, Eigen::Vector3d _color, int vid) {
    position = std::move(_position);
    normal = std::move(_normal);
    color = std::move(_color);
    id = vid;
}

void garland::Vertex::setId(int i) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    id = i;
}

void garland::Vertex::setClusterId(int cid) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    clusterId = cid;
}

void garland::Vertex::setInvalid(bool state) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    invalid = state;
}

void garland::Vertex::setOnBorder(bool border) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    onBorder = border;
}

void garland::Vertex::setPosition(const Eigen::Vector3d &p) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    position = p;
}

void garland::Vertex::setNormal(const Eigen::Vector3d &n) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    normal = n;
}

void garland::Vertex::setColor(const Eigen::Vector3d &c) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    color = c;
}

void garland::Vertex::setQuadric(const std::shared_ptr<garland::Quadric> &q) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    Q = q;
}

void garland::Vertex::addQuadric(const std::shared_ptr<garland::Quadric> &q) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    Q->add(q);
}

void garland::Vertex::multiplyQuadric(double s) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    Q->multiply(s);
}

void garland::Vertex::addFace(int faceId) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    faces.insert(faceId);
}

int garland::Vertex::removeFace(int faceId) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    return faces.erase(faceId);
}

void garland::Vertex::save(std::ofstream &out) const {
    Eigen::Vector3f f = position.cast<float>();
    out.write((char *) &f.x(), sizeof(float));
    out.write((char *) &f.y(), sizeof(float));
    out.write((char *) &f.z(), sizeof(float));
}

void garland::Vertex::update(const Eigen::VectorXd &attributes) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    if (attributes.size() > 0)
        position = attributes.segment<3>(0);
    if (attributes.size() > 3)
        color = attributes.segment<3>(3).array().abs();
    if (attributes.size() > 6)
        normal = attributes.segment<3>(6);
}

Eigen::VectorXd garland::Vertex::attributes() {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    Eigen::VectorXd parameters(9);
    parameters << position, color, normal;
    return parameters;
}

void garland::Vertex::addNormal(const Eigen::Vector3d &n) {
    normal += n;
}

void garland::Vertex::normalize() {
    normal.normalize();
}