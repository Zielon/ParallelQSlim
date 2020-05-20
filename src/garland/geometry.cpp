#include "geometry.h"

#include <memory>

double garland::Geometry::computeArea(
        const Eigen::Vector3d &v0, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2) {

    auto a = (v0 - v1).norm();
    auto b = (v1 - v2).norm();
    auto c = (v2 - v0).norm();
    auto s = (a + b + c) / 2.0;

    return sqrt(s * (s - a) * (s - b) * (s - c));
}

Eigen::Vector3d garland::Geometry::computeNormal(
        const Eigen::Vector3d &v0, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2) {

    Eigen::Vector3d a = v1 - v0;
    Eigen::Vector3d b = v2 - v0;

    Eigen::Vector3d n = a.cross(b);
    n.normalize();
    return n;
}

bool garland::Geometry::movedToTarget(garland::Mesh &mesh, garland::Edge &edge) {
    auto v = mesh.getVertices()[edge.v];
    auto u = mesh.getVertices()[edge.u];

    if (v->isOnBorder() != u->isOnBorder())
        return false;

    auto faces = mesh.getFacesAroundEdge(edge);

    /**
     * Try to lock the neighbourhood of the edge. If try_lock returns false, it means that
     * we touched an edge already handling by a different thread on a border between two clusters
     */

    auto lockedFaces = std::vector<int>();
    auto lockedVertices = std::vector<int>();

    for (auto &face: faces) {
        if (face->tryLock())
            lockedFaces.push_back(face->getId());
        else
            break;
        for (auto i : face->getIndex())
            if (mesh.getVertices()[i]->tryLock()) lockedVertices.push_back(i);
    }

    if (lockedFaces.size() != faces.size() || lockedVertices.size() != 3 * faces.size()) {
        for (auto i : lockedFaces)
            mesh.getFaces()[i]->unlock();

        for (auto i : lockedVertices)
            mesh.getVertices()[i]->unlock();

        return false;
    }

    /**
     * End of the locking logic
     */

    if (!mesh.isValid(edge))
        return false;

    auto remove = mesh.getFacesForEdge(edge);

    // Update vertex properties
    v->update(edge.optimized);
    v->addQuadric(u->getQuadric());

    std::for_each(remove.begin(), remove.end(), [&mesh](const auto &face) { mesh.removeFace(face); });

    mesh.removeVertex(edge.u);

    for (auto &face : mesh.getFacesForVertex(edge.u)) {
        mesh.removeFace(face);
        face->reconnect(edge.u, edge.v); // Update key. Reconnect -> u to v
        face->setInvalid(false);
        for (auto i : face->getIndex()) {
            mesh.getVertices()[i]->addFace(face->getId());
        }
    }

    /**
     * Release previously obtained locks
     */

    for (auto i : lockedFaces) mesh.getFaces()[i]->unlock();
    for (auto i : lockedVertices) mesh.getVertices()[i]->unlock();

    return true;
}

bool garland::Geometry::flipped(garland::Mesh &mesh, garland::Edge &edge) {
    if (!mesh.isValid(edge))
        return false;

    auto remove = mesh.getFacesForEdge(edge);

    auto faces = mesh.getFacesForVertex(edge.v);
    auto facesU = mesh.getFacesForVertex(edge.u);
    faces.insert(faces.end(), facesU.begin(), facesU.end());
    auto silhouette = std::vector<std::shared_ptr<garland::Face>>();

    for (auto &face : faces) {
        bool add = true;
        for (auto &r : remove)
            if (face->getId() == r->getId()) {
                add = false;
                break;
            }
        if (add) silhouette.emplace_back(face);
    }

    for (auto &face: silhouette) {
        int id = face->hasVertex(edge.u) ? edge.u : edge.v;

        EdgeKey opposite = face->getOppositeEdge(id);

        Eigen::Vector3d u = mesh.getPosition(opposite.get<0>()) - edge.getTarget();
        u.normalize();

        Eigen::Vector3d v = mesh.getPosition(opposite.get<1>()) - edge.getTarget();
        v.normalize();

        if (fabs(u.dot(v)) > 0.999)
            return true;

        auto n = u.cross(v);
        n.normalize();

        auto faceNormal = computeNormal(
                mesh.getVertices()[id]->getPosition(),
                mesh.getVertices()[opposite.get<0>()]->getPosition(),
                mesh.getVertices()[opposite.get<1>()]->getPosition());

        if (n.dot(faceNormal) < 0.2)
            return true;
    }

    return false;
}

bool garland::Geometry::checkBorder(garland::Mesh &mesh, garland::Edge &edge) {
    auto faces = mesh.getFacesForEdge(edge);

    if (faces.size() == 1) {
        mesh.getVertices()[edge.u]->setOnBorder(true);
        mesh.getVertices()[edge.v]->setOnBorder(true);
        faces[0]->setOnBorder(true);
    }

    return faces.size() == 1;
}

std::vector<Eigen::VectorXd> garland::Geometry::gramSchmidt(const Eigen::VectorXd &k, const Eigen::VectorXd &h) {
    auto e1 = h / h.norm();
    auto a = k - (e1 * k.dot(e1));
    auto e2 = a / a.norm();
    return std::vector<Eigen::VectorXd>{e1, e2};
}

void garland::Geometry::moveToCluster(garland::Mesh &mesh, garland::Edge &edge,
                                      const std::vector<partition::AABB> &clusterAABBs) {

    for (auto id : {edge.v, edge.u}) {
        int clusterId = -1;
        for (auto aabb: clusterAABBs) {
            if (aabb.inside(mesh.getPosition(id))) {
                clusterId = aabb.id;
                break;
            }
        }

        if (clusterId != -1)
            mesh.getVertices()[id]->setClusterId(clusterId);
    }
}
