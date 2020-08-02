#include "mesh.h"
#include "../geometry.h"

garland::Mesh::Mesh() = default;

garland::Mesh::~Mesh() {
    faces.clear();
    vertices.clear();
}

std::shared_ptr<garland::Mesh> garland::Mesh::getCopy() {
    auto mesh = std::make_shared<Mesh>();
    auto references = std::map<int, int>();

    int i = 0;
    for (auto &pair : vertices) {
        auto vertex = pair.second;
        references[vertex->getId()] = i;
        mesh->insert(std::make_shared<Vertex>(vertex->getPosition(), vertex->getNormal(), vertex->getColor(), i));
        i++;
    }

    i = 0;
    for (auto &pair : faces) {
        auto face = pair.second;
        mesh->insert(std::make_shared<Face>(
                references[face->getVertex(0)],
                references[face->getVertex(1)],
                references[face->getVertex(2)], i));
        i++;
    }

    return mesh;
}

garland::FaceRefsVector garland::Mesh::getFacesForVertex(VertexId vid) {
    auto result = std::vector<std::shared_ptr<garland::Face>>();

    for (auto key: vertices[vid]->getFaces()) {
        if (!isValidFace(key)) continue;
        result.emplace_back(faces[key]);
    }

    return result;
}

garland::FaceRefsVector garland::Mesh::getFacesAroundEdge(garland::Edge &edge) {
    auto facesU = getFacesForVertex(edge.u);
    auto facesV = getFacesForVertex(edge.v);

    garland::FaceRefsVector facesRefs;

    facesRefs.reserve(facesU.size() + facesV.size());
    facesRefs.insert(facesRefs.end(), facesU.begin(), facesU.end());
    facesRefs.insert(facesRefs.end(), facesV.begin(), facesV.end());

    return facesRefs;
}

garland::FaceRefsVector garland::Mesh::getFacesForEdge(garland::Edge &edge) {
    auto around = getFacesAroundEdge(edge);
    auto intersection = std::map<int, int>();

    for (auto &face : around) {
        if (intersection.find(face->getId()) == intersection.end()) intersection[face->getId()] = 0;
        else intersection[face->getId()] = 1;
    }

    auto result = FaceRefsVector();

    for (auto pair : intersection) {
        if (pair.second == 1)
            result.push_back(faces[pair.first]);
    }

    return result;
}

std::vector<garland::Edge> garland::Mesh::createEdges(const std::vector<FaceId> &keys) {
    std::map<EdgeKey, Edge> edges;

    for (auto &key: keys) {
        auto &face = faces[key];
        if (face->getInvalid()) continue;

        face->lock();

        for (auto &edge: face->getEdges()) {
            edge.setFaceId(face->getId());
            edges[edge.getKey()] = edge;
        }

        face->unlock();
    }

    std::vector<garland::Edge> result;
    std::transform(edges.begin(), edges.end(), std::back_inserter(result),
                   [](const auto &pair) { return pair.second; });

    return result;
}

std::vector<garland::FaceId> garland::Mesh::getFaceKeys() {
    auto keys = std::vector<FaceId>();
    for (auto &pair: faces) if (!pair.second->getInvalid()) keys.push_back(pair.first);
    return keys;
}

void garland::Mesh::insert(const std::shared_ptr<garland::Face> &face) {
    face->setInvalid(false);
    FaceId key = face->getId();
    faces[key] = face;
    for (auto i : face->getIndex()) {
        vertices[i]->addFace(key);
    }
}

void garland::Mesh::insert(const std::shared_ptr<garland::Vertex> &vertex) {
    vertex->setInvalid(false);
    vertices[vertex->getId()] = vertex;
}

garland::Faces &garland::Mesh::getFaces() {
    return faces;
}

garland::Vertices &garland::Mesh::getVertices() {
    return vertices;
}

void garland::Mesh::removeFace(const std::shared_ptr<garland::Face> &face) {
    face->lock();
    FaceId key = face->getId();
    faces[key]->setInvalid(true);
    for (auto i : face->getIndex()) {
        vertices[i]->removeFace(key);
    }
    face->unlock();
}

void garland::Mesh::removeFace(garland::FaceId key) {
    removeFace(faces[key]);
}

void garland::Mesh::removeVertex(garland::VertexId vid) {
    vertices[vid]->setInvalid(true);
}

bool garland::Mesh::isValidFace(garland::FaceId fid) {
    return faces.find(fid) != faces.end() && !faces[fid]->getInvalid();
}

bool garland::Mesh::isBorderEdge(garland::Edge &edge) {
    return vertices[edge.v]->isOnBorder() && vertices[edge.u]->isOnBorder();
}

bool garland::Mesh::isValid(garland::Face &face) {
    return isValidFace(face.getId());
}

bool garland::Mesh::isValid(garland::Edge &edge) {
    return isValidVertex(edge.v) && isValidVertex(edge.u);
}

bool garland::Mesh::isValidVertex(garland::VertexId vid) {
    return vertices.find(vid) != vertices.end() && !vertices[vid]->getInvalid();
}

bool garland::Mesh::sameCluster(garland::Edge &edge) {
    return getVertices()[edge.v]->getClusterId() == getVertices()[edge.u]->getClusterId();
}

Eigen::Vector3d garland::Mesh::getPosition(garland::VertexId vid) {
    return vertices[vid]->getPosition();
}

void garland::Mesh::updateFaceNormals(const std::vector<FaceId> &keys) {
#pragma omp parallel for
    for (int i = 0; i < int(keys.size()); i++) {
        auto &face = faces[keys[i]];
        auto p = vertices[face->getVertex(0)];
        auto q = vertices[face->getVertex(1)];
        auto r = vertices[face->getVertex(2)];
        face->setNormal(Geometry::computeNormal(p->getPosition(), q->getPosition(), r->getPosition()));
    }
}

void garland::Mesh::updateAABB(const Eigen::Vector3d &vertex) {
    for (int i = 0; i < 3; i++) {
        aabb.max[i] = vertex[i] > aabb.max[i] ? vertex[i] : aabb.max[i];
        aabb.min[i] = vertex[i] < aabb.min[i] ? vertex[i] : aabb.min[i];
    }
}

partition::AABB garland::Mesh::getAABB() {
    return aabb;
}

void garland::Mesh::update() {
    std::vector<VertexId> verticesToRemove;
    std::vector<FaceId> facesToRemove;

    for (const auto &pair: vertices) {
        auto &vertex = pair.second;
        if (vertex->getInvalid()) {
            verticesToRemove.emplace_back(vertex->getId());
            continue;
        }

        vertex->setOnBorder(false);
        vertex->setQuadric(nullptr);
        vertex->setNormal(Eigen::Vector3d(0, 0, 0));
    }

    for (const auto &pair: faces) {
        auto &face = pair.second;
        if (face->getInvalid()) {
            facesToRemove.emplace_back(face->getId());
            continue;
        }

        face->setOnBorder(false);
        face->setClustered(false);

        auto p = vertices[face->getVertex(0)];
        auto q = vertices[face->getVertex(1)];
        auto r = vertices[face->getVertex(2)];

        auto n = garland::Geometry::computeNormal(p->getPosition(), q->getPosition(), r->getPosition());

        face->setNormal(n);

        p->addNormal(n);
        q->addNormal(n);
        r->addNormal(n);
    }

    for (const auto &index: facesToRemove) faces.erase(index);
    for (const auto &index: verticesToRemove) vertices.erase(index);

    updateBorders();
}

void garland::Mesh::reindex() {

    /**
     * Reindex vertices. Start enumerating from 0
     */
    int newIndex = 0;
    for (auto &key : vertices) {
        auto oldIndex = key.first;
        auto facesVertex = getFacesForVertex(oldIndex);
        for (auto &face: facesVertex) {
            auto index = face->getIndex();
            for (int i = 0; i < 3; i++) {
                if (index[i] == oldIndex) {
                    face->setVertex(i, newIndex);
                }
            }
        }
        newIndex++;
    }
}

void garland::Mesh::updateBorders() {
    auto keys = std::vector<FaceId>();
    boost::copy(faces | boost::adaptors::map_keys, std::back_inserter(keys));
#pragma omp parallel for
    for (unsigned int i = 0; i < keys.size(); i++) {
        auto &face = faces[keys[i]];
        for (auto &edge: face->getEdges())
            Geometry::checkBorder(*this, edge);
    }
}
