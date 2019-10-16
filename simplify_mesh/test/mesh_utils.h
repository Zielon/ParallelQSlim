#pragma once

#include "../src/garland/models/mesh.h"
#include "../src/garland/geometry.h"
#include <vector>

class Utils {
public:
    static std::vector<std::shared_ptr<garland::Face>> getFaces() {
        auto faces = std::vector<std::shared_ptr<garland::Face>>();

        faces.emplace_back(std::make_shared<garland::Face>(0, 3, 1, 0));
        faces.emplace_back(std::make_shared<garland::Face>(2, 3, 0, 1));
        faces.emplace_back(std::make_shared<garland::Face>(3, 5, 4, 2));
        faces.emplace_back(std::make_shared<garland::Face>(3, 4, 1, 3));
        faces.emplace_back(std::make_shared<garland::Face>(2, 6, 3, 4));
        faces.emplace_back(std::make_shared<garland::Face>(3, 7, 5, 5));
        faces.emplace_back(std::make_shared<garland::Face>(6, 7, 3, 6));
        faces.emplace_back(std::make_shared<garland::Face>(7, 8, 5, 7));
        faces.emplace_back(std::make_shared<garland::Face>(6, 9, 7, 8));
        faces.emplace_back(std::make_shared<garland::Face>(7, 10, 8, 9));
        faces.emplace_back(std::make_shared<garland::Face>(9, 10, 7, 10));

        return faces;
    }

    static std::vector<std::shared_ptr<garland::Vertex>> getVertices() {
        auto vertices = std::vector<std::shared_ptr<garland::Vertex>>();

        vertices.emplace_back(std::make_shared<garland::Vertex>(4.75336, 2.18592, 0.416958, 0));
        vertices.emplace_back(std::make_shared<garland::Vertex>(4.75336, 2.16592, 0.417958, 1));
        vertices.emplace_back(std::make_shared<garland::Vertex>(4.73336, 2.18592, 0.420254, 2));
        vertices.emplace_back(std::make_shared<garland::Vertex>(4.73336, 2.16592, 0.418966, 3));
        vertices.emplace_back(std::make_shared<garland::Vertex>(4.75330, 2.14580, 0.418031, 4));
        vertices.emplace_back(std::make_shared<garland::Vertex>(4.73336, 2.15592, 0.417724, 5));
        vertices.emplace_back(std::make_shared<garland::Vertex>(4.71336, 2.16592, 0.417844, 6));
        vertices.emplace_back(std::make_shared<garland::Vertex>(4.71336, 2.14592, 0.418484, 7));
        vertices.emplace_back(std::make_shared<garland::Vertex>(4.71336, 2.12592, 0.419321, 8));
        vertices.emplace_back(std::make_shared<garland::Vertex>(4.69336, 2.14592, 0.418134, 9));
        vertices.emplace_back(std::make_shared<garland::Vertex>(4.69336, 2.12592, 0.418397, 10));

        return vertices;
    }

    static garland::Mesh *getMesh() {
        auto *mesh = new garland::Mesh();

        for (auto &vertex: Utils::getVertices()) {
            vertex->setQuadric(std::make_shared<garland::Quadric3>());
            mesh->insert(vertex);
        }

        for (auto &face: Utils::getFaces())
            mesh->insert(face);

        mesh->updateFaceNormals(mesh->getFaceKeys());

        return mesh;
    }
};