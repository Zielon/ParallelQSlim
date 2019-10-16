#include <memory>

#include <gtest/gtest.h>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "mesh_utils.h"
#include "../src/garland/q_slim.h"
#include "../src/garland/models/quadric/6x6/quadric6.h"

class QuadricTest : public ::testing::Test {
protected:

    garland::Mesh *mesh = nullptr;

    void SetUp() override {

        mesh = new garland::Mesh();

        for (auto &vertex: getVertices()) {
            vertex->setQuadric(std::make_shared<garland::Quadric6>());
            mesh->insert(vertex);
        }

        for (auto &face: getFaces())
            mesh->insert(face);

        mesh->updateFaceNormals(mesh->getFaceKeys());
    }

    void TearDown() override {
        delete mesh;
    }

private:
    /**
     * The example from Garland's PHD Thesis on the page 88
     */
    static std::vector<std::shared_ptr<garland::Vertex>> getVertices() {
        auto vertices = std::vector<std::shared_ptr<garland::Vertex>>();

        vertices.emplace_back(std::make_shared<garland::Vertex>(0, 0, 0, 0.7, 0.3, 0.3, 0));
        vertices.emplace_back(std::make_shared<garland::Vertex>(1, 0, 0, 0.7, 0.4, 0.3, 1));
        vertices.emplace_back(std::make_shared<garland::Vertex>(2, 1, 0, 0.5, 0.5, 0.3, 2));
        vertices.emplace_back(std::make_shared<garland::Vertex>(2, 2, 0, 0.3, 0.5, 0.3, 3));
        vertices.emplace_back(std::make_shared<garland::Vertex>(1, 2, 0, 0.3, 0.4, 0.3, 4));
        vertices.emplace_back(std::make_shared<garland::Vertex>(0, 1, 0, 0.5, 0.3, 0.3, 5));
        vertices.emplace_back(std::make_shared<garland::Vertex>(1, 1, 0, 0.5, 0.4, 0.3, 6));

        return vertices;
    }

    static std::vector<std::shared_ptr<garland::Face>> getFaces() {
        auto faces = std::vector<std::shared_ptr<garland::Face>>();

        faces.emplace_back(std::make_shared<garland::Face>(0, 1, 6, 0));
        faces.emplace_back(std::make_shared<garland::Face>(0, 5, 6, 1));
        faces.emplace_back(std::make_shared<garland::Face>(1, 2, 6, 2));
        faces.emplace_back(std::make_shared<garland::Face>(6, 2, 3, 3));
        faces.emplace_back(std::make_shared<garland::Face>(6, 4, 3, 4));
        faces.emplace_back(std::make_shared<garland::Face>(4, 5, 6, 5));

        return faces;
    }
};

TEST_F(QuadricTest, checkQuadric6x6) {

    boost::program_options::variables_map options;
    auto keys = mesh->getFaceKeys();
    auto *garland = new garland::QSlim(*mesh, keys, options, 1);

    mesh->updateFaceNormals(keys);

    garland->collectQuadrics<garland::Quadric6>(garland::Weighting::none);

    auto vertex = mesh->getVertices()[6];

    Eigen::MatrixXd matrix(6, 6);

    matrix <<
           0.06, 0, 0, 0, -0.59, 0,
            0, 0.23, 0, 1.15, 0, 0,
            0, 0, 6, 0, 0, 0,
            0, 1.15, 0, 5.77, 0, 0,
            -0.59, 0, 0, 0, 5.94, 0,
            0, 0, 0, 0, 0, 6;

    ASSERT_FALSE(vertex->getQuadric()->A.isApprox(matrix));

    delete garland;
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return (RUN_ALL_TESTS());
}