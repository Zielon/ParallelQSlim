#include <memory>
#include <gtest/gtest.h>

#include "mesh_utils.h"

class MeshTest : public ::testing::Test {
protected:

    garland::Mesh *mesh = nullptr;

    void SetUp() override {
        mesh = Utils::getMesh();
    }

    void TearDown() override {
        delete mesh;
    }
};

TEST_F(MeshTest, getFacesForEdge) {

    auto edge = garland::Edge(10, 9);
    auto faces = mesh->getFacesForEdge(edge);

    ASSERT_TRUE(faces.size() == 1);

    auto face = faces[0];
    auto index = face->getIndex();
    ASSERT_TRUE(index[0] == 9 && index[1] == 10 && index[2] == 7);

    edge = garland::Edge(0, 3);
    faces = mesh->getFacesForEdge(edge);

    ASSERT_TRUE(faces.size() == 2);

    face = faces[0];
    index = face->getIndex();
    ASSERT_TRUE(index[0] == 0 && index[1] == 3 && index[2] == 1);
}

TEST_F(MeshTest, remove) {

    auto face = mesh->getFaces()[10];
    mesh->removeFace(face);

    auto edge = garland::Edge(10, 7);
    auto faces = mesh->getFacesForEdge(edge);

    ASSERT_TRUE(faces.size() == 1);

    face = faces[0];
    auto index = face->getIndex();
    ASSERT_TRUE(index[0] == 7 && index[1] == 10 && index[2] == 8);
}

TEST_F(MeshTest, collectIntersectionNeighbors) {

    auto edge = garland::Edge(7, 3);

    auto intersection = mesh->getFacesForEdge(edge);

    ASSERT_TRUE(intersection.size() == 2);

    auto face = intersection[1];
    auto index = face->getIndex();
    ASSERT_TRUE(index[0] == 6 && index[1] == 7 && index[2] == 3);

    face = intersection[0];
    index = face->getIndex();
    ASSERT_TRUE(index[0] == 3 && index[1] == 7 && index[2] == 5);
}

TEST_F(MeshTest, moveToTarget) {

    auto edge = garland::Edge(3, 7);
    edge.setTarget(mesh->getPosition(3));
    edge.Q = std::make_shared<garland::Quadric3>();

    garland::Geometry::movedToTarget(*mesh, edge);

    auto faces = mesh->getFacesForVertex(3);

    ASSERT_TRUE(faces.size() == 9);

    auto face = faces[0];
    auto index = face->getIndex();
    ASSERT_TRUE(index[0] == 0 && index[1] == 3 && index[2] == 1);

    edge = garland::Edge(0, 3);
    faces = mesh->getFacesForEdge(edge);

    ASSERT_TRUE(faces.size() == 2);
}

TEST_F(MeshTest, checkBorder) {

    auto edge = garland::Edge(5, 4);
    ASSERT_TRUE(garland::Geometry::checkBorder(*mesh, edge));

    edge = garland::Edge(7, 3);
    ASSERT_FALSE(garland::Geometry::checkBorder(*mesh, edge));
}

TEST_F(MeshTest, flippedTrue) {

    auto edge = garland::Edge(7, 3);
    edge.setTarget(mesh->getPosition(7));
    ASSERT_TRUE(garland::Geometry::flipped(*mesh, edge));
}

TEST_F(MeshTest, flippedFalse) {

    auto edge = garland::Edge(3, 7);
    edge.setTarget(mesh->getPosition(3));
    ASSERT_FALSE(garland::Geometry::flipped(*mesh, edge));
}