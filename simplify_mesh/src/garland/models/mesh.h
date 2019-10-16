#ifndef MESH_RECONSTRUCTION_MESH_H
#define MESH_RECONSTRUCTION_MESH_H

#include <set>
#include <map>
#include <mutex>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/assign.hpp>
#include <boost/thread.hpp>

#include "../../partition/aabb.h"
#include "face.h"
#include "vertex.h"
#include "edge.h"

namespace garland {

    typedef int VertexId;
    typedef int FaceId;
    typedef std::map<VertexId, std::shared_ptr<garland::Vertex>> Vertices;
    typedef std::map<FaceId, std::shared_ptr<garland::Face>> Faces;
    typedef std::vector<std::shared_ptr<garland::Face>> FaceRefsVector;

    class Mesh {
    private:
        Faces faces;
        Vertices vertices;
        partition::AABB aabb;
        int id = -1;

    public:
        Mesh();

        ~Mesh();

        /**
         * This method is not thread safe and should be called from the main thread
         */
        void update();

        void reindex();

        void updateBorders();

        void insert(const std::shared_ptr<garland::Face> &);

        void insert(const std::shared_ptr<garland::Vertex> &);

        void removeFace(const std::shared_ptr<garland::Face> &);

        void removeFace(FaceId);

        void removeVertex(VertexId);

        bool isValidFace(garland::FaceId);

        bool isValidVertex(garland::VertexId);

        bool isValid(garland::Edge &);

        bool isValid(garland::Face &);

        bool sameCluster(garland::Edge &);

        bool isBorderEdge(garland::Edge &);

        void updateFaceNormals(const std::vector<FaceId> &);

        void updateAABB(const Eigen::Vector3d &);

        // GETTERS

        FaceRefsVector getFacesForVertex(VertexId);

        FaceRefsVector getFacesForEdge(Edge &);

        FaceRefsVector getFacesAroundEdge(Edge &);

        std::vector<garland::Edge> createEdges(const std::vector<FaceId> &);

        Eigen::Vector3d getPosition(VertexId);

        Faces &getFaces();

        Vertices &getVertices();

        std::vector<FaceId> getFaceKeys();

        partition::AABB getAABB();

        int getId() { return id; }

        void setId(int i) { id = i; }
    };
}

#endif //MESH_RECONSTRUCTION_MESH_H
