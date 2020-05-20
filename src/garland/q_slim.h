#ifndef MESH_RECONSTRUCTION_Q_SLIM_H
#define MESH_RECONSTRUCTION_Q_SLIM_H

#include <utility>
#include <vector>
#include <chrono>
#include <boost/program_options/variables_map.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <boost/tuple/tuple.hpp>

#include "geometry.h"
#include "types.h"
#include "enums.h"

namespace garland {

    class QSlim {
    public:
        explicit QSlim(garland::Mesh &,
                       std::vector<garland::FaceId>,
                       boost::program_options::variables_map, int);

        ~QSlim();

    private:
        Heap heap;
        Edges edges;
        Mesh &mesh;

        std::vector<garland::FaceId> keys;
        boost::program_options::variables_map options;
        std::vector<partition::AABB> clustersAABBs;
        int currentIteration = 0;

        /**
         * Add all edges to the minimum heap with quadric cost as a weight
         */
        void buildHeap();

        /**
         * Apply contraction to the edge with the smallest cost/weight
         */
        void applyContraction();

        /**
         * Rebuild the heap when an edge cost has changed
         */
        void updateHeap(Edge &);

        /**
         * Update all errors Q(v) for vertices connected with the edge v
         * The u end has been deleted -> Edge[v, u]
         */
        void updateEdge(Edge &);

        /**
         * Get references to vertices in the map of edges
         * @return Vector of references to edges
         */
        EdgeRefsVector getEdgesForVertex(garland::VertexId);

        /**
         * Check if the optimization has converged to local minimum
         * @return True if converged
         */
        bool converged(garland::Edge &);

        /**
         * Remove an edge and update correct cluster's id for a vertex
         */
        void contract(garland::Edge &);

    public:
        static double ERROR_LEVEL;

        void setClustersAABBs(std::vector<partition::AABB> clusters) {
            clustersAABBs = std::move(clusters);
        }

        /**
         * Accumulate quadrics for each edge by adding two corresponding quadrics v and u
         * @tparam QuadricError Quadric error metric
         * @param keys Face keys used to operate on the map
         */
        template<typename QuadricError>
        void collectEdges() {
            for (auto edge : mesh.createEdges(keys)) {
                if (mesh.isBorderEdge(edge) && typeid(QuadricError) == typeid(garland::Quadric3)) {
                    auto Q = Geometry::borderPenalty<QuadricError>(mesh, edge);
                    mesh.getVertices()[edge.v]->getQuadric()->add(Q);
                    mesh.getVertices()[edge.u]->getQuadric()->add(Q);
                }

                edge.Q = std::shared_ptr<QuadricError>(new QuadricError());
                edge.Q->add(mesh.getVertices()[edge.v]->getQuadric());
                edge.Q->add(mesh.getVertices()[edge.u]->getQuadric());

                edges[edge.getKey()] = edge;
            }
        }

        /**
         * Collect all quadric errors for each vertex in the mesh
         * @tparam QuadricError Quadric error metric
         * @param keys Face keys used to operate on the map
         * @param weighting The enum for different weighting quadric strategies
         */
        template<typename QuadricError>
        void collectQuadrics(Weighting weighting) {
            for (const auto &key : keys) {
                auto face = mesh.getFaces()[key];

                auto p = mesh.getVertices()[face->getVertex(0)];
                auto q = mesh.getVertices()[face->getVertex(1)];
                auto r = mesh.getVertices()[face->getVertex(2)];

                face->setNormal(Geometry::computeNormal(p->getPosition(), q->getPosition(), r->getPosition()));

                p->normalize();
                q->normalize();
                r->normalize();

                auto attributes = Geometry::computeAttributes<QuadricError>(p, q, r);
                auto Q = std::shared_ptr<QuadricError>(new QuadricError(attributes));

                if (std::isnan(Q->c)) continue;

                switch (weighting) {
                    case area:
                        Q->multiply(Geometry::computeArea(p->getPosition(), q->getPosition(), r->getPosition()));
                    default:
                        break;
                }

                for (auto v: face->getIndex()) {
                    auto &vertex = mesh.getVertices()[v];
                    if (!vertex->getQuadric())
                        vertex->setQuadric(std::shared_ptr<QuadricError>(new QuadricError()));

                    vertex->addQuadric(Q);
                }
            }
        }

        template<typename QuadricError>
        void initialize(garland::Weighting weighting) {
            collectQuadrics<QuadricError>(weighting);
            collectEdges<QuadricError>();
            buildHeap();
        }

        /**
         * The entry method for simplification
         * @tparam QuadricError is a selected Quadric metric. Has to be derived from garland::Quadric
         * @param vm Program options
         */
        template<typename QuadricError>
        void simplify() {
            static_assert(std::is_base_of<Quadric, QuadricError>::value);

            auto start = std::chrono::system_clock::now();
            int heapSize = (int) heap.size();

            applyContraction();

            bool verbose = options["verbose"].as<bool>();
            std::chrono::duration<double> seconds = std::chrono::system_clock::now() - start;
            auto threadId = boost::lexical_cast<std::string>(boost::this_thread::get_id());
            unsigned long threadNumber = 0;
            sscanf(threadId.c_str(), "%lx", &threadNumber);

            if (verbose)
                printf("[INFO] Thread: [%lu] | heap size = %10.d | time = %.2fs\n",
                       threadNumber, heapSize, seconds.count());
        }
    };
}

#endif //MESH_RECONSTRUCTION_Q_SLIM_H
