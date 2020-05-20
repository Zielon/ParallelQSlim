#ifndef MESH_RECONSTRUCTION_EDGE_H
#define MESH_RECONSTRUCTION_EDGE_H

#include <cmath>
#include <memory>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include <boost/heap/binomial_heap.hpp>

#include "quadric/quadric.h"
#include "quadric/3x3/quadric3.h"
#include "quadric/9x9/quadric9.h"
#include "quadric/6x6/quadric6.h"

namespace garland {

    typedef boost::tuple<int, int> EdgeKey;

    /**
     * Passive data structure for an edge
     */
    class Edge {
    public:
        struct Comparator {
            bool operator()(const std::reference_wrapper<Edge> &e1, const std::reference_wrapper<Edge> &e2) const {
                return e1.get().cost > e2.get().cost;
            }
        };

        typedef boost::heap::binomial_heap<std::reference_wrapper<Edge>, boost::heap::compare<Comparator>>::handle_type Handle;

        int v = -1;
        int u = -1;
        int faceId = -1;
        double cost = INFINITY;
        bool inHeap = false;
        Handle handle;
        std::shared_ptr<garland::Quadric> Q;
        Eigen::VectorXd optimized = Eigen::VectorXd(9);

        Edge() = default;

        Edge(int, int);

    public:
        /**
         * Unique key identifier
         * @return Boost tuple as a key
         */
        EdgeKey getKey() const;

        void setFaceId(int);

        int getFaceId() { return faceId; }

        Eigen::Vector3d getTarget();

        void setTarget(const Eigen::Vector3d &);

        void computeOptimum(const Eigen::VectorXd &, const Eigen::VectorXd &);

        bool operator<(const Edge &right) const {
            return getKey() < right.getKey();
        }

    private:
        Eigen::Vector3d target;

        void computeBorderOptimum(const Eigen::VectorXd &, const Eigen::VectorXd &);
    };
}

#endif //MESH_RECONSTRUCTION_EDGE_H
