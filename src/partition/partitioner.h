#ifndef MESH_RECONSTRUCTION_PARTITIONER_H
#define MESH_RECONSTRUCTION_PARTITIONER_H

#include <vector>

#include "../garland/models/mesh.h"
#include "cluster.h"

namespace partition {

    template<typename T>
    class Partitioner {
    public:
        /**
         * Returns clusters based on a given mesh
         * @param factor The division level/factor
         * @param mesh Read mesh
         * @return Collection of clusters, nodes to consume
         */
        virtual std::vector<Cluster<T>> getClusters(double factor, garland::Mesh &mesh) = 0;
    };
}

#endif //MESH_RECONSTRUCTION_PARTITIONER_H