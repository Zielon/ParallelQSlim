#ifndef MESH_RECONSTRUCTION_CLUSTER_H
#define MESH_RECONSTRUCTION_CLUSTER_H

#include <vector>

#include "aabb.h"

namespace partition {

    template<typename T>
    class Cluster {
    public:
        int id;
        AABB aabb;
        std::vector<T> elements;
        Eigen::Vector3d color;
    };
}

#endif //MESH_RECONSTRUCTION_CLUSTER_H
