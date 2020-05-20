#ifndef MESH_RECONSTRUCTION_BASIC_PARTITIONER_H
#define MESH_RECONSTRUCTION_BASIC_PARTITIONER_H

#include <map>
#include <random>

#include "../partitioner.h"

namespace partition {

    /**
     * Basic partitioner which divides a mesh evenly by the factor
     * For instance factor = 2 is 2x2x2, factor = 3 is 3x3x3 division
     * @tparam T Type for the cluster container
     */
    template<typename T>
    class BasicPartitioner : public Partitioner<T> {
    public:
        std::vector<Cluster<T>> getClusters(double factor, garland::Mesh &mesh) {
            auto clusters = std::vector<Cluster<T>>();
            Eigen::Vector3d step = ((mesh.getAABB().max - mesh.getAABB().min) / factor).array().abs();
            std::random_device rd;
            std::mt19937 mt(rd());
            std::uniform_real_distribution<double> dist(0.0, 255.0);
            int id = 0;
            auto aabb = AABB();
            auto meshMin = mesh.getAABB().min;

            // Create clusters for a given mesh
            for (int x = 0; x < factor; x++) {
                for (int y = 0; y < factor; y++) {
                    for (int z = 0; z < factor; z++) {

                        aabb.min.x() = meshMin.x() + x * step.x();
                        aabb.min.y() = meshMin.y() + y * step.y();
                        aabb.min.z() = meshMin.z() + z * step.z();

                        aabb.max.x() = aabb.min.x() + step.x();
                        aabb.max.y() = aabb.min.y() + step.y();
                        aabb.max.z() = aabb.min.z() + step.z();

                        auto cluster = Cluster<T>();

                        cluster.id = id++;
                        aabb.id = cluster.id;
                        cluster.aabb = aabb;
                        cluster.color = Eigen::Vector3d(dist(mt), dist(mt), dist(mt)) / 255.0;

                        clusters.push_back(cluster);
                    }
                }
            }

            // Populate clusters with elements which are inside AABB
            for (auto &tuple : mesh.getVertices()) {
                auto &vertex = tuple.second;
                for (auto &cluster: clusters) {
                    if (cluster.aabb.inside(vertex->getPosition())) {
                        cluster.elements.push_back(vertex->getId());
                        vertex->setClusterId(cluster.id);
                        //vertex->setColor(cluster.color); // Only for testing purposes
                    }
                }
            }

            return clusters;
        }
    };
}

#endif //MESH_RECONSTRUCTION_BASIC_PARTITIONER_H
