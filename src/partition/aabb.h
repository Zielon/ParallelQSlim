#ifndef MESH_RECONSTRUCTION_AABB_H
#define MESH_RECONSTRUCTION_AABB_H

#include <Eigen/Dense>

namespace partition {

    /**
     * Basic class for mesh clustering
     */
    class AABB {
    public:
        int id = -1;
        Eigen::Vector3d min = Eigen::Vector3d(INFINITY, INFINITY, INFINITY);
        Eigen::Vector3d max = Eigen::Vector3d(-INFINITY, -INFINITY, -INFINITY);

        /**
         * Check if a given vertex is inside the bounding box
         * @param point Position
         * @return True if inside the bounding box
         */
        bool inside(const Eigen::Vector3d &point) {
            return point.x() >= min.x() && point.x() <= max.x() &&
                   point.y() >= min.y() && point.y() <= max.y() &&
                   point.z() >= min.z() && point.z() <= max.z();
        };
    };
}

#endif //MESH_RECONSTRUCTION_AABB_H
