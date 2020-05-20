#ifndef MESH_RECONSTRUCTION_GEOMETRY_H
#define MESH_RECONSTRUCTION_GEOMETRY_H

#include <Eigen/Dense>
#include <string>
#include <map>

#include "types.h"
#include "models/mesh.h"
#include "models/quadric/3x3/quadric3.h"
#include "models/quadric/6x6/quadric6.h"
#include "models/quadric/9x9/quadric9.h"

namespace garland {

    class Geometry {
    public:
        /**
         * Move vertices to an optimized target and update the underlying mesh structure
         * @param mesh The mesh structure object
         * @param edge Current edge to collapse
         */
        static bool movedToTarget(Mesh &, Edge &);

        /**
         * Heron's formula for an arbitrary triangle
         * @return triangle area
         */
        static double computeArea(
                const Eigen::Vector3d &p, const Eigen::Vector3d &q, const Eigen::Vector3d &r);

        static Eigen::Vector3d computeNormal(
                const Eigen::Vector3d &p, const Eigen::Vector3d &q, const Eigen::Vector3d &r);

        static bool flipped(Mesh &, Edge &);

        static bool checkBorder(Mesh &, Edge &);

        static void moveToCluster(Mesh &, Edge &, const std::vector<partition::AABB> &);

        /**
         * Compute a plane equation for a given face. Based on that the quadric is calculated.
         * @param face The current face to compute a plane
         * @param mesh The mesh structure object
         * @return Vector with set of proper attributes
         */
        template<typename QuadricError>
        static Eigen::VectorXd computeAttributes(
                const std::shared_ptr<garland::Vertex> &p,
                const std::shared_ptr<garland::Vertex> &q,
                const std::shared_ptr<garland::Vertex> &r) {

            // ==== QUADRIC 3X3 ====

            if (std::is_same<QuadricError, Quadric3>::value) {
                // n^Tv + d = 0 get plane normal
                auto n = computeNormal(p->getPosition(), q->getPosition(), r->getPosition());
                auto d = -(n.dot(p->getPosition()));
                auto result = Eigen::VectorXd(4);

                result << n, d;

                return result;
            }

            // ==== QUADRIC NXN ====

            int size = 0; // Size of attributes to parse geometry, color, normal

            if (std::is_same<QuadricError, Quadric6>::value) {
                size = 6;
            } else if (std::is_same<QuadricError, Quadric9>::value) {
                size = 9;
            } else {
                throw std::runtime_error("Wrong quadric class!");
            }

            Eigen::VectorXd h = q->attributes().segment(0, size) - p->attributes().segment(0, size);
            Eigen::VectorXd k = r->attributes().segment(0, size) - p->attributes().segment(0, size);

            auto e = gramSchmidt(k, h);

            Eigen::VectorXd e1 = e[0];
            Eigen::VectorXd e2 = e[1];
            Eigen::VectorXd result(3 * size);

            result << p->attributes().segment(0, size), e1, e2;

            return result;
        };

        /**
         * For each face adjacent to a given boundary edge, it computes a plane perpendicular to the
         * face through the edge. The perpendicular plane defines a boundary constraint plane
         * @return Penalty quadric
         */
        template<typename QuadricError>
        static std::shared_ptr<QuadricError> borderPenalty(Mesh &mesh, Edge &edge) {
            auto &face = mesh.getFaces()[edge.getFaceId()];

            auto v = mesh.getVertices()[edge.v]->getPosition();
            auto u = mesh.getVertices()[edge.u]->getPosition();

            auto e = v - u;
            auto n = face->getNormal();
            Eigen::Vector3d n2 = e.cross(n);
            n2.normalize();

            auto d = -(n2.dot(u));
            auto result = Eigen::VectorXd(4);

            result << n2, d;
            auto Q = std::make_shared<QuadricError>(result);
            //Q->multiply(1000);

            return Q;
        }

    private:
        /**
         * Gram-Schmidt orthogonalization process only for 2 vertices
         * @return A set of 2 orthonormal vectors
         */
        static std::vector<Eigen::VectorXd> gramSchmidt(const Eigen::VectorXd &, const Eigen::VectorXd &);
    };
}

#endif //MESH_RECONSTRUCTION_GEOMETRY_H
