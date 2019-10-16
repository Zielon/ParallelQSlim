#ifndef MESH_RECONSTRUCTION_QUADRIC_H
#define MESH_RECONSTRUCTION_QUADRIC_H

#include <Eigen/Dense>
#include <memory>
#include <iostream>

namespace garland {

    /**
     * Abstract class for all version of quadric error
     */
    class Quadric {
    public:

        Quadric() = default;

        Quadric(const Eigen::VectorXd &);

        virtual ~Quadric() = default;

        /**
         * The accumulated outer product of norms of faces neighbouring with a vertex
         * (nn^T)
         */
        Eigen::MatrixXd A;

        /**
         * The plane displacement along the norm
         * (dn)
         */
        Eigen::VectorXd b;

        double c = 0;

        virtual void reset() = 0;

        /**
         * Find the optimal target (a contraction point) vector
         * @return Optimized parameters
         */
        virtual bool optimize(Eigen::VectorXd &result) {
            Eigen::FullPivLU<Eigen::MatrixXd> lu = A.fullPivLu();

            if (!lu.isInvertible())
                return false;

            result = -lu.solve(b);

            return true;
        };

        /**
         * Evaluate a quadric error
         * @param v A mesh vertex
         * @return Quadric error
         */
        virtual double evaluate(const Eigen::VectorXd &v) {
            return b.dot(v) + c;
        };

        void add(const std::shared_ptr<Quadric>& q) {
            if (!q) return;
            A += q->A;
            b += q->b;
            c += q->c;
        };

        void multiply(double s) {
            A *= s;
            b *= s;
            c *= s;
        };
    };
}

#endif //MESH_RECONSTRUCTION_QUADRIC_H
