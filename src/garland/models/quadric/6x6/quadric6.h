#ifndef MESH_RECONSTRUCTION_QUADRIC6_H
#define MESH_RECONSTRUCTION_QUADRIC6_H

#include "../quadric.h"

namespace garland {

    /**
     * Extended quadric error class for geometry, color.
     */
    class Quadric6 : public Quadric {
    public:
        Quadric6();

        Quadric6(const Eigen::VectorXd &);

        ~Quadric6() override {}

        void reset() override;
    };
}

#endif //MESH_RECONSTRUCTION_QUADRIC6_H
