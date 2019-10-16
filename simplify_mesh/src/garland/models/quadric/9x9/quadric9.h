#ifndef MESH_RECONSTRUCTION_QUADRIC9_H
#define MESH_RECONSTRUCTION_QUADRIC9_H

#include "../quadric.h"

namespace garland {

    /**
     * Extended quadric error class for geometry, color and normals.
     */
    class Quadric9 : public Quadric {
    public:
        Quadric9();

        Quadric9(const Eigen::VectorXd &);

        ~Quadric9() override {}

        void reset() override;
    };
}

#endif //MESH_RECONSTRUCTION_QUADRIC9_H
