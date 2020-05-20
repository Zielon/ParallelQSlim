#ifndef MESH_RECONSTRUCTION_QUADRIC3_H
#define MESH_RECONSTRUCTION_QUADRIC3_H

#include "../quadric.h"

namespace garland {

    /**
     * Basic quadric error class for a give plane n^Tv + d = 0
     */
    class Quadric3 : public Quadric {
    public:
        Quadric3();

        Quadric3(const Eigen::VectorXd &);

        ~Quadric3() override {}

        void reset() override;
    };
}

#endif //MESH_RECONSTRUCTION_QUADRIC3_H
