#include "quadric3.h"

garland::Quadric3::Quadric3() {
    reset();
}

garland::Quadric3::Quadric3(const Eigen::VectorXd &attribs) {
    auto n = attribs.segment<3>(0);
    auto d = attribs[3];

    A = n * n.transpose();
    b = d * n;
    c = d * d;
}

void garland::Quadric3::reset() {
    A = Eigen::Matrix3d::Zero();
    b = Eigen::Vector3d::Zero();
    c = 0.0;
}