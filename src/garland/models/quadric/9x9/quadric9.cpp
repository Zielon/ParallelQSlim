#include "quadric9.h"

garland::Quadric9::Quadric9() {
    reset();
}

garland::Quadric9::Quadric9(const Eigen::VectorXd &attribs) {
    Eigen::VectorXd p = attribs.segment<9>(0);
    Eigen::VectorXd e1 = attribs.segment<9>(9);
    Eigen::VectorXd e2 = attribs.segment<9>(18);

    auto I = Eigen::MatrixXd::Identity(9, 9);

    A = I - (e1 * e1.transpose()) - (e2 * e2.transpose());
    b = (p.dot(e1) * e1) + (p.dot(e2) * e2) - p;
    c = p.dot(p) - pow(p.dot(e1), 2) - pow(p.dot(e2), 2);
}

void garland::Quadric9::reset() {
    A = Eigen::MatrixXd::Zero(9, 9);
    b = Eigen::VectorXd::Zero(9);
    c = 0.0;
}