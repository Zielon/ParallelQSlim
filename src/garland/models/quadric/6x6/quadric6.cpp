#include "quadric6.h"

garland::Quadric6::Quadric6() {
    reset();
}

garland::Quadric6::Quadric6(const Eigen::VectorXd &attribs) {
    Eigen::VectorXd p = attribs.segment<6>(0);
    Eigen::VectorXd e1 = attribs.segment<6>(6);
    Eigen::VectorXd e2 = attribs.segment<6>(12);

    auto I = Eigen::MatrixXd::Identity(6, 6);

    A = I - (e1 * e1.transpose()) - (e2 * e2.transpose());
    b = (p.dot(e1) * e1) + (p.dot(e2) * e2) - p;
    c = p.dot(p) - pow(p.dot(e1), 2) - pow(p.dot(e2), 2);
}

void garland::Quadric6::reset() {
    A = Eigen::MatrixXd::Zero(6, 6);
    b = Eigen::VectorXd::Zero(6);
    c = 0.0;
}