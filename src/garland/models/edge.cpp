#include "edge.h"

garland::Edge::Edge(int v, int u) {
    this->v = v;
    this->u = u;
}

garland::EdgeKey garland::Edge::getKey() const {
    return boost::make_tuple(std::max({v, u}), std::min({v, u}));
}

Eigen::Vector3d garland::Edge::getTarget() {
    return target;
}

void garland::Edge::setFaceId(int fid) {
    faceId = fid;
}

void garland::Edge::setTarget(const Eigen::Vector3d &t) {
    target = t;
}

void garland::Edge::computeOptimum(const Eigen::VectorXd &a, const Eigen::VectorXd &b) {
    Eigen::VectorXd optimum;

    if (!Q->optimize(optimum)) {
        computeBorderOptimum(a, b);
        return;
    }

    cost = Q->evaluate(optimum);
    target = optimum.segment<3>(0);
    optimized = optimum;
}

void garland::Edge::computeBorderOptimum(const Eigen::VectorXd &a, const Eigen::VectorXd &b) {
    Eigen::VectorXd optimum;
    int size = -1;

    if (typeid(*Q) == typeid(garland::Quadric3)) size = 3;
    if (typeid(*Q) == typeid(garland::Quadric6)) size = 6;
    if (typeid(*Q) == typeid(garland::Quadric9)) size = 9;

    double error1 = Q->evaluate(a.segment(0, size));
    double error2 = Q->evaluate(b.segment(0, size));
    double error3 = Q->evaluate((a.segment(0, size) + b.segment(0, size)) / 2.0);

    double error = std::min(error1, std::min(error2, error3));

    if (error1 == error) optimum = a.segment(0, size);
    if (error2 == error) optimum = b.segment(0, size);
    if (error3 == error) optimum = (a.segment(0, size) + b.segment(0, size)) / 2.0;

    cost = error;
    target = optimum.segment<3>(0);
    optimized = optimum;
}
