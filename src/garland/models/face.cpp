#include "face.h"

void garland::Face::setVertex(int i, int vertex) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    index[i] = vertex;
}

void garland::Face::setInvalid(bool state) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    invalid = state;
}

void garland::Face::setOnBorder(bool state) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    onBorder = state;
}

void garland::Face::setId(int i) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    id = i;
}

void garland::Face::setClustered(bool flag) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    clustered = flag;
}

void garland::Face::setNormal(const Eigen::Vector3d &n) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    normal = n;
}

void garland::Face::save(std::ofstream &out) const {
    out << 3 << " " << index[0] << " " << index[1] << " " << index[2];
    out << std::endl;
}

garland::EdgeKey garland::Face::getOppositeEdge(int id) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    Edge edge;
    for (auto &i : index)
        if (i != id) {
            if (edge.u == -1) { edge.u = i; } else { edge.v = i; }
        }
    return boost::make_tuple(edge.v, edge.u);
}

bool garland::Face::hasEdge(int u, int v) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    int has = 0;
    for (auto e : {u, v})
        for (auto i: index)
            if (i == e) has++;

    return has == 2;
}

void garland::Face::reconnect(int u, int v) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    for (auto &i : index) {
        if (i == u) {
            i = v;
        }
    }
}

std::vector<garland::Edge> garland::Face::getEdges() {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    std::vector<garland::Edge> edges;
    int j = 1;
    for (int i = 0; i < 3; i++) {
        if (j == 3) j = 0;
        edges.emplace_back(Edge(index[i], index[j]));
        j++;
    }
    return edges;
}

bool garland::Face::hasVertex(int vertexId) {
    boost::lock_guard<boost::recursive_mutex> lock(mtx);

    for (auto i : index)
        if (i == vertexId)
            return true;

    return false;
}