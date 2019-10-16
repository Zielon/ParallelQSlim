#include <boost/range/adaptor/map.hpp>
#include <boost/assign.hpp>
#include <utility>

#include "q_slim.h"

double garland::QSlim::ERROR_LEVEL = 0;

garland::QSlim::QSlim(garland::Mesh &mesh,
                      std::vector<garland::FaceId> keys,
                      boost::program_options::variables_map vm,
                      int iteration)
        : heap(Heap()), mesh(mesh), keys(std::move(keys)), options(std::move(vm)), currentIteration(iteration) {
}

garland::QSlim::~QSlim() = default;

void garland::QSlim::buildHeap() {
    for (auto &edge : edges)
        updateHeap(edge.second);
}

bool garland::QSlim::converged(garland::Edge &edge) {
    return edge.cost > ERROR_LEVEL;
}

void garland::QSlim::contract(garland::Edge &edge) {
    if (Geometry::flipped(mesh, edge)) return;
    if (!Geometry::movedToTarget(mesh, edge)) return;

    updateEdge(edge);
}

void garland::QSlim::applyContraction() {
    while (!heap.empty()) {
        auto edge = heap.top().get();

        heap.pop();
        edges.erase(edge.getKey());

        if (!mesh.isValid(edge))
            continue;

        if (!mesh.sameCluster(edge))
            continue;

        if (converged(edge))
            break;

        contract(edge);
    }
}

void garland::QSlim::updateEdge(garland::Edge &edge) {
    for (auto &ref : getEdgesForVertex(edge.v)) {
        auto &edgeToUpdate = ref.get();

        edgeToUpdate.Q->reset();
        edgeToUpdate.Q->add(mesh.getVertices()[edgeToUpdate.v]->getQuadric());
        edgeToUpdate.Q->add(mesh.getVertices()[edgeToUpdate.u]->getQuadric());

        updateHeap(edgeToUpdate);

        Geometry::moveToCluster(mesh, edgeToUpdate, clustersAABBs);
    }
}

void garland::QSlim::updateHeap(garland::Edge &edge) {
    auto v = mesh.getVertices()[edge.v];
    auto u = mesh.getVertices()[edge.u];

    if (!mesh.isValid(edge))
        return;

    edge.computeOptimum(v->attributes(), u->attributes());

    if (!edge.inHeap) {
        edge.inHeap = true;
        edge.handle = heap.push(std::ref(edge));
        return;
    }

    heap.erase(edge.handle);
    edge.handle = heap.push(std::ref(edge));
}

garland::EdgeRefsVector garland::QSlim::getEdgesForVertex(garland::VertexId id) {
    auto neighbourEdges = std::vector<std::reference_wrapper<garland::Edge>>();

    if (!mesh.isValidVertex(id))
        return neighbourEdges;

    auto faces = mesh.getFacesForVertex(id);
    auto edgeKeys = std::set<EdgeKey>();

    // Find valid keys for edges connected with the vertex
    for (const auto &face: faces) {
        for (const auto &edge: face->getEdges()) {
            edgeKeys.insert(edge.getKey());
        }
    }

    for (auto key: edgeKeys) {
        if (edges.find(key) == edges.end()) continue;

        auto &edge = edges[key];
        if (mesh.isValid(edge))
            neighbourEdges.emplace_back(std::ref(edges[key]));
    }

    return neighbourEdges;
}