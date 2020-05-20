#ifndef MESH_RECONSTRUCTION_TYPES_H
#define MESH_RECONSTRUCTION_TYPES_H

#include <map>
#include <vector>
#include <boost/heap/binomial_heap.hpp>

#include "models/edge.h"

namespace garland {
    typedef boost::heap::binomial_heap<std::reference_wrapper<Edge>, boost::heap::compare<Edge::Comparator>> Heap;
    typedef std::map<garland::EdgeKey, garland::Edge> Edges;
    typedef std::vector<std::reference_wrapper<garland::Edge>> EdgeRefsVector;
}

#endif //MESH_RECONSTRUCTION_TYPES_H
