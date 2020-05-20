#ifndef MESH_RECONSTRUCTION_MESH_READER_H
#define MESH_RECONSTRUCTION_MESH_READER_H

#include <string>

#include "../garland/models/vertex.h"

namespace reader {

    /**
     * An interface for each mesh reader
     * @tparam Mesh template
     */
    template<class Mesh, class InVertex, class InFace>
    class MeshReader {
    public:
        /**
         * Read with a configurable lambda function for creation a vertex with selected parameters
         */
        virtual void read(const std::string &, Mesh &, std::function<std::shared_ptr<garland::Vertex>(InVertex, int)>) = 0;

        /**
         * Save vertices with properties set in the lambda functions
         */
        virtual void save(const std::string &, Mesh &,
                          std::function<void(std::ofstream &)>,
                          std::function<void(std::shared_ptr<garland::Vertex>, std::ofstream &)>) = 0;
    };
}

#endif //MESH_RECONSTRUCTION_MESH_READER_H
