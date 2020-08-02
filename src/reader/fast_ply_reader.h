#ifndef MESH_RECONSTRUCTION_FAST_PLY_READER_H
#define MESH_RECONSTRUCTION_FAST_PLY_READER_H

#include "fastply.h"
#include "fastply_datalayouts.h"

#include "mesh_reader.h"
#include "../garland/models/mesh.h"
#include "../garland/geometry.h"

namespace reader {

    template<class InVertex, class InFace>
    class FastPlyReader : public MeshReader<garland::Mesh, InVertex, InFace> {

    private:
        fastply::FastPly<InVertex, InFace> fp;

    public:
        void
        read(const std::string &path, garland::Mesh &mesh,
             std::function<std::shared_ptr<garland::Vertex>(InVertex, int)> parser) override {

            if (!fp.open(path))
                return;

            auto input_vertices = fp.template get<InVertex>();
            auto input_faces = fp.template get<InFace>();

            printf("[INFO] INPUT MESH\n");
            printf("    Faces: %d\n", (int) input_faces.size());
            printf("    Vertices: %d\n", (int) input_vertices.size());

            for (unsigned int i = 0; i < input_vertices.size(); i++) {
                auto vertex = parser(input_vertices[i], i);
                mesh.updateAABB(vertex->getPosition());
                mesh.insert(vertex);
            }

            for (unsigned int i = 0; i < input_faces.size(); i++) {
                const auto &input = input_faces[i];
                auto face = std::make_shared<garland::Face>(input.v1, input.v2, input.v3, i);
                auto n = garland::Geometry::computeNormal(
                        mesh.getVertices()[input.v1]->getPosition(),
                        mesh.getVertices()[input.v2]->getPosition(),
                        mesh.getVertices()[input.v3]->getPosition());

                face->setNormal(n);
                mesh.insert(face);
            }

            fp.close();
        };

        void save(const std::string &path, garland::Mesh &mesh,
                  std::function<void(std::ofstream &)> headerWrite,
                  std::function<void(std::shared_ptr<garland::Vertex>, std::ofstream &)> vertexWrite) override {

            printf("[INFO] OUTPUT MESH\n");
            printf("    Faces: %d\n", (int) mesh.getFaces().size());
            printf("    Vertices: %d\n", (int) mesh.getVertices().size());

            std::ofstream out(path);

            out << "ply" << std::endl;
            out << "format ascii 1.0" << std::endl;

            out << "element vertex " + std::to_string(mesh.getVertices().size()) << std::endl;
            headerWrite(out);

            out << "element face " + std::to_string(mesh.getFaces().size()) << std::endl;
            garland::Face::writeHeader(out);

            out << "end_header" << std::endl;

            for (auto &v : mesh.getVertices()) vertexWrite(v.second, out);
            for (auto &t : mesh.getFaces()) t.second->save(out);

            out.close();

            std::cout << "[INFO] Mesh has been saved in [" << path << "]" << std::endl;
        };
    };
}

#endif //MESH_RECONSTRUCTION_FAST_PLY_READER_H
