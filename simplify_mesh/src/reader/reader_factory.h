#ifndef MESH_RECONSTRUCTION_READER_FACTORY_H
#define MESH_RECONSTRUCTION_READER_FACTORY_H

#include <string>
#include <functional>
#include "fast_ply_reader.h"

namespace reader {

    enum MeshAttributes {
        geometry_color_normal,
        geometry
    };

    /**
     * Returns a lambda function with a proper reader object according to predefined mesh attributes
     */
    class ReaderFactory {
    public:
        static std::function<void(garland::Mesh &, const std::string &)> getReader(MeshAttributes attributes) {

            return [attributes](garland::Mesh &mesh, const std::string &path) {

                if (attributes == MeshAttributes::geometry) {

                    reader::FastPlyReader<navvis::VertexUntexturedReconstruction, navvis::FaceUntexturedReconstruction> reader;
                    reader.read(path, mesh, [](navvis::VertexUntexturedReconstruction v, int id) {
                        return std::make_shared<garland::Vertex>(v.x, v.y, v.z, id);
                    });

                } else if (attributes == MeshAttributes::geometry_color_normal) {

                    reader::FastPlyReader<navvis::Vertex9x9Reconstruction, navvis::FaceUntexturedReconstruction> reader;
                    reader.read(path, mesh, [](navvis::Vertex9x9Reconstruction v, int id) {
                        return std::make_shared<garland::Vertex>(
                                /* Position */ v.x, v.y, v.z,
                                /* Color */ double(v.red) / 255.0, double(v.green) / 255.0, double(v.blue) / 255.0,
                                /* Normal */ v.nx, v.ny, v.nz, id);
                    });
                }
            };
        }

        static std::function<void(garland::Mesh &, const std::string &)> getWriter(MeshAttributes attributes) {

            return [attributes](garland::Mesh &mesh, const std::string &path) {

                if (attributes == MeshAttributes::geometry) {

                    reader::FastPlyReader<navvis::VertexUntexturedReconstruction, navvis::FaceUntexturedReconstruction> reader;

                    auto writeHeader = [](std::ofstream &out) {
                        out << "property float x" << std::endl;
                        out << "property float y" << std::endl;
                        out << "property float z" << std::endl;
                    };

                    auto writeVertex = [](const std::shared_ptr<garland::Vertex> &v, std::ofstream &out) {
                        Eigen::VectorXf f = v->getPosition().cast<float>();
                        for (int i = 0; i < 3; i++) out << f[i] << " ";
                        out << std::endl;
                    };

                    reader.save(path, mesh, writeHeader, writeVertex);

                } else if (attributes == MeshAttributes::geometry_color_normal) {

                    reader::FastPlyReader<navvis::Vertex9x9Reconstruction, navvis::FaceUntexturedReconstruction> reader;

                    auto writeHeader = [](std::ofstream &out) {
                        out << "property float x" << std::endl;
                        out << "property float y" << std::endl;
                        out << "property float z" << std::endl;

                        out << "property float nx" << std::endl;
                        out << "property float ny" << std::endl;
                        out << "property float nz" << std::endl;

                        out << "property uchar red" << std::endl;
                        out << "property uchar green" << std::endl;
                        out << "property uchar blue" << std::endl;
                        out << "property uchar alpha" << std::endl;
                    };

                    auto writeVertex = [](const std::shared_ptr<garland::Vertex> &v, std::ofstream &out) {
                        Eigen::VectorXd parameters(10);
                        parameters << v->getPosition(), v->getNormal(), (v->getColor() * 255).array().round(), 255;
                        Eigen::VectorXf f = parameters.cast<float>();
                        for (int i = 0; i < 10; i++) out << f[i] << " ";
                        out << std::endl;
                    };

                    reader.save(path, mesh, writeHeader, writeVertex);
                }
            };
        }
    };
}

#endif //MESH_RECONSTRUCTION_READER_FACTORY_H
