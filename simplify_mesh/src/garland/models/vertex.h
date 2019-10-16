#ifndef MESH_RECONSTRUCTION_VERTEX_H
#define MESH_RECONSTRUCTION_VERTEX_H

#include <vector>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <memory>
#include <boost/thread.hpp>

#include "quadric/quadric.h"

namespace garland {
    /**
     * Passive data structure for a vertex
     */
    class Vertex {
    private:
        int id = -1;
        int clusterId = -1;
        bool invalid = false;
        bool onBorder = false;
        Eigen::Vector3d position = Eigen::Vector3d(0,0,0);
        Eigen::Vector3d normal = Eigen::Vector3d(0,0,0);
        Eigen::Vector3d color = Eigen::Vector3d(0,0,0);
        std::shared_ptr<garland::Quadric> Q;
        std::set<int> faces;
        boost::recursive_mutex mtx;

    public:
        Vertex() = default;

        Vertex(double x, double y, double z, int vid);

        Vertex(double x, double y, double z, double r, double g, double b, int vid);

        Vertex(double x, double y, double z, double r, double g, double b, double nx, double ny, double nz, int vid);

        Vertex(Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, int vid);

        void lock() { mtx.lock(); }

        void unlock() { mtx.unlock(); }

        bool tryLock() { return mtx.try_lock(); }

        void setId(int);

        void setClusterId(int);

        void setInvalid(bool);

        void setOnBorder(bool);

        void setPosition(const Eigen::Vector3d &);

        void setNormal(const Eigen::Vector3d &);

        void setColor(const Eigen::Vector3d &);

        void setQuadric(const std::shared_ptr<garland::Quadric> &);

        void addQuadric(const std::shared_ptr<garland::Quadric> &);

        void multiplyQuadric(double);

        int getClusterId() { return clusterId; }

        bool getInvalid() { return invalid; }

        bool isOnBorder() { return onBorder; }

        std::set<int> getFaces() { return faces; }

        void clearFaces() { faces.clear(); }

        int getId() { return id; }

        void addFace(int);

        int removeFace(int);

        Eigen::Vector3d getPosition() { return position; }

        Eigen::Vector3d getNormal() { return normal; }

        Eigen::Vector3d getColor() { return color; }

        std::shared_ptr<garland::Quadric> getQuadric() { return Q; }

        void save(std::ofstream &out) const;

        /**
         * Update properties in the order: position, normal, color
         */
        void update(const Eigen::VectorXd &);

        /**
         * Merge all attributes into one vector
         * @return a vector with position. normal and color, in this order
         */
        Eigen::VectorXd attributes();
    };
}

#endif //MESH_RECONSTRUCTION_VERTEX_H
