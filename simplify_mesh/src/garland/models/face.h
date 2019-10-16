#ifndef MESH_RECONSTRUCTION_FACE_H
#define MESH_RECONSTRUCTION_FACE_H

#include <algorithm>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <boost/thread.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>

#include "edge.h"

namespace garland {
    /**
     * Passive data structure for a face
     */
    class Face {
    private:
        int id = -1;
        int index[3]{};
        bool onBorder = false;
        bool invalid = false;
        bool clustered = false;
        Eigen::Vector3d normal;
        boost::recursive_mutex mtx;

    public:
        Face() = default;

        Face(int v1, int v2, int v3, int _id) : id(_id), index{v1, v2, v3} {}

        void lock() { mtx.lock(); }

        void unlock() { mtx.unlock(); }

        bool tryLock() { return mtx.try_lock(); }

        void setId(int);

        void setVertex(int, int);

        void setOnBorder(bool);

        void setNormal(const Eigen::Vector3d &);

        void setInvalid(bool);

        void setClustered(bool);

        int getId() { return id; }

        std::vector<int> getIndex() { return {index[0], index[1], index[2]}; }

        int getVertex(int i) { return index[i]; }

        bool isOnBorder() { return onBorder; }

        bool isClustered() { return clustered; }

        bool getInvalid() { return invalid; }

        Eigen::Vector3d getNormal() { return normal; }

        void save(std::ofstream &out) const;

        EdgeKey getOppositeEdge(int);

        std::vector<Edge> getEdges();

        bool hasEdge(int, int);

        bool hasVertex(int);

        /**
         *  Every edge currently linked to u must be relinked to v
         */
        void reconnect(int, int);

        int operator[](int i) const { return index[i]; }

        static void writeHeader(std::ofstream &out) {
            out << "property list uchar int vertex_indices" << std::endl;
        }
    };
}

#endif //MESH_RECONSTRUCTION_FACE_H
