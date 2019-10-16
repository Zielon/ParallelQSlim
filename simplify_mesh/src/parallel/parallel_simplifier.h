#ifndef MESH_RECONSTRUCTION_PARALLEL_SIMPLIFIER_H
#define MESH_RECONSTRUCTION_PARALLEL_SIMPLIFIER_H

#include <memory>
#include <boost/lockfree/queue.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>
#include <utility>

#include "../partition/partitioner.h"
#include "../garland/models/mesh.h"
#include "../garland/q_slim.h"
#include "../smooth/taubin.hpp"

namespace parallel {

    /**
     * Class to create clusters of a mesh and consume them in a parallel fashion
     * @tparam P Partitioner which has to derive from Partitioner<E>
     * @tparam E Element of each cluster
     */
    template<template<class> class P, typename E>
    class ParallelSimplifier {
    public:
        explicit ParallelSimplifier(garland::Mesh &mesh) : masterMesh(mesh) {
            static_assert(std::is_base_of<partition::Partitioner<E>, P<E>>::value);
            partitioner = std::shared_ptr<P<E>>(new P<E>());
        }

    private:
        /**
         * Build a queue of clusters
         */
        template<typename QuadricError>
        void produce(int currentIteration) {
            clustersAABBs.clear();
            int size = options["clusters"].template as<int>();
            auto parts = partitioner->getClusters(size, masterMesh);
            auto clusters = std::vector<partition::Cluster<garland::FaceId>>();

#pragma omp declare reduction (merge : std::vector<partition::Cluster<garland::FaceId>> : omp_out.insert(omp_out.end(), omp_in.begin(), omp_in.end()))
#pragma omp parallel for reduction(merge: clusters)
            for (unsigned int i = 0; i < parts.size(); i++) {
                auto &cluster = parts[i];
                if (cluster.elements.size() == 0) continue;
                auto faceCluster = partition::Cluster<garland::FaceId>();

                faceCluster.id = cluster.id;
                faceCluster.color = cluster.color;
                faceCluster.aabb = cluster.aabb;

                /**
                 * For each vertex in a cluster get faces and add their
                 * keys to the final cluster which will be parallelly consumed
                 */
                std::set<garland::FaceId> keys;
                for (auto j : cluster.elements) {
                    for (auto &face: masterMesh.getFacesForVertex(j)) {
                        if (vote(face, cluster.id)) {
                            keys.insert(face->getId());
                            face->setClustered(true);
                        }
                    }
                }

                for (auto key: keys) faceCluster.elements.push_back(key);

                clusters.push_back(faceCluster);
                clustersAABBs.push_back(cluster.aabb);
            }

            for (auto &cluster: clusters)
                ioService.post(
                        boost::bind(&task < QuadricError > , cluster, this, clustersAABBs, options, currentIteration));
        };

        /**
         * Check to which cluster a face belongs
         * @param face
         * @param clusterId
         * @return True if 2 vertices belongs to the cluster
         */
        bool vote(const std::shared_ptr<garland::Face> &face, int clusterId) {
            auto cluster = std::vector<int>();
            for (auto i : face->getIndex())
                cluster.push_back(masterMesh.getVertices()[i]->getClusterId());

            return std::count_if(cluster.begin(), cluster.end(), [clusterId](int i) { return i == clusterId; }) >= 2;
        }

        /**
         * A task run asynchronously by a consumer
         * @tparam QuadricError Selected quadric error
         * @param cluster The faces key for consumptions
         * @param _this This reference
         * @param vm Program variables
         */
        template<typename QuadricError>
        static void task(partition::Cluster<garland::FaceId> cluster,
                         ParallelSimplifier *_this,
                         std::vector<partition::AABB> clustersAABBs,
                         boost::program_options::variables_map vm, int currentIteration) {

            std::shared_ptr<garland::QSlim> garland = nullptr;
            auto weight = vm["weight"].template as<int>();

            garland = std::make_shared<garland::QSlim>(_this->masterMesh, cluster.elements, vm, currentIteration);
            garland->setClustersAABBs(std::move(clustersAABBs));
            garland->initialize<QuadricError>(static_cast<garland::Weighting>(weight));
            garland->simplify<QuadricError>();
        }

        /**
         * Consume clusters in parallel. Join all threads and reset state.
         */
        void consume(int currentIteration) {
            auto vertices = masterMesh.getVertices().size();
            auto start = std::chrono::system_clock::now();
            auto numberOfThreads = options["threads"].template as<int>();

            restart(numberOfThreads);

            threadpool.join_all();

            masterMesh.update();

            std::chrono::duration<double> seconds = std::chrono::system_clock::now() - start;
            updateErrorLevel(currentIteration, seconds.count(), vertices);
        };

        /**
         * Based on the current iteration and the reduction level, compute the threshold level for a next
         * iteration or stop the execution.
         * @param currentIteration
         * @param seconds Time for the iteration
         * @param vertices Number of vertices before the simplification
         */
        void updateErrorLevel(int currentIteration, double seconds, double vertices) {
            int reduction = options["reduction"].template as<int>();
            double maxIterations = double(options["max-iter"].template as<int>());
            double step = (1.0 - masterMesh.getVertices().size() / vertices) * 100.0;
            double global = (masterMesh.getVertices().size() / float(inputVertices)) * 100.0;

            printf("[INFO] ---> Iteration = %-8i "
                   "| step = %14.3f%% | original = %6.3f%% "
                   "| error level = %.10f | time = %.2fs\n",
                   currentIteration + 1, step, global, garland::QSlim::ERROR_LEVEL, seconds);

            // Check if terminate a next iteration
            if (reduction > 0.0 && global < reduction) {
                iterations = 0;
                return;
            }

            /**
             *  If a user specified the reduction size, we have increase iterations to fulfill
             *  the requirements. It can loop twice as much as specified maximum iterations.
             */
            if (reduction > 0.0 && global > reduction && currentIteration == iterations - 1) {
                iterations++;
                if (iterations > 2 * maxIterations)
                    iterations = 0; // terminate
            }

            garland::QSlim::ERROR_LEVEL = getErrorLevel(currentIteration + 1);
        }

        /**
         * Remove threads with Not-a-Thread state
         * @param numberOfThreads
         */
        void restart(int numberOfThreads) {
            for (auto thread: threads) threadpool.remove_thread(thread);
            threads.clear();
            for (int i = 0; i < numberOfThreads; i++) {
                auto task = boost::bind(&boost::asio::io_service::run, &ioService);
                boost::thread *thread = threadpool.create_thread(task);
                threads.push_back(thread);
            }
        }

        double getErrorLevel(int currentIteration) {
            return 0.000000001 * pow(double(currentIteration + 3), aggressiveness);
        }

    private:
        garland::Mesh &masterMesh;
        std::shared_ptr<P<E>> partitioner;
        std::vector<boost::thread *> threads;
        std::vector<partition::AABB> clustersAABBs;
        boost::asio::io_service ioService;
        boost::thread_group threadpool;
        boost::program_options::variables_map options;

        int iterations = 0;
        double inputVertices = 0.0;
        double aggressiveness = 0.0;

    public:
        /**
         * Main parallel loop for simplification
         * @tparam QuadricError Selected error metric
         * @param vm Program options
         */
        template<typename QuadricError>
        void simplify(const boost::program_options::variables_map &vm) {
            options = vm;
            inputVertices = masterMesh.getVertices().size();
            iterations = options["max-iter"].template as<int>();
            aggressiveness = options["aggressiveness"].template as<float>();

            garland::QSlim::ERROR_LEVEL = getErrorLevel(0);

            masterMesh.updateBorders();

            printf("[START] Simplification has begun...\n");

            for (int i = 0; i < iterations; i++) {
                ioService.reset();
                produce<QuadricError>(i);
                consume(i);
            }

            ioService.stop();

            masterMesh.reindex();

            printf("[END] Simplification of vertices = %.2f%%\n",
                   (1.0 - masterMesh.getVertices().size() / inputVertices) * 100.0);
        }
    };
}

#endif //MESH_RECONSTRUCTION_PARALLEL_SIMPLIFIER_H