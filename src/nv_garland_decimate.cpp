#include "timer.h"
#include "boost_argument_helpers.h"

#include <boost/filesystem.hpp>
#include <memory>
#include <boost/program_options.hpp>
#include <cstdio>
#include <cmath>
#include <iostream>

#include "reader/reader_factory.h"
#include "garland/models/quadric/3x3/quadric3.h"
#include "garland/models/quadric/6x6/quadric6.h"
#include "garland/models/quadric/9x9/quadric9.h"
#include "partition/basic/basic_partitioner.h"
#include "parallel/parallel_simplifier.h"
#include "smooth/taubin.hpp"

namespace bfs = boost::filesystem;
namespace bpo = boost::program_options;

//==============================================================================
// help function
//==============================================================================
void help(const bpo::options_description &opt) {
    std::cout << std::endl
              << "Usage:\n"
              << "     nv_simplify_mesh --in <fine_mesh> --out <simplified_mesh> [options]\n\n"
              << opt << '\n'
              << std::endl;
}

int main(int argc, char const *argv[]) {
    bpo::variables_map vm;
    bpo::options_description opts("Allowed options");

    Eigen::initParallel();

    try {
        opts.add_options()

                // --help
                ("help,h", "Produce help message")

                // --in
                ("in", bpo::value<std::string>()->required(), "Path fine input mesh")

                // --out
                ("out", bpo::value<std::string>()->required(), "Output path of simplified mesh")

                // --verbose, -v
                ("verbose,v", bpo::bool_switch()->default_value(false), "Show debug output")

                // --force, -f
                ("force,f", bpo::bool_switch()->default_value(false), "Enable file overwrite")

                // --smooth, -s
                ("smooth,s", bpo::bool_switch()->default_value(false), "Smooth the mesh using Taubin")

                // --weight, -w
                ("weight,w", bpo::value<int>()->default_value(0),
                 "Quadric error weighting strategy\n 0 = none\n 1 = area\n")

                // --reduction, -r
                ("reduction,r", bpo::value<int>()->default_value(0),
                 "The percentage reduction which we want to achieve; e.g. 10 of the input mesh")

                // --max-iter, -i
                ("max-iter,i", bpo::value<int>()->default_value(1), "Max iterations to perform")

                // --threads, -t
                ("threads,t", bpo::value<int>()->default_value(1), "Number of threads")

                // --quadric, -q
                ("quadric,q", bpo::value<int>()->default_value(3),
                 "Type of quadric metric\n 3 = [geometry]\n 6 = [geometry, color]\n 9 = [geometry, color, normal]\n")

                // --clusters, -c
                ("clusters,c", bpo::value<int>()->default_value(1),
                 "Number of clusters e.g.\n 2 will be 2x2x2=8, 3x3x3=27 clusters")

                // --attributes, -m
                ("attributes,m", bpo::value<int>()->default_value(1),
                 "Input mesh attributes\n 1 = [geometry]\n 2 = [geometry, color, normal]")

                // --aggressiveness, -a
                ("aggressiveness,a", bpo::value<float>()->default_value(3.0)->notifier(
                        boost::bind(&navvis::boost_args::check_range<float>, _1, 1.0f, 10.0f)),
                 "Aggressiveness (directly relates to the maximum permissive error) [1.0-10.0]");

        bpo::store(bpo::command_line_parser(argc, argv).options(opts).run(), vm);

        if (vm.count("help")) {
            help(opts);
            exit(EXIT_SUCCESS);
        }

        bpo::notify(vm);
    }
    catch (const std::logic_error &ex) {
        fprintf(stderr, "[ERROR] %s\n", ex.what());
        exit(EXIT_FAILURE);
    }
    catch (const std::exception &ex) {
        fprintf(stderr, "[ERROR] %s\n", ex.what());
        exit(EXIT_FAILURE);
    }

    // Too few arguments
    if (argc < 3) {
        exit(EXIT_FAILURE);
    }

    auto mesh_in = vm["in"].as<std::string>();
    if (!bfs::exists(mesh_in)) {
        fprintf(stderr, "[ERROR] Input mesh '%s' does not exist!\n", mesh_in.c_str());
        exit(EXIT_FAILURE);
    }

    bool force = vm["force"].as<bool>();

    auto mesh_out = vm["out"].as<std::string>();
    if (bfs::exists(mesh_out) && !force) {
        fprintf(stdout, "[INFO] Simplified mesh '%s' already exists. Use option '-f, --force' for file overwrite.\n",
                mesh_out.c_str());
        exit(EXIT_SUCCESS);
    } else if (bfs::exists(mesh_out) && force) {
        fprintf(stdout, "[WARN] Forced to overwrite existing mesh '%s'.\n", mesh_out.c_str());
    }

    bfs::path out_dir = bfs::path(mesh_out).parent_path();
    if (!bfs::exists(out_dir) && !bfs::create_directories(out_dir)) {
        fprintf(stderr, "[ERROR] Can not create output directory '%s'.\n", out_dir.c_str());
        exit(EXIT_FAILURE);
    }

    std::function<void(garland::Mesh &, const std::string &)> read;
    std::function<void(garland::Mesh &, const std::string &)> save;

    switch (vm["attributes"].as<int>()) {
        case 1:
            read = reader::ReaderFactory::getReader(reader::MeshAttributes::geometry);
            save = reader::ReaderFactory::getWriter(reader::MeshAttributes::geometry);
            break;
        case 2:
            read = reader::ReaderFactory::getReader(reader::MeshAttributes::geometry_color_normal);
            save = reader::ReaderFactory::getWriter(reader::MeshAttributes::geometry_color_normal);
            break;
        default:
            fprintf(stderr, "[ERROR] Wrong mesh attributes type!\n");
            exit(EXIT_FAILURE);
    }

    garland::Mesh mesh;

    /**
     * Smooth mesh, save it, read to the mesh object, delete
     */
    if (vm["smooth"].as<bool>() && vm["quadric"].as<int>() == 3) {
        auto mesh_smoothed = mesh_in.substr(0, mesh_in.size() - 4) + "_smoothed.ply";
        smooth(mesh_in, mesh_smoothed, true, 7);
        read(mesh, mesh_smoothed);
        if (std::remove(mesh_smoothed.c_str()) != 0)
            fprintf(stderr, "[ERROR] File %s was not removed!\n", mesh_smoothed.c_str());
        else
            printf("[INFO] File %s was removed\n", mesh_smoothed.c_str());
    } else
        read(mesh, mesh_in);

    auto parallel = std::make_unique<parallel::ParallelSimplifier<partition::BasicPartitioner, int>>(mesh);

    INIT_TIMER(total)
    START_TIMER(total)

    switch (vm["quadric"].as<int>()) {
        case 3:
            parallel->simplify<garland::Quadric3>(vm);
            break;
        case 6:
            parallel->simplify<garland::Quadric6>(vm);
            break;
        case 9:
            parallel->simplify<garland::Quadric9>(vm);
            break;
        default:
            fprintf(stderr, "[ERROR] Wrong quadric type!\n");
            exit(EXIT_FAILURE);
    }

    save(mesh, mesh_out);

//    if (vm["smooth"].as<bool>() && vm["quadric"].as<int>() == 3)
//        smooth(mesh_out, mesh_out, false, 1);

    FINISH_AND_PRINT_TIMER(total, "Total processing time")
}