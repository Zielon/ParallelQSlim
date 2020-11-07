cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(simplify_mesh)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE -O3)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS filesystem program_options system thread REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_executable(
        main

        # GARLAND FOLDER
        src/garland/models/quadric/3x3/quadric3.cpp
        src/garland/models/quadric/6x6/quadric6.cpp
        src/garland/models/quadric/9x9/quadric9.cpp
        src/garland/models/face.cpp
        src/garland/models/vertex.cpp
        src/garland/models/mesh.cpp
        src/garland/models/edge.cpp
        src/garland/geometry.cpp
        src/garland/q_slim.cpp
        src/garland/enums.h
        src/garland/types.h

        # PARTITIONER FOLDER
        src/partition/aabb.h
        src/partition/basic/basic_partitioner.h

        # READER FOLDER
        src/reader/fastply_datalayouts.h
        src/reader/mesh_reader.h
        src/reader/open_mesh_reader.h
        src/reader/fast_ply_reader.h

        # EXTERNAL DEPENDENCIES FOLDER
        src/external/fastply.h
        src/external/fastply_macros.h

        src/main.cpp
)

target_link_libraries(main Eigen3::Eigen ${Boost_LIBRARIES})