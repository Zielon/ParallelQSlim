cmake_minimum_required(VERSION 3.14 FATAL_ERROR)
project(nv_simplify_mesh)

set(CMAKE_CXX_STANDARD 17)

find_package (Eigen3)

include_directories(${OPENMESH_INCLUDE_DIRS})
link_directories(${OPENMESH_LIBRARY_DIR})

add_executable (nv_garland_decimate
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

        src/partition/aabb.h
        src/partition/basic/basic_partitioner.h

        src/reader/mesh_reader.h
        src/reader/open_mesh_reader.h
        src/reader/fast_ply_reader.h

        src/garland/types.h
        src/nv_garland_decimate.cpp
)

target_link_libraries(nv_garland_decimate ${LIBRARIES} ${OPENMESH_LIBRARIES})
install(TARGETS nv_garland_decimate DESTINATION bin)