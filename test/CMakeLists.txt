#=============================================================================
# Unit Tests
#=============================================================================

# -----------------------------------------------------------------------------
# Set up GTest - only include gtest once in your project
# -----------------------------------------------------------------------------


##=============================================================================

# -----------------------------------------------------------------------------
# Utility library for the tests
# -----------------------------------------------------------------------------
add_library(meshutils STATIC mesh_utils.h)
target_link_libraries(meshutils tests) # make tests link against gtest

##=============================================================================
## Add the tests themselves
##=============================================================================

## Discover the source files for all test executable starting with test_*
FILE(GLOB_RECURSE TESTCASES test_*.cpp)

foreach (testsrcfile ${TESTCASES})
    get_filename_component(testcase ${testsrcfile} NAME_WE)

    add_executable(${testcase} ${testsrcfile}
            mesh_utils.h
            test_mesh_general.cpp
            test_quadric.cpp
            ../src/garland/models/mesh.cpp
            ../src/garland/models/edge.cpp
            ../src/garland/models/face.cpp
            ../src/garland/models/vertex.cpp
            ../src/garland/models/quadric/3x3/quadric3.cpp
            ../src/garland/models/quadric/6x6/quadric6.cpp
            ../src/garland/models/quadric/9x9/quadric9.cpp
            ../src/garland/q_slim.cpp
            ../src/garland/geometry.cpp
            )

    target_link_libraries(${testcase} testutils_recon ${LIBRARIES})
    add_test(NAME ${testcase} COMMAND ${testcase})
endforeach ()
