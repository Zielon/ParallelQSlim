#pragma once

#include <cstdint>
#include <cstring>

struct __attribute__((packed)) Vertex9x9Reconstruction {
    const float x;
    const float y;
    const float z;

    const float nx;
    const float ny;
    const float nz;

    const unsigned char red;
    const unsigned char green;
    const unsigned char blue;
    const unsigned char alpha;

    bool operator==(const Vertex9x9Reconstruction &rhs) const {
        return
                x == rhs.x && y == rhs.y && z == rhs.z &&
                nx == rhs.nx && ny == rhs.ny && nz == rhs.nz &&
                red == rhs.red && blue == rhs.blue && green == rhs.green && alpha == rhs.alpha;
    }
};

struct __attribute__((packed)) VertexUntexturedReconstruction {
    const float x;
    const float y;
    const float z;
    // const float quality;

    bool operator==(const VertexUntexturedReconstruction &rhs) const {
        return x == rhs.x && y == rhs.y && z == rhs.z;  // && quality == rhs.quality;
    }
};

struct __attribute__((packed)) FaceUntexturedReconstruction {
    const unsigned char n;
    const int v1;
    const int v2;
    const int v3;
    // const float quality;

    bool operator==(const FaceUntexturedReconstruction &rhs) const {
        return v1 == rhs.v1 && v2 == rhs.v2 && v3 == rhs.v3;  // && quality == rhs.quality;
    }
};