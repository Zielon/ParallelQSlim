#ifndef MESH_RECONSTRUCTION_TAUBIN_SMOOTH_H
#define MESH_RECONSTRUCTION_TAUBIN_SMOOTH_H

// https://github.com/benwu95/mesh-smoothing

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

typedef OpenMesh::PolyMesh_ArrayKernelT<>  MyMesh;
OpenMesh::EPropHandleT<MyMesh::Scalar> eWeights, atheta, btheta;
OpenMesh::VPropHandleT<MyMesh::Normal> vNormals;
OpenMesh::VPropHandleT<MyMesh::Point> cogs;
OpenMesh::FPropHandleT<MyMesh::Scalar> fRank;

void Scale(MyMesh& mesh, MyMesh::Scalar s) {
    MyMesh::VertexIter v_it, v_end(mesh.vertices_end());
    for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it)
        mesh.set_point(*v_it, mesh.point(*v_it)*s);
}

void VertexNormal(MyMesh& mesh) {
    MyMesh::VertexIter v_it, v_end(mesh.vertices_end());
    MyMesh::VertexFaceIter vf_it;
    MyMesh::Normal tmp;
    MyMesh::Scalar count;
    for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it) {
        tmp[0] = tmp[1] = tmp[2] = 0.0;
        count = 0.0;
        for (vf_it = mesh.vf_iter(*v_it); vf_it.is_valid(); ++vf_it) {
            tmp += mesh.calc_face_normal(*vf_it);
            ++count;
        }
        mesh.property(vNormals, *v_it) = tmp / count;
    }
}

void OriginalRank(MyMesh& mesh) {
    MyMesh::FaceIter f_it, f_end(mesh.faces_end());
    MyMesh::FaceFaceIter ff_it;
    MyMesh::Scalar original, lb(-1.0), ub(1.0), count;
    MyMesh::Normal aon, d0, d1;
    for (f_it = mesh.faces_begin(); f_it != f_end; ++f_it) {
        aon[0] = aon[1] = aon[2] = count = 0.0;
        for (ff_it = mesh.ff_iter(*f_it); ff_it.is_valid(); ++ff_it) {
            aon += mesh.calc_face_normal(*ff_it).normalize();
            ++count;
        }
        d0 = mesh.calc_face_normal(*f_it);
        d0.normalize();
        d1 = aon / count;
        d1.normalize();
        original = std::abs(std::max(lb, std::min(ub, dot(d0, d1))));
        mesh.property(fRank, *f_it) = original;
    }
}

void SimpleRank(MyMesh& mesh, int mode) {
    MyMesh::FaceIter f_it, f_end(mesh.faces_end());
    MyMesh::FaceFaceIter ff_it;
    MyMesh::Scalar lb(-1.0), ub(1.0);
    MyMesh::Normal aon, d0, d1;
    MyMesh::Scalar weight, weight0;
    MyMesh::Scalar valence;
    MyMesh::Normal tmp;
    int r = 0, o = 0, y = 0, g = 0;
    int r0 = 0, o0 = 0, y0 = 0, g0 = 0;

    for (f_it = mesh.faces_begin(); f_it != f_end; ++f_it) {
        aon[0] = aon[1] = aon[2] = valence = 0.0;
        for (ff_it = mesh.ff_iter(*f_it); ff_it.is_valid(); ++ff_it) {
            tmp = mesh.calc_face_normal(*ff_it);
            tmp.normalize();
            aon += tmp;
            ++valence;
        }
        d0 = mesh.calc_face_normal(*f_it);
        d0.normalize();
        d1 = aon / valence;
        d1.normalize();
        weight = std::abs(std::max(lb, std::min(ub, dot(d0, d1))));
        weight0 = mesh.property(fRank, *f_it);

        // 1          0.92387953251 0.70710678118 0.38268343236 0
        // cos(0)     cos(22.5)     cos(45)       cos(67.5)     cos(90)
        if (weight < 0.4)
            r++;
        if ((weight >= 0.4) && (weight < 0.7))
            o++;
        if ((weight >= 0.7) && (weight < 0.9))
            y++;
        if ((weight >= 0.9) && (weight <= 1))
            g++;

        if (weight0 < 0.4)
            r0++;
        if ((weight0 >= 0.4) && (weight0 < 0.7))
            o0++;
        if ((weight0 >= 0.7) && (weight0 < 0.9))
            y0++;
        if ((weight0 >= 0.9) && (weight0 <= 1))
            g0++;
    }

    // print result
    std::cout << "Before smoothing:\n";
    std::cout << "  -*Rank*-    " << "-*number of faces*-\n";
    std::cout << "    Bad:        " << r0 << "\n";
    std::cout << "    Good:       " << o0 << "\n";
    std::cout << "    Fine:       " << y0 << "\n";
    std::cout << "    Perfect:    " << g0 << "\n";
    std::cout << "\n";
    if (mode == 1)
        std::cout << "After Laplacian smoothing:\n";
    if (mode == 2)
        std::cout << "After Taubin smoothing:\n";
    if (mode == 3)
        std::cout << "After Laplacian smoothing with custom weight:\n";
    std::cout << "  -*Rank*-    " << "-*number of faces*-\n";
    std::cout << "    Bad:        " << r << "\n";
    std::cout << "    Good:       " << o << "\n";
    std::cout << "    Fine:       " << y << "\n";
    std::cout << "    Perfect:    " << g << "\n";
}

void ObjOutput(MyMesh& mesh, char* file) {
    // add vertex normal to .obj file
    std::string line, name, sub;
    name.assign(file);
    sub.assign(name.end()-4, name.end());
    name.erase(name.end()-4, name.end());
    name.append("_n");
    name.append(sub);
    std::fstream input(file, std::ios::in);
    std::fstream output(name, std::ios::out);

    while (getline(input, line)) {
        if (line.at(0) != 'f')
            output << line << std::endl;
        else
            break;
    }

    MyMesh::VertexIter v_it, v_end(mesh.vertices_end());
    MyMesh::Normal tmp;
    for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it) {
        tmp = mesh.property(vNormals, *v_it);
        output << "vn " << tmp[0] << " " << tmp[1] << " " << tmp[2] << std::endl;
    }

    size_t begin, end;
    while (getline(input, line)) {
        begin = 0;
        end = line.find(" ");
        if (line.at(0) == 'f') {
            output << line.substr(begin, end - begin);
            while (end != std::string::npos) {
                begin = end + 1;
                end = line.find(" ", begin);
                output <<  " " << line.substr(begin, end - begin) << "//" << line.substr(begin, end - begin);
            }
            output << "\n";
        }
    }
    input.close();
    output.close();
}

void LenWeight(MyMesh& mesh) {
    MyMesh::EdgeIter e_it, e_end(mesh.edges_end());
    for (e_it = mesh.edges_begin(); e_it != e_end; ++e_it)
        mesh.property(eWeights, *e_it) = 1.0 / mesh.calc_edge_length(*e_it);
}

void CotanWeight(MyMesh& mesh) {
    MyMesh::EdgeIter e_it, e_end(mesh.edges_end());
    MyMesh::Scalar weight;
    MyMesh::Scalar a, b;
    MyMesh::HalfedgeHandle h0, h1, h2;
    MyMesh::Point p0, p1, p2;
    MyMesh::Normal d0, d1;
    const float pi = 3.14159265359;

    for (e_it = mesh.edges_begin(); e_it != e_end; ++e_it) {
        weight = 0.0;

        h0 = mesh.halfedge_handle(*e_it, 0);
        p0 = mesh.point(mesh.to_vertex_handle(h0));
        h1 = mesh.halfedge_handle(*e_it, 1);
        p1 = mesh.point(mesh.to_vertex_handle(h1));

        h2 = mesh.next_halfedge_handle(h0);
        p2 = mesh.point(mesh.to_vertex_handle(h2));
        d0 = (p0 - p2); d0.normalize();
        d1 = (p1 - p2); d1.normalize();

        a = acos(dot(d0, d1));
        weight += MyMesh::Scalar(1.0) / tan(a);

        h2 = mesh.next_halfedge_handle(h1);
        p2 = mesh.point(mesh.to_vertex_handle(h2));
        d0 = (p0 - p2); d0.normalize();
        d1 = (p1 - p2); d1.normalize();

        b = acos(dot(d0, d1));
        weight += MyMesh::Scalar(1.0) / tan(b);

        mesh.property(eWeights, *e_it) = weight;
        mesh.property(atheta, *e_it) = a * 180.0 / pi;
        mesh.property(btheta, *e_it) = b * 180.0 / pi;
    }
}

void VertexSmooth(MyMesh& mesh, unsigned int mode, unsigned int N) {
    /////////////////////////////////////////////////////////
    // mode 1 = Laplacian smoothing                        //
    // mode 2 = Taubin smoothing                           //
    // mode 3 = Laplacian smoothing with custom weight     //
    // 0 < lamda < 1, -1 < mu < 0                          //
    /////////////////////////////////////////////////////////

    // smoothing mesh N times
    MyMesh::VertexIter v_it, v_end(mesh.vertices_end());
    MyMesh::VertexEdgeIter ve_it;
    MyMesh::VertexVertexIter vv_it;
    MyMesh::EdgeIter e_it, e_end(mesh.edges_end());
    MyMesh::Point cog, tmp, p0, p1;
    MyMesh::HalfedgeHandle h0, h1;
    MyMesh::Scalar valence, weight;
    unsigned int i;
    float lamda, mu;

    if (mode == 1 || mode == 3)
        lamda = 0.25;
    if (mode == 2) {
        lamda = 0.5;
        mu = -0.67;
    }
    for (i = 0; i < N; ++i) {
        if (mode == 3)
            CotanWeight(mesh);

        for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it) {
            cog[0] = cog[1] = cog[2] = valence = 0.0;

            // iterate over all neighboring vertices
            if (mode == 1 || mode == 2) {
                for (vv_it = mesh.vv_iter(*v_it); vv_it.is_valid(); ++vv_it) {
                    cog += mesh.point(*vv_it);
                    ++valence;
                }
                tmp = (cog / valence - mesh.point(*v_it)) * lamda + mesh.point(*v_it);
                if (mode == 2)
                    tmp = (cog / valence - tmp) * mu + tmp;
            }
            else if (mode == 3) {
                for (ve_it = mesh.ve_iter(*v_it); ve_it.is_valid(); ++ve_it) {
                    h0 = mesh.halfedge_handle(*ve_it, 0);
                    p0 = mesh.point(mesh.to_vertex_handle(h0));
                    h1 = mesh.halfedge_handle(*ve_it, 1);
                    p1 = mesh.point(mesh.to_vertex_handle(h1));
                    weight = mesh.property(eWeights, *ve_it);
                    if (p0 == mesh.point(*v_it))
                        cog += weight * (p1 - p0);
                    else
                        cog += weight * (p0 - p1);
                    valence += weight;
                }
                tmp = (cog / valence) * lamda + mesh.point(*v_it);
            }
            mesh.property(cogs, *v_it) = tmp;
        }
        for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it)
            if (!mesh.is_boundary(*v_it))
                mesh.set_point(*v_it, mesh.property(cogs, *v_it));
    }
}

void smooth(std::string input, std::string output, bool binaryInput, int iterations){

    MyMesh mesh;
    unsigned mode(atoi("2")), N(iterations);
    // read mesh from stdin

    OpenMesh::IO::Options ropt;

    if (binaryInput){
        ropt += OpenMesh::IO::Options::Binary;
        ropt += OpenMesh::IO::Options::LSB;
    }

//    ropt += OpenMesh::IO::Options::VertexNormal;
//    ropt += OpenMesh::IO::Options::VertexColor;
//    ropt += OpenMesh::IO::Options::ColorAlpha;

    if (!OpenMesh::IO::read_mesh(mesh, input, ropt)) {
        std::cerr << "Error: Cannot read mesh from " << input << std::endl;
        return;
    }
    mesh.request_vertex_normals();
    mesh.request_face_normals();
    mesh.request_vertex_colors();

    mesh.update_vertex_normals();
    mesh.update_face_normals();

    // add custom properties
    mesh.add_property(vNormals);
    mesh.add_property(cogs);
    mesh.add_property(fRank);
    if (mode == 3) {
        mesh.add_property(atheta);
        mesh.add_property(btheta);
        mesh.add_property(eWeights);
    }

    // main function
    OriginalRank(mesh);
    VertexSmooth(mesh, mode, N);

    // add vertex normals
    mesh.update_vertex_normals();
    mesh.update_face_normals();
    VertexNormal(mesh);

    OpenMesh::IO::Options wopt;

    wopt += OpenMesh::IO::Options::Binary;
    wopt += OpenMesh::IO::Options::LSB;

    //auto file = output.substr(0, output.size() - 4) + "_smoothed.ply";
    const auto& file = output;

    // write mesh to stdout
    if (!OpenMesh::IO::write_mesh(mesh, file, wopt)) {
        std::cerr << "[ERROR] Cannot write mesh to " << file << std::endl;
        return;
    } else
        std::cout << "[INFO] Smoothed mesh has been saved in [" << file << "]" << std::endl;

//    std::string str = "string";
//    char *cstr = new char[str.length() + 1];
//    strcpy(cstr, str.c_str());

    // ObjOutput(mesh, cstr);

    // print simple rank
    // SimpleRank(mesh, mode);
}

#endif