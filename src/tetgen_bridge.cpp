#include "tetgen_bridge.h"
#include "../third_party/tetgen.h"
#include <cstdlib>
#include <cstring>

// Suppress tetgen's use of exit() on error by catching it via setjmp/longjmp
// would require patching tetgen; instead we just let it run and trust the switches.

extern "C" {

TetResult *tet_mesh(const double *vertices, int num_verts,
                    const int *triangles, int num_triangles) {
    tetgenio in, out;

    in.numberofpoints = num_verts;
    in.pointlist = new REAL[num_verts * 3];
    for (int i = 0; i < num_verts * 3; i++)
        in.pointlist[i] = vertices[i];

    in.numberoffacets = num_triangles;
    in.facetlist = new tetgenio::facet[num_triangles];
    in.facetmarkerlist = new int[num_triangles];

    for (int i = 0; i < num_triangles; i++) {
        tetgenio::facet &f = in.facetlist[i];
        f.numberofpolygons = 1;
        f.polygonlist = new tetgenio::polygon[1];
        f.numberofholes = 0;
        f.holelist = nullptr;
        tetgenio::polygon &p = f.polygonlist[0];
        p.numberofvertices = 3;
        p.vertexlist = new int[3];
        p.vertexlist[0] = triangles[i * 3 + 0];
        p.vertexlist[1] = triangles[i * 3 + 1];
        p.vertexlist[2] = triangles[i * 3 + 2];
        in.facetmarkerlist[i] = 0;
    }

    // "p" = PLC mode (surface mesh input)
    // "q" = quality mesh generation
    // "Y" = don't insert Steiner points on surfaces (preserve surface)
    // "Q" = quiet
    try {
        tetrahedralize((char*)"pYQq", &in, &out);
    } catch (...) {
        return nullptr;
    }

    if (out.numberofpoints == 0 || out.numberoftetrahedra == 0)
        return nullptr;

    TetResult *r = new TetResult;
    r->num_points = out.numberofpoints;
    r->num_tets   = out.numberoftetrahedra;

    r->points = new double[r->num_points * 3];
    for (int i = 0; i < r->num_points * 3; i++)
        r->points[i] = out.pointlist[i];

    r->tetrahedra = new int[r->num_tets * 4];
    for (int i = 0; i < r->num_tets * 4; i++)
        r->tetrahedra[i] = out.tetrahedronlist[i];

    return r;
}

void tet_result_free(TetResult *r) {
    if (!r) return;
    delete[] r->points;
    delete[] r->tetrahedra;
    delete r;
}

} // extern "C"
