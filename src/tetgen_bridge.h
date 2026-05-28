#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle to tetgen output
typedef struct TetResult {
    double *points;       // [num_points * 3]
    int    *tetrahedra;   // [num_tets * 4], 0-indexed vertex indices
    int     num_points;
    int     num_tets;
} TetResult;

// Tetrahedralize a surface triangle mesh.
// vertices: [num_verts * 3] (x,y,z) coordinates
// triangles: [num_triangles * 3] 0-indexed vertex indices
// Returns heap-allocated TetResult; caller must call tet_result_free().
// Returns NULL on error.
TetResult *tet_mesh(const double *vertices, int num_verts,
                    const int *triangles, int num_triangles);

void tet_result_free(TetResult *r);

#ifdef __cplusplus
}
#endif
