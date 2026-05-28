// Mesh deformation: find new vertex positions that make each tet match
// the rotation prescribed by the rotation field.
// The problem is linear LS: minimize Σ_cells || N @ new_v[cell] - R[cell] @ N @ old_v[cell] ||^2
const std = @import("std");
const m    = @import("math.zig");
const sp   = @import("sparse.zig");
const lsqr = @import("lsmr.zig");
const rot  = @import("rotation.zig");
const mesh = @import("mesh.zig");

// N matrix (4×4, row-major): centers tet vertices around origin
// N = I - (1/4) * ones(4,4)
const N: [16]f64 = blk: {
    const v = 3.0 / 4.0; const w = -1.0 / 4.0;
    break :blk .{
        v, w, w, w,
        w, v, w, w,
        w, w, v, w,
        w, w, w, v,
    };
};

// Multiply 4×4 N matrix by a 4×3 matrix of tet vertices.
// result[i][j] = Σ_k N[i][k] * verts[k][j]
fn n_mul_verts(verts: [4]m.Vec3) [4]m.Vec3 {
    var out: [4]m.Vec3 = undefined;
    for (0..4) |i| {
        out[i] = .{ 0, 0, 0 };
        for (0..4) |k| {
            out[i][0] += N[i * 4 + k] * verts[k][0];
            out[i][1] += N[i * 4 + k] * verts[k][1];
            out[i][2] += N[i * 4 + k] * verts[k][2];
        }
    }
    return out;
}

pub const Params = struct {
    max_iters: usize = 2000,
    atol: f64 = 1e-8,
};

// Returns new vertex positions (same count as tet.points).
pub fn deform(
    tet: *const mesh.TetMesh,
    rot_mats: []const m.Mat3x3,
    params: Params,
    allocator: std.mem.Allocator,
) ![]m.Vec3 {
    const num_cells  = tet.cells.len;
    const num_points = tet.points.len;

    // For each cell, compute target: R[cell] @ (N @ old_verts[cell])
    // This is a 4×3 matrix (4 rows = 4 centered vertices, 3 cols = xyz)
    // We'll flatten the residual per cell into 3 scalar equations (one per row of ||.||^2, summed over 4 rows)
    // Actually: the residual is ||new_n_verts - target||_F^2 which expands to
    // Σ_{i=0..3} ||new_n_verts[i] - target[i]||^2
    // = Σ_{i=0..3} Σ_{j=0..2} (new_n_verts[i][j] - target[i][j])^2
    //
    // This is 12 scalar residuals per cell (4 verts × 3 coords) but they're coupled
    // through the N matrix (each new_n_vert[i][j] = Σ_k N[i][k] * new_pts[cell[k]][j]).
    //
    // We separate the 3 coordinate axes and solve 3 independent systems (x, y, z).

    const new_pts = try allocator.alloc(m.Vec3, num_points);

    // Build one shared sparsity structure; J is (num_cells*4) × num_points
    // for each axis independently (J is the same for all 3 axes).
    //
    // Actually for efficiency we build J once and solve 3 RHS.

    // Rows: num_cells * 4 (4 N-transformed rows per cell)
    const nrows = num_cells * 4;
    const ncols = num_points;

    var builder = sp.CsrBuilder.init(nrows, ncols, allocator);
    defer builder.deinit();

    // Fill Jacobian: row = cell*4+i, for each k in 0..4: col = cell[k], val = N[i][k]
    for (tet.cells, 0..) |cell, ci| {
        for (0..4) |i| {
            for (0..4) |k| {
                const val = N[i * 4 + k];
                if (@abs(val) < 1e-15) continue;
                try builder.set(@intCast(ci * 4 + i), cell[k], @floatCast(val));
            }
        }
    }

    var jac = try builder.build(allocator);
    defer jac.deinit();

    // Solve for x, y, z independently
    const rhs = try allocator.alloc(f64, nrows);
    defer allocator.free(rhs);
    const sol = try allocator.alloc(f64, ncols);
    defer allocator.free(sol);

    inline for (0..3) |axis| {
        // Build rhs: for each cell, target = R @ (N @ old_verts)
        for (tet.cells, 0..) |cell, ci| {
            var old_verts: [4]m.Vec3 = undefined;
            for (0..4) |k| old_verts[k] = tet.points[cell[k]];
            const n_verts = n_mul_verts(old_verts);
            // target[i] = R[ci] @ n_verts[i]  →  target[i][axis]
            for (0..4) |i| {
                const rotated = m.mat3_mul_vec3(rot_mats[ci], n_verts[i]);
                rhs[ci * 4 + i] = rotated[axis];
            }
        }

        // Initial guess: old positions
        for (0..num_points) |j| sol[j] = tet.points[j][axis];

        _ = try lsqr.solve_normal_equations(&jac, rhs, sol, .{
            .atol = params.atol,
            .max_iter = params.max_iters,
        }, allocator);

        for (0..num_points) |j| new_pts[j][axis] = sol[j];
    }

    std.debug.print("  deformation: {} cells, {} points\n", .{ num_cells, num_points });
    return new_pts;
}
