// Rotation field computation and optimization.
const std = @import("std");
const m    = @import("math.zig");
const sp   = @import("sparse.zig");
const lsqr = @import("lsmr.zig");
const mesh = @import("mesh.zig");

pub const Params = struct {
    neighbour_loss_weight: f64 = 20.0,
    max_overhang_deg: f64 = 30.0,
    rotation_multiplier: f64 = 2.0,
    initial_smoothing_iters: u32 = 30,
    set_initial_to_zero: bool = false,
    steep_overhang_compensation: bool = true,
    max_pos_rotation_deg: f64 = 3600.0,
    max_neg_rotation_deg: f64 = -3600.0,
    max_solver_iters: usize = 2000,
};

// Compute per-cell rotation matrices (axis = tangential to radial direction × z).
pub fn rotation_matrices(
    tet: *const mesh.TetMesh,
    rotation_field: []const f64,
    allocator: std.mem.Allocator,
) ![]m.Mat3x3 {
    const n = tet.cells.len;
    const mats = try allocator.alloc(m.Mat3x3, n);
    for (0..n) |i| {
        const cc = tet.cell_center[i];
        const xy = m.Vec3{ cc[0], cc[1], 0 };
        const up = m.Vec3{ 0, 0, 1 };
        var tangent = m.cross3(up, xy);
        tangent = if (m.norm3(tangent) < 1e-12) m.Vec3{ 1, 0, 0 } else m.normalize3(tangent);
        mats[i] = m.rotation_matrix_from_axis_angle(tangent, rotation_field[i]);
    }
    return mats;
}

// Compute initial rotation field: magnitude = |angle needed to fix overhang|,
// sign = gradient of path-length-to-base in the radial direction.
pub fn compute_initial_field(
    tet: *const mesh.TetMesh,
    params: Params,
    allocator: std.mem.Allocator,
) ![]f64 {
    const num_cells = tet.cells.len;
    const field = try allocator.alloc(f64, num_cells);
    const nan = std.math.nan(f64);
    const max_overhang_rad = std.math.degreesToRadians(90.0 + params.max_overhang_deg);

    // Magnitude: how much rotation is needed for each overhang cell
    for (0..num_cells) |i| {
        const oa = tet.overhang_angle[i];
        if (std.math.isNan(oa) or oa <= max_overhang_rad) {
            field[i] = nan;
        } else {
            field[i] = @abs(max_overhang_rad - oa);
        }
    }

    // Apply steep overhang compensation
    if (params.steep_overhang_compensation) {
        for (0..num_cells) |i| {
            if (tet.in_air[i] and !std.math.isNan(field[i])) {
                field[i] += 2.0 * (std.math.pi - tet.overhang_angle[i]);
            }
        }
    }

    // Compute path-length gradient in the radial direction
    const gradient = try path_length_gradient(tet, params, allocator);
    defer allocator.free(gradient);

    // Apply gradient (gives sign and smoothed magnitude)
    for (0..num_cells) |i| {
        if (!std.math.isNan(field[i]) and !std.math.isNan(gradient[i])) {
            field[i] *= gradient[i];
        } else if (!std.math.isNan(field[i])) {
            field[i] = nan; // no gradient → skip
        }
    }

    // Apply rotation multiplier and clamp
    const max_pos = std.math.degreesToRadians(params.max_pos_rotation_deg);
    const max_neg = std.math.degreesToRadians(params.max_neg_rotation_deg);
    for (0..num_cells) |i| {
        if (!std.math.isNan(field[i])) {
            field[i] = std.math.clamp(field[i] * params.rotation_multiplier, max_neg, max_pos);
        }
    }

    return field;
}

fn path_length_gradient(
    tet: *const mesh.TetMesh,
    params: Params,
    allocator: std.mem.Allocator,
) ![]f64 {
    const num_cells = tet.cells.len;
    const gradient = try allocator.alloc(f64, num_cells);
    @memset(gradient, 0);

    const max_overhang_rad = std.math.degreesToRadians(90.0 + params.max_overhang_deg);

    var tmp_points = std.array_list.Managed(m.Vec3).init(allocator);
    defer tmp_points.deinit();

    for (0..num_cells) |ci| {
        const dist = tet.cell_distance_to_bottom[ci];
        if (std.math.isNan(dist)) continue;
        const oa = tet.overhang_angle[ci];
        if (std.math.isNan(oa) or oa <= max_overhang_rad) continue;

        // Collect edge-neighbours with valid distances
        const nbrs = tet.point_neighbours[ci];
        tmp_points.clearRetainingCapacity();
        try tmp_points.append(.{
            tet.cell_center[ci][0], tet.cell_center[ci][1], dist,
        });
        for (nbrs) |ni| {
            const nd = tet.cell_distance_to_bottom[ni];
            if (!std.math.isNan(nd)) {
                try tmp_points.append(.{
                    tet.cell_center[ni][0], tet.cell_center[ni][1], nd,
                });
            }
        }

        const cc = tet.cell_center[ci];
        const r = @sqrt(cc[0] * cc[0] + cc[1] * cc[1]);

        if (tmp_points.items.len < 3 or r < 1e-10) {
            // Not enough neighbours → roll towards nearest bottom cell
            const path = tet.path_to_bottom[ci];
            if (path.len >= 2 and path[path.len - 1] >= 0) {
                const bottom_idx: usize = @intCast(path[path.len - 1]);
                const bcc = tet.cell_center[bottom_idx];
                const dx = bcc[0] - cc[0]; const dy = bcc[1] - cc[1];
                const dn = @sqrt(dx * dx + dy * dy);
                if (dn > 1e-10) {
                    const cx = cc[0] / r; const cy = cc[1] / r;
                    const dot = cx * dx / dn + cy * dy / dn;
                    gradient[ci] = if (dot >= 0) 1.0 else -1.0;
                }
            }
            continue;
        }

        // Plane fit to (cx, cy, dist) → gradient in radial direction
        const fit = try m.plane_fit(tmp_points.items, allocator);
        const normal = fit.normal;
        const cx = cc[0] / r; const cy = cc[1] / r;
        const dot = cx * normal[0] + cy * normal[1];
        gradient[ci] = dot;
    }

    // Smooth gradient
    if (params.initial_smoothing_iters > 0) {
        const smooth = try allocator.alloc(f64, num_cells);
        defer allocator.free(smooth);
        for (0..@intCast(params.initial_smoothing_iters)) |_| {
            @memcpy(smooth, gradient);
            for (0..num_cells) |ci| {
                if (smooth[ci] == 0) continue;
                const nbrs = tet.point_neighbours[ci];
                var sum: f64 = 0; var cnt: f64 = 0;
                for (nbrs) |ni| {
                    if (smooth[ni] != 0) { sum += smooth[ni]; cnt += 1; }
                }
                if (cnt > 0) gradient[ci] = sum / cnt;
            }
        }
    }

    // Zero → NaN if not set_initial_to_zero
    if (!params.set_initial_to_zero) {
        for (gradient) |*g| if (g.* == 0) { g.* = std.math.nan(f64); };
    }

    return gradient;
}

// Optimize the rotation field by solving a sparse linear LS problem.
// Minimizes: NEIGHBOUR_LOSS_WEIGHT * Σ(r[a]-r[b])^2 + Σ(r[i]-initial[i])^2
pub fn optimize(
    tet: *const mesh.TetMesh,
    initial_field: []const f64,
    params: Params,
    allocator: std.mem.Allocator,
) ![]f64 {
    const num_cells = tet.cells.len;
    const face_pairs = tet.face_neighbours;
    const num_pairs = face_pairs.len;

    // Count cells with valid initial rotation
    var valid_indices = std.array_list.Managed(u32).init(allocator);
    defer valid_indices.deinit();
    for (0..num_cells) |i| {
        if (!std.math.isNan(initial_field[i])) try valid_indices.append(@intCast(i));
    }
    const num_valid = valid_indices.items.len;

    // Build Jacobian: rows = num_pairs + num_valid, cols = num_cells
    const nrows = num_pairs + num_valid;
    var builder = sp.CsrBuilder.init(nrows, num_cells, allocator);
    defer builder.deinit();

    const wt = @sqrt(params.neighbour_loss_weight);
    for (face_pairs, 0..) |pair, i| {
        try builder.set(@intCast(i), pair[0], wt);
        try builder.set(@intCast(i), pair[1], -wt);
    }
    for (valid_indices.items, 0..) |vi, k| {
        try builder.set(@intCast(num_pairs + k), vi, 1.0);
    }

    var jac = try builder.build(allocator);
    defer jac.deinit();

    // Build rhs b: rows 0..num_pairs = 0 (neighbour constraint rhs), rows num_pairs.. = initial[i]
    const b = try allocator.alloc(f64, nrows);
    defer allocator.free(b);
    @memset(b[0..num_pairs], 0);
    for (valid_indices.items, 0..) |vi, k| {
        b[num_pairs + k] = initial_field[vi];
    }

    // Solve
    const rotation_field = try allocator.alloc(f64, num_cells);
    @memset(rotation_field, 0);
    _ = try lsqr.solve_normal_equations(&jac, b, rotation_field, .{
        .atol = 1e-6,
        .max_iter = params.max_solver_iters,
    }, allocator);

    std.debug.print("  rotation field optimized over {} cells, {} pairs, {} valid\n", .{ num_cells, num_pairs, num_valid });
    return rotation_field;
}
