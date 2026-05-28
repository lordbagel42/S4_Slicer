const std = @import("std");
const stl       = @import("stl.zig");
const mesh_mod  = @import("mesh.zig");
const rot_mod   = @import("rotation.zig");
const deform    = @import("deformation.zig");
const gcode     = @import("gcode.zig");
const spatial   = @import("spatial.zig");
const m         = @import("math.zig");

const version = "0.1.0-m2";

// --------------------------------------------------------------------------
// CLI parameters — all tunable via flags, defaulting to notebook values.
// --------------------------------------------------------------------------
const Params = struct {
    input_stl:   []const u8 = "",
    input_gcode: []const u8 = "",
    output_gcode: []const u8 = "",
    // Rotation
    neighbour_loss_weight: f64 = 20.0,
    max_overhang_deg: f64 = 30.0,
    rotation_multiplier: f64 = 2.0,
    initial_smoothing: u32 = 30,
    set_initial_to_zero: bool = false,
    steep_overhang: bool = true,
    max_pos_rotation_deg: f64 = 3600.0,
    max_neg_rotation_deg: f64 = -3600.0,
    rotation_iters: usize = 2000,
    // Deformation
    deform_iters: usize = 2000,
    // G-code
    nozzle_offset: f64 = 42.0,
    retraction_length: f64 = 1.0,
    rotation_avg_alpha: f64 = 0.2,
    rotation_max_delta_deg: f64 = 1.0,
    max_extrusion_multiplier: f64 = 10.0,
    min_rotation_deg: f64 = -130.0,
    max_rotation_deg: f64 = 30.0,
    // Misc
    show_help: bool = false,
    show_version: bool = false,
};

const help_text =
    \\s4slicer — non-planar 4-axis slicer  (https://github.com/jyjblrd/S4_Slicer)
    \\
    \\Usage:
    \\  s4slicer <input.stl> <input.gcode> -o <output.gcode> [options]
    \\
    \\Required:
    \\  <input.stl>              Surface mesh (binary or ASCII STL)
    \\  <input.gcode>            Pre-sliced G-code of the deformed mesh (from Cura)
    \\  -o, --output <path>      Output polar 4-axis G-code path
    \\
    \\Tuning (defaults match the reference notebook):
    \\  --max-overhang <deg>     Max printable overhang angle [30]
    \\  --rot-multiplier <f>     Rotation multiplier [2.0]
    \\  --neighbour-weight <f>   Neighbour smoothing weight [20.0]
    \\  --smoothing-iters <n>    Initial rotation field smoothing passes [30]
    \\  --rotation-iters <n>     Rotation optimizer iterations [2000]
    \\  --deform-iters <n>       Deformation optimizer iterations [2000]
    \\  --nozzle-offset <mm>     Nozzle offset from B-axis [42.0]
    \\  --steep-overhang         Enable steep overhang compensation [on]
    \\  --no-steep-overhang      Disable steep overhang compensation
    \\
    \\  -h, --help               Show this help
    \\  -V, --version            Show version
    \\
;

fn parse_cli(init: std.process.Init, allocator: std.mem.Allocator) !Params {
    var p: Params = .{};
    var it = try std.process.Args.Iterator.initAllocator(init.minimal.args, allocator);
    defer it.deinit();
    _ = it.skip(); // program name

    var positional: usize = 0;
    while (it.next()) |arg| {
        if (std.mem.eql(u8, arg, "-h") or std.mem.eql(u8, arg, "--help")) {
            p.show_help = true;
        } else if (std.mem.eql(u8, arg, "-V") or std.mem.eql(u8, arg, "--version")) {
            p.show_version = true;
        } else if (std.mem.eql(u8, arg, "-o") or std.mem.eql(u8, arg, "--output")) {
            p.output_gcode = try allocator.dupe(u8, it.next() orelse return error.MissingArg);
        } else if (std.mem.eql(u8, arg, "--max-overhang")) {
            p.max_overhang_deg = try std.fmt.parseFloat(f64, it.next() orelse return error.MissingArg);
        } else if (std.mem.eql(u8, arg, "--rot-multiplier")) {
            p.rotation_multiplier = try std.fmt.parseFloat(f64, it.next() orelse return error.MissingArg);
        } else if (std.mem.eql(u8, arg, "--neighbour-weight")) {
            p.neighbour_loss_weight = try std.fmt.parseFloat(f64, it.next() orelse return error.MissingArg);
        } else if (std.mem.eql(u8, arg, "--smoothing-iters")) {
            p.initial_smoothing = try std.fmt.parseInt(u32, it.next() orelse return error.MissingArg, 10);
        } else if (std.mem.eql(u8, arg, "--rotation-iters")) {
            p.rotation_iters = try std.fmt.parseInt(usize, it.next() orelse return error.MissingArg, 10);
        } else if (std.mem.eql(u8, arg, "--deform-iters")) {
            p.deform_iters = try std.fmt.parseInt(usize, it.next() orelse return error.MissingArg, 10);
        } else if (std.mem.eql(u8, arg, "--nozzle-offset")) {
            p.nozzle_offset = try std.fmt.parseFloat(f64, it.next() orelse return error.MissingArg);
        } else if (std.mem.eql(u8, arg, "--steep-overhang")) {
            p.steep_overhang = true;
        } else if (std.mem.eql(u8, arg, "--no-steep-overhang")) {
            p.steep_overhang = false;
        } else if (!std.mem.startsWith(u8, arg, "-")) {
            switch (positional) {
                0 => p.input_stl   = try allocator.dupe(u8, arg),
                1 => p.input_gcode = try allocator.dupe(u8, arg),
                else => return error.TooManyArgs,
            }
            positional += 1;
        } else {
            std.debug.print("unknown flag: {s}\n", .{arg});
            return error.UnknownFlag;
        }
    }
    return p;
}

fn now_ns() i64 {
    var ts: std.os.linux.timespec = undefined;
    _ = std.os.linux.clock_gettime(.MONOTONIC, &ts);
    return ts.sec * 1_000_000_000 + ts.nsec;
}

fn timer_ms(t0: i64) i64 {
    return @divTrunc(now_ns() - t0, 1_000_000);
}

pub fn main(init: std.process.Init) !void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    const params = parse_cli(init, allocator) catch |err| {
        std.debug.print("error: {s}\n\n{s}", .{ @errorName(err), help_text });
        std.process.exit(2);
    };

    if (params.show_version) {
        try std.Io.File.stdout().writeStreamingAll(init.io, "s4slicer " ++ version ++ "\n");
        return;
    }
    if (params.show_help) {
        try std.Io.File.stdout().writeStreamingAll(init.io, help_text);
        return;
    }
    if (params.input_stl.len == 0 or params.input_gcode.len == 0 or params.output_gcode.len == 0) {
        try std.Io.File.stdout().writeStreamingAll(init.io, help_text);
        std.process.exit(2);
    }

    var t0 = now_ns();

    // ── Stage 1: Load STL ──────────────────────────────────────────────────
    std.debug.print("[1/7] Loading STL: {s}\n", .{params.input_stl});
    var surface = try stl.load(params.input_stl, init.io, allocator);
    defer surface.deinit();
    std.debug.print("      {} verts, {} triangles  ({} ms)\n", .{
        surface.vertices.len, surface.triangles.len, timer_ms(t0),
    });
    t0 = now_ns();

    // ── Stage 2: Tetrahedralize ────────────────────────────────────────────
    std.debug.print("[2/7] Tetrahedralizing...\n", .{});
    const tet_result = try tetrahedralize(&surface, allocator);
    defer allocator.free(tet_result.points);
    defer allocator.free(tet_result.tets);
    std.debug.print("      {} tet verts, {} tets  ({} ms)\n", .{
        tet_result.num_verts, tet_result.num_tets, timer_ms(t0),
    });
    t0 = now_ns();

    // ── Stage 3: Build tet mesh + attributes ──────────────────────────────
    std.debug.print("[3/7] Building mesh attributes (Dijkstra, normals, bottom detection)...\n", .{});
    var tet = try mesh_mod.build(
        tet_result.points, tet_result.num_verts,
        tet_result.tets,   tet_result.num_tets,
        allocator,
    );
    defer tet.deinit();
    const num_bottom = blk: {
        var c: usize = 0;
        for (tet.is_bottom) |b| if (b) { c += 1; };
        break :blk c;
    };
    std.debug.print("      {} cells, {} bottom cells  ({} ms)\n", .{
        tet.cells.len, num_bottom, timer_ms(t0),
    });
    t0 = now_ns();

    // Keep a copy of original mesh for barycentric lookup later
    const original_points = try allocator.dupe(m.Vec3, tet.points);

    // ── Stage 4: Rotation field optimization ──────────────────────────────
    std.debug.print("[4/7] Optimizing rotation field...\n", .{});
    const rot_params = rot_mod.Params{
        .neighbour_loss_weight = params.neighbour_loss_weight,
        .max_overhang_deg = params.max_overhang_deg,
        .rotation_multiplier = params.rotation_multiplier,
        .initial_smoothing_iters = params.initial_smoothing,
        .set_initial_to_zero = params.set_initial_to_zero,
        .steep_overhang_compensation = params.steep_overhang,
        .max_pos_rotation_deg = params.max_pos_rotation_deg,
        .max_neg_rotation_deg = params.max_neg_rotation_deg,
        .max_solver_iters = params.rotation_iters,
    };
    const initial_field = try rot_mod.compute_initial_field(&tet, rot_params, allocator);
    const rotation_field = try rot_mod.optimize(&tet, initial_field, rot_params, allocator);
    std.debug.print("      ({} ms)\n", .{timer_ms(t0)});
    t0 = now_ns();

    // ── Stage 5: Mesh deformation ─────────────────────────────────────────
    std.debug.print("[5/7] Deforming mesh...\n", .{});
    const rot_mats = try rot_mod.rotation_matrices(&tet, rotation_field, allocator);
    const new_pts = try deform.deform(&tet, rot_mats, .{
        .max_iters = params.deform_iters,
        .atol = 1e-8,
    }, allocator);

    // Update tet with new points
    @memcpy(tet.points, new_pts);
    mesh_mod.update_attributes(&tet);

    // Vertex transformations: deformed - original
    const vertex_transformations = try allocator.alloc(m.Vec3, tet.points.len);
    for (0..tet.points.len) |i| {
        vertex_transformations[i] = m.sub3(tet.points[i], original_points[i]);
    }
    std.debug.print("      ({} ms)\n", .{timer_ms(t0)});
    t0 = now_ns();

    // ── Stage 5b: Compute per-cell & per-vertex rotations ─────────────────
    const cell_rotations  = try allocator.alloc(f64, tet.cells.len);
    const vertex_rotations = try allocator.alloc(f64, tet.points.len);
    const num_cells_per_vert = try allocator.alloc(f64, tet.points.len);
    @memset(cell_rotations, 0);
    @memset(vertex_rotations, 0);
    @memset(num_cells_per_vert, 0);
    for (tet.cells, 0..) |cell, ci| {
        for (cell) |vi| num_cells_per_vert[vi] += 1;
        // Kabsch angle in radial plane
        const cc_old = blk: {
            var c = m.Vec3{ 0, 0, 0 };
            for (cell) |vi| c = m.add3(c, original_points[vi]);
            break :blk m.scale3(c, 0.25);
        };
        const cc_new = tet.cell_center[ci];
        var old_v: [4][2]f64 = undefined;
        var new_v: [4][2]f64 = undefined;
        const r = @sqrt(cc_old[0] * cc_old[0] + cc_old[1] * cc_old[1]);
        if (r < 1e-10) { cell_rotations[ci] = 0; continue; }
        const px = m.Vec3{ cc_old[0] / r, cc_old[1] / r, 0 };
        const py = m.Vec3{ 0, 0, 1 };
        for (0..4) |k| {
            const vo = m.sub3(original_points[cell[k]], cc_old);
            const vn = m.sub3(tet.points[cell[k]], cc_new);
            old_v[k] = m.project_onto_plane(px, py, vo);
            new_v[k] = m.project_onto_plane(px, py, vn);
        }
        var angle = m.kabsch_angle_2d(&new_v, &old_v);
        angle = std.math.clamp(angle,
            std.math.degreesToRadians(params.min_rotation_deg),
            std.math.degreesToRadians(params.max_rotation_deg));
        cell_rotations[ci] = angle;
        for (cell) |vi| vertex_rotations[vi] += angle / num_cells_per_vert[vi];
    }

    // Z-squish scales
    const z_squish = try allocator.alloc(f64, tet.cells.len);
    for (tet.cells, 0..) |cell, ci| {
        const a0 = original_points[cell[0]]; const b0 = original_points[cell[1]];
        const c0 = original_points[cell[2]]; const d0 = original_points[cell[3]];
        const a1 = tet.points[cell[0]]; const b1 = tet.points[cell[1]];
        const c1 = tet.points[cell[2]]; const d1 = tet.points[cell[3]];
        const vol_old = m.tet_volume(a0, b0, c0, d0);
        const vol_new = m.tet_volume(a1, b1, c1, d1);
        z_squish[ci] = if (vol_new > 1e-15) vol_old / vol_new else 1.0;
    }

    // ── Stage 6: Parse G-code + Barycentric mapping ───────────────────────
    std.debug.print("[6/7] Parsing G-code and mapping to original mesh...\n", .{});
    const gcode_points = try gcode.parse(params.input_gcode, init.io, allocator);
    std.debug.print("      {} gcode points parsed  ({} ms)\n", .{ gcode_points.len, timer_ms(t0) });
    t0 = now_ns();

    // Build spatial index on deformed mesh for containing-cell lookup
    var idx = try spatial.build(&tet, allocator);
    defer idx.deinit();

    // Transform each gcode point back to the original mesh coordinates
    var output_points = std.array_list.Managed(gcode.OutputPoint).init(allocator);
    var prev_rotation: f64 = 0;
    var prev_new_pos: ?m.Vec3 = null;
    var highest_z: f64 = 0;
    var travelling_over_air = false;
    const lost: usize = 0;

    const RETRACT = params.retraction_length;
    var is_travelling = false;

    for (gcode_points) |gp| {
        const pos = gp.position;
        var containing = idx.find_containing(pos);
        var new_pos: m.Vec3 = undefined;
        var rot: f64 = 0;

        if (containing < 0) {
            if (gp.command == .G01) {
                const closest = idx.find_closest(pos);
                containing = @intCast(closest);
            } else {
                // Travel over air
                if (!travelling_over_air and prev_new_pos != null) {
                    new_pos = .{ prev_new_pos.?[0], prev_new_pos.?[1], highest_z };
                    rot = std.math.clamp(prev_rotation, std.math.degreesToRadians(-45.0), std.math.degreesToRadians(45.0));
                    travelling_over_air = true;
                } else if (travelling_over_air) {
                    continue;
                } else {
                    continue;
                }
                try append_output(&output_points, new_pos, rot, &prev_rotation, prev_new_pos, &highest_z, gp, z_squish, is_travelling, params);
                prev_new_pos = new_pos;
                continue;
            }
        } else {
            travelling_over_air = false;
        }

        const ci: usize = @intCast(containing);
        const cell = tet.cells[ci];
        const ca = tet.points[cell[0]]; const cb = tet.points[cell[1]];
        const cc = tet.points[cell[2]]; const cd = tet.points[cell[3]];
        const bary = m.barycentric(ca, cb, cc, cd, pos);

        // Check if inside (sum ≈ 1 and all >= 0)
        const bsum = bary[0] + bary[1] + bary[2] + bary[3];
        if (@abs(bsum - 1.0) > 0.1 and gp.command == .G00) { continue; }

        // Interpolate transformation
        var transform = m.Vec3{ 0, 0, 0 };
        for (0..4) |k| {
            transform = m.add3(transform, m.scale3(vertex_transformations[cell[k]], bary[k]));
        }
        new_pos = m.sub3(pos, transform);

        // Interpolate rotation
        rot = 0;
        for (0..4) |k| rot += vertex_rotations[cell[k]] * bary[k];

        // Smooth rotation
        rot = params.rotation_avg_alpha * rot + (1.0 - params.rotation_avg_alpha) * prev_rotation;

        // Insert interpolation points if rotation delta too large
        if (prev_new_pos != null) {
            const delta_rot = rot - prev_rotation;
            const max_delta = std.math.degreesToRadians(params.rotation_max_delta_deg);
            if (@abs(delta_rot) > max_delta) {
                const num_interp: usize = @intFromFloat(@ceil(@abs(delta_rot) / max_delta));
                const delta_pos = m.sub3(new_pos, prev_new_pos.?);
                for (1..num_interp + 1) |si| {
                    const t = @as(f64, @floatFromInt(si)) / @as(f64, @floatFromInt(num_interp));
                    const interp_pos = m.add3(prev_new_pos.?, m.scale3(delta_pos, t));
                    const interp_rot = prev_rotation + delta_rot * t;
                    try output_points.append(.{
                        .position = interp_pos,
                        .rotation = interp_rot,
                        .command = gp.command,
                        .extrusion = if (gp.extrusion) |e| e / @as(f64, @floatFromInt(num_interp)) else null,
                        .inv_time_feed = if (gp.inv_time_feed) |f| f * @as(f64, @floatFromInt(num_interp)) else null,
                        .extrusion_multiplier = z_squish[ci],
                        .feed = gp.feed,
                        .travelling = is_travelling,
                    });
                }
                prev_rotation = rot;
                prev_new_pos = new_pos;
                if (gp.extrusion) |e| {
                    if (e > 0 and gp.command == .G01) highest_z = @max(highest_z, new_pos[2]);
                    if (e == -RETRACT) is_travelling = true;
                    if (e == RETRACT)  is_travelling = false;
                }
                continue;
            }
        }

        try output_points.append(.{
            .position = new_pos,
            .rotation = rot,
            .command = gp.command,
            .extrusion = gp.extrusion,
            .inv_time_feed = gp.inv_time_feed,
            .extrusion_multiplier = z_squish[ci],
            .feed = gp.feed,
            .travelling = is_travelling,
        });
        prev_rotation = rot;
        prev_new_pos = new_pos;
        if (gp.extrusion) |e| {
            if (e > 0 and gp.command == .G01) highest_z = @max(highest_z, new_pos[2]);
            if (e == -RETRACT) is_travelling = true;
            if (e == RETRACT)  is_travelling = false;
        }
    }
    _ = lost;
    std.debug.print("      {} output points  ({} ms)\n", .{ output_points.items.len, timer_ms(t0) });
    t0 = now_ns();

    // ── Stage 7: Write polar G-code ───────────────────────────────────────
    std.debug.print("[7/7] Writing polar G-code: {s}\n", .{params.output_gcode});
    try gcode.write_polar(params.output_gcode, output_points.items, .{
        .nozzle_offset = params.nozzle_offset,
        .retraction_length = RETRACT,
        .rotation_avg_alpha = params.rotation_avg_alpha,
        .rotation_max_delta_deg = params.rotation_max_delta_deg,
        .max_extrusion_multiplier = params.max_extrusion_multiplier,
        .min_rotation_deg = params.min_rotation_deg,
        .max_rotation_deg = params.max_rotation_deg,
    }, init.io, allocator);
    std.debug.print("      Done.  ({} ms)\n", .{timer_ms(t0)});
}

fn append_output(
    list: *std.array_list.Managed(gcode.OutputPoint),
    new_pos: m.Vec3,
    rot: f64,
    prev_rotation: *f64,
    prev_new_pos: ?m.Vec3,
    highest_z: *f64,
    gp: gcode.GcodePoint,
    z_squish: []const f64,
    is_travelling: bool,
    params: Params,
) !void {
    _ = prev_new_pos;
    _ = highest_z;
    _ = z_squish;
    _ = params;
    try list.append(.{
        .position = new_pos,
        .rotation = rot,
        .command = gp.command,
        .extrusion = gp.extrusion,
        .inv_time_feed = gp.inv_time_feed,
        .extrusion_multiplier = 1.0,
        .feed = gp.feed,
        .travelling = is_travelling,
    });
    prev_rotation.* = rot;
}

// ── TetGen bridge ─────────────────────────────────────────────────────────
const TetgenResult = struct {
    points: []f64,
    tets: []c_int,
    num_verts: usize,
    num_tets: usize,
};

extern fn tet_mesh(
    vertices: [*]const f64, num_verts: c_int,
    triangles: [*]const c_int, num_triangles: c_int,
) ?*anyopaque;
extern fn tet_result_free(r: ?*anyopaque) void;

// Mirror of the C struct layout (must match tetgen_bridge.h TetResult)
const TetResultC = extern struct {
    points: [*]f64,
    tetrahedra: [*]c_int,
    num_points: c_int,
    num_tets: c_int,
};

fn tetrahedralize(surface: *const stl.Mesh, allocator: std.mem.Allocator) !TetgenResult {
    const verts_raw = try allocator.alloc(f64, surface.vertices.len * 3);
    defer allocator.free(verts_raw);
    for (surface.vertices, 0..) |v, i| {
        verts_raw[i * 3 + 0] = v[0]; verts_raw[i * 3 + 1] = v[1]; verts_raw[i * 3 + 2] = v[2];
    }

    const tris_raw = try allocator.alloc(c_int, surface.triangles.len * 3);
    defer allocator.free(tris_raw);
    for (surface.triangles, 0..) |t, i| {
        tris_raw[i * 3 + 0] = @intCast(t[0]);
        tris_raw[i * 3 + 1] = @intCast(t[1]);
        tris_raw[i * 3 + 2] = @intCast(t[2]);
    }

    const handle = tet_mesh(
        verts_raw.ptr, @intCast(surface.vertices.len),
        tris_raw.ptr,  @intCast(surface.triangles.len),
    ) orelse return error.TetgenFailed;

    const res: *TetResultC = @alignCast(@ptrCast(handle));
    const np: usize = @intCast(res.num_points);
    const nt: usize = @intCast(res.num_tets);

    // Copy out before freeing
    const pts = try allocator.alloc(f64, np * 3);
    @memcpy(pts, res.points[0 .. np * 3]);
    const tets = try allocator.alloc(c_int, nt * 4);
    @memcpy(tets, res.tetrahedra[0 .. nt * 4]);

    tet_result_free(handle);

    return .{ .points = pts, .tets = tets, .num_verts = np, .num_tets = nt };
}

// ── Tests ──────────────────────────────────────────────────────────────────
test "version non-empty" {
    try std.testing.expect(version.len > 0);
}
test "help text has usage" {
    try std.testing.expect(std.mem.indexOf(u8, help_text, "Usage:") != null);
}
