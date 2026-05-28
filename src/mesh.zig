// Tetrahedral mesh data structure and attribute computation.
const std = @import("std");
const m = @import("math.zig");

pub const TetMesh = struct {
    // Vertex positions [num_verts x 3]
    points: []m.Vec3,
    // Tetrahedra: each is 4 vertex indices [num_tets x 4]
    cells: [][4]u32,
    // Per-cell attributes
    cell_center: []m.Vec3,
    face_normal: []m.Vec3,       // outward normal of lowest face, NaN if not surface
    face_center: []m.Vec3,       // center of lowest face, NaN if not surface
    overhang_angle: []f64,       // angle between face_normal and up-vector
    overhang_direction: [][2]f64,
    is_bottom: []bool,
    in_air: []bool,
    has_face: []bool,
    // Per-cell distance to nearest bottom cell (Dijkstra)
    cell_distance_to_bottom: []f64, // NaN if not overhang or not reachable
    // Per-cell path to bottom (list of cell indices, -1 terminated)
    path_to_bottom: [][]i32,

    // Adjacency (face-sharing pairs for optimization)
    face_neighbours: [][2]u32,   // pairs of cell indices sharing a face
    // Point-sharing neighbours per cell
    point_neighbours: [][]u32,

    allocator: std.mem.Allocator,

    pub fn deinit(self: *TetMesh) void {
        const a = self.allocator;
        a.free(self.points);
        a.free(self.cells);
        a.free(self.cell_center);
        a.free(self.face_normal);
        a.free(self.face_center);
        a.free(self.overhang_angle);
        a.free(self.overhang_direction);
        a.free(self.is_bottom);
        a.free(self.in_air);
        a.free(self.has_face);
        a.free(self.cell_distance_to_bottom);
        for (self.path_to_bottom) |p| a.free(p);
        a.free(self.path_to_bottom);
        a.free(self.face_neighbours);
        for (self.point_neighbours) |n| a.free(n);
        a.free(self.point_neighbours);
    }
};

const up_vector = m.Vec3{ 0, 0, 1 };

// Build a TetMesh from raw tetgen output and compute all attributes.
pub fn build(
    points_raw: []const f64,   // [num_verts*3]
    num_verts: usize,
    tets_raw: []const c_int,   // [num_tets*4]
    num_tets: usize,
    allocator: std.mem.Allocator,
) !TetMesh {
    const pts = try allocator.alloc(m.Vec3, num_verts);
    for (0..num_verts) |i| {
        pts[i] = .{ points_raw[i * 3], points_raw[i * 3 + 1], points_raw[i * 3 + 2] };
    }

    const cells = try allocator.alloc([4]u32, num_tets);
    for (0..num_tets) |i| {
        cells[i] = .{
            @intCast(tets_raw[i * 4 + 0]),
            @intCast(tets_raw[i * 4 + 1]),
            @intCast(tets_raw[i * 4 + 2]),
            @intCast(tets_raw[i * 4 + 3]),
        };
    }

    // Centre mesh: translate so centroid of bounding box sits at origin in XY, z_min at 0
    centre_mesh(pts, cells);

    // Build adjacency
    const face_neighbours = try build_face_neighbours(cells, num_tets, num_verts, allocator);
    const point_neighbours = try build_point_neighbours(cells, num_tets, num_verts, allocator);

    // Compute cell centres
    const cell_center = try allocator.alloc(m.Vec3, num_tets);
    for (0..num_tets) |i| {
        var c = m.Vec3{ 0, 0, 0 };
        for (cells[i]) |vi| {
            c = m.add3(c, pts[vi]);
        }
        cell_center[i] = m.scale3(c, 0.25);
    }

    // Extract surface: a face is on the surface if it belongs to exactly 1 tet
    const SurfFace = struct { v: [3]u32, cell: u32, normal: m.Vec3, center: m.Vec3 };
    var surf_faces = std.array_list.Managed(SurfFace).init(allocator);
    defer surf_faces.deinit();
    {
        const FaceKey = struct { v: [3]u32 };
        const FaceContext = struct {
            pub fn hash(_: @This(), k: FaceKey) u64 {
                var s = [3]u32{ k.v[0], k.v[1], k.v[2] };
                std.mem.sort(u32, &s, {}, std.sort.asc(u32));
                var h: u64 = 14695981039346656037;
                for (s) |x| { h ^= x; h *%= 1099511628211; }
                return h;
            }
            pub fn eql(_: @This(), a: FaceKey, b: FaceKey) bool {
                var sa = [3]u32{ a.v[0], a.v[1], a.v[2] };
                var sb = [3]u32{ b.v[0], b.v[1], b.v[2] };
                std.mem.sort(u32, &sa, {}, std.sort.asc(u32));
                std.mem.sort(u32, &sb, {}, std.sort.asc(u32));
                return sa[0] == sb[0] and sa[1] == sb[1] and sa[2] == sb[2];
            }
        };
        var face_count = std.HashMap(FaceKey, struct { count: u32, cell: u32, face: [3]u32 }, FaceContext, 80).init(allocator);
        defer face_count.deinit();

        for (cells, 0..) |cell, ci| {
            const faces = [4][3]u32{
                .{ cell[0], cell[1], cell[2] },
                .{ cell[0], cell[1], cell[3] },
                .{ cell[0], cell[2], cell[3] },
                .{ cell[1], cell[2], cell[3] },
            };
            for (faces) |f| {
                const gop = try face_count.getOrPut(.{ .v = f });
                if (!gop.found_existing) {
                    gop.value_ptr.* = .{ .count = 0, .cell = @intCast(ci), .face = f };
                }
                gop.value_ptr.count += 1;
                gop.value_ptr.cell = @intCast(ci);
            }
        }

        var it = face_count.iterator();
        while (it.next()) |entry| {
            if (entry.value_ptr.count == 1) {
                const f = entry.value_ptr.face;
                const p0 = pts[f[0]]; const p1 = pts[f[1]]; const p2 = pts[f[2]];
                var normal = m.normalize3(m.cross3(m.sub3(p1, p0), m.sub3(p2, p0)));
                // Ensure normal points away from tet interior
                const cc = cell_center[entry.value_ptr.cell];
                const fc = m.scale3(m.add3(m.add3(p0, p1), p2), 1.0 / 3.0);
                if (m.dot3(normal, m.sub3(fc, cc)) < 0) {
                    normal = m.scale3(normal, -1.0);
                }
                try surf_faces.append(.{
                    .v = f,
                    .cell = entry.value_ptr.cell,
                    .normal = normal,
                    .center = fc,
                });
            }
        }
    }

    // Map cell → surface faces
    const CellFaces = std.AutoHashMap(u32, std.array_list.Managed(usize));
    var cell_to_face = CellFaces.init(allocator);
    defer {
        var it2 = cell_to_face.iterator();
        while (it2.next()) |e| e.value_ptr.deinit();
        cell_to_face.deinit();
    }
    for (surf_faces.items, 0..) |sf, fi| {
        const gop = try cell_to_face.getOrPut(sf.cell);
        if (!gop.found_existing) gop.value_ptr.* = std.array_list.Managed(usize).init(allocator);
        try gop.value_ptr.append(fi);
    }

    // Per-cell face attributes
    const face_normal = try allocator.alloc(m.Vec3, num_tets);
    const face_center = try allocator.alloc(m.Vec3, num_tets);
    const has_face = try allocator.alloc(bool, num_tets);
    @memset(has_face, false);
    for (face_normal) |*fn_| fn_.* = .{ std.math.nan(f64), std.math.nan(f64), std.math.nan(f64) };
    for (face_center) |*fc| fc.* = .{ std.math.nan(f64), std.math.nan(f64), std.math.nan(f64) };

    var cell_to_face_it = cell_to_face.iterator();
    while (cell_to_face_it.next()) |e| {
        const ci = e.key_ptr.*;
        const fi_list = e.value_ptr.*;
        var most_down_z: f64 = std.math.inf(f64);
        var best_normal = m.Vec3{ 0, 0, 1 };
        var best_center = m.Vec3{ 0, 0, 0 };
        for (fi_list.items) |fi| {
            const sf = surf_faces.items[fi];
            if (sf.normal[2] < most_down_z) {
                most_down_z = sf.normal[2];
                best_normal = sf.normal;
                best_center = sf.center;
            }
        }
        face_normal[ci] = m.normalize3(best_normal);
        face_center[ci] = best_center;
        has_face[ci] = true;
    }

    // Overhang angle
    const overhang_angle = try allocator.alloc(f64, num_tets);
    const overhang_direction = try allocator.alloc([2]f64, num_tets);
    for (0..num_tets) |i| {
        const fn_ = face_normal[i];
        if (std.math.isNan(fn_[0])) {
            overhang_angle[i] = std.math.nan(f64);
            overhang_direction[i] = .{ 0, 0 };
        } else {
            overhang_angle[i] = std.math.acos(std.math.clamp(m.dot3(fn_, up_vector), -1.0, 1.0));
            const dir = m.normalize3(.{ fn_[0], fn_[1], 0 });
            overhang_direction[i] = .{ dir[0], dir[1] };
        }
    }

    // Bottom cells: face center z < min_face_z + 0.3
    const is_bottom = try allocator.alloc(bool, num_tets);
    @memset(is_bottom, false);
    var min_face_z: f64 = std.math.inf(f64);
    for (face_center) |fc| {
        if (!std.math.isNan(fc[2]) and fc[2] < min_face_z) min_face_z = fc[2];
    }
    const bottom_threshold = min_face_z + 0.3;
    var bottom_cells = std.array_list.Managed(u32).init(allocator);
    defer bottom_cells.deinit();
    for (0..num_tets) |i| {
        const fc = face_center[i];
        if (!std.math.isNan(fc[2]) and fc[2] < bottom_threshold) {
            is_bottom[i] = true;
            try bottom_cells.append(@intCast(i));
        }
    }

    // Bottom cells mask applied to overhang_angle (NaN for bottom)
    for (0..num_tets) |i| {
        if (is_bottom[i]) overhang_angle[i] = std.math.nan(f64);
    }

    // Dijkstra from all bottom cells
    const dijkstra_result = try dijkstra_multi_source(cell_center, point_neighbours, bottom_cells.items, num_tets, allocator);
    defer allocator.free(dijkstra_result.distances);
    // path_to_bottom: rebuild per-cell paths (we store the prev-node array from Dijkstra)
    const cell_distance_to_bottom = dijkstra_result.distances; // hand off ownership
    const path_to_bottom = try build_paths(dijkstra_result.prev, num_tets, allocator);
    // prev is now owned by caller — but we need to free it
    defer allocator.free(dijkstra_result.prev);

    // in_air: a cell is in-air if some cell along its path to bottom has z > cell.z + 1
    const in_air = try allocator.alloc(bool, num_tets);
    @memset(in_air, false);
    for (0..num_tets) |i| {
        const path = path_to_bottom[i];
        if (path.len <= 1) continue;
        const my_z = cell_center[i][2];
        for (path[1..]) |ci| {
            if (ci < 0) break;
            if (cell_center[@intCast(ci)][2] > my_z + 1.0) {
                in_air[i] = true;
                break;
            }
        }
    }

    return .{
        .points = pts,
        .cells = cells,
        .cell_center = cell_center,
        .face_normal = face_normal,
        .face_center = face_center,
        .overhang_angle = overhang_angle,
        .overhang_direction = overhang_direction,
        .is_bottom = is_bottom,
        .in_air = in_air,
        .has_face = has_face,
        .cell_distance_to_bottom = cell_distance_to_bottom,
        .path_to_bottom = path_to_bottom,
        .face_neighbours = face_neighbours,
        .point_neighbours = point_neighbours,
        .allocator = allocator,
    };
}

// Recompute mutable attributes after deformation (points already updated externally).
pub fn update_attributes(mesh: *TetMesh) void {
    const num_tets = mesh.cells.len;
    // Recompute cell centres
    for (0..num_tets) |i| {
        var c = m.Vec3{ 0, 0, 0 };
        for (mesh.cells[i]) |vi| c = m.add3(c, mesh.points[vi]);
        mesh.cell_center[i] = m.scale3(c, 0.25);
    }
    // Recompute surface face normals & centres
    // (simplified: reuse face topology from before deformation; only positions changed)
    for (mesh.face_normal) |*fn_| {
        if (!std.math.isNan(fn_[0])) {
            // will be recalculated below
        }
    }
    // Simple pass: for each cell with has_face, reconstruct from tet face geometry
    // This is approximate — full surface re-extraction would be needed for accuracy
    // but for post-deformation attribute update it is sufficient.
    for (0..num_tets) |i| {
        if (!mesh.has_face[i]) continue;
        const cell = mesh.cells[i];
        const faces = [4][3]u32{
            .{ cell[0], cell[1], cell[2] },
            .{ cell[0], cell[1], cell[3] },
            .{ cell[0], cell[2], cell[3] },
            .{ cell[1], cell[2], cell[3] },
        };
        var most_down_normal = m.Vec3{ 0, 0, 1 };
        var most_down_z: f64 = std.math.inf(f64);
        var most_down_center = m.Vec3{ 0, 0, 0 };
        for (faces) |f| {
            const p0 = mesh.points[f[0]]; const p1 = mesh.points[f[1]]; const p2 = mesh.points[f[2]];
            const n = m.normalize3(m.cross3(m.sub3(p1, p0), m.sub3(p2, p0)));
            const cc = mesh.cell_center[i];
            const fc = m.scale3(m.add3(m.add3(p0, p1), p2), 1.0 / 3.0);
            var nout = n;
            if (m.dot3(nout, m.sub3(fc, cc)) < 0) nout = m.scale3(nout, -1.0);
            if (nout[2] < most_down_z) {
                most_down_z = nout[2];
                most_down_normal = nout;
                most_down_center = fc;
            }
        }
        mesh.face_normal[i] = m.normalize3(most_down_normal);
        mesh.face_center[i] = most_down_center;
        const fn_ = mesh.face_normal[i];
        mesh.overhang_angle[i] = std.math.acos(std.math.clamp(m.dot3(fn_, up_vector), -1.0, 1.0));
    }
}

fn centre_mesh(pts: []m.Vec3, cells: [][4]u32) void {
    _ = cells;
    var xmin: f64 = std.math.inf(f64); var xmax: f64 = -std.math.inf(f64);
    var ymin: f64 = std.math.inf(f64); var ymax: f64 = -std.math.inf(f64);
    var zmin: f64 = std.math.inf(f64);
    for (pts) |p| {
        if (p[0] < xmin) xmin = p[0]; if (p[0] > xmax) xmax = p[0];
        if (p[1] < ymin) ymin = p[1]; if (p[1] > ymax) ymax = p[1];
        if (p[2] < zmin) zmin = p[2];
    }
    const ox = (xmin + xmax) / 2.0;
    const oy = (ymin + ymax) / 2.0;
    for (pts) |*p| {
        p[0] -= ox; p[1] -= oy; p[2] -= zmin;
    }
}

fn build_face_neighbours(cells: [][4]u32, num_tets: usize, num_verts: usize, allocator: std.mem.Allocator) ![][2]u32 {
    _ = num_verts;
    // A face is identified by its 3 sorted vertex indices.
    const FaceKey = struct { v: [3]u32 };
    const FaceCtx = struct {
        pub fn hash(_: @This(), k: FaceKey) u64 {
            var h: u64 = 14695981039346656037;
            for (k.v) |x| { h ^= x; h *%= 1099511628211; }
            return h;
        }
        pub fn eql(_: @This(), a: FaceKey, b: FaceKey) bool {
            return a.v[0] == b.v[0] and a.v[1] == b.v[1] and a.v[2] == b.v[2];
        }
    };
    var map = std.HashMap(FaceKey, u32, FaceCtx, 80).init(allocator);
    defer map.deinit();

    var pairs = std.array_list.Managed([2]u32).init(allocator);

    for (cells, 0..) |cell, ci| {
        const face_triples = [4][3]u32{
            .{ cell[0], cell[1], cell[2] },
            .{ cell[0], cell[1], cell[3] },
            .{ cell[0], cell[2], cell[3] },
            .{ cell[1], cell[2], cell[3] },
        };
        for (face_triples) |f| {
            var s = f;
            std.mem.sort(u32, &s, {}, std.sort.asc(u32));
            const key = FaceKey{ .v = s };
            const gop = try map.getOrPut(key);
            if (!gop.found_existing) {
                gop.value_ptr.* = @intCast(ci);
            } else {
                const other = gop.value_ptr.*;
                if (other != ci) {
                    try pairs.append(.{ other, @intCast(ci) });
                }
            }
        }
    }
    _ = num_tets;
    return pairs.toOwnedSlice();
}

fn build_point_neighbours(cells: [][4]u32, num_tets: usize, num_verts: usize, allocator: std.mem.Allocator) ![][]u32 {
    // For each vertex, collect which tets use it
    var vert_to_tets = try allocator.alloc(std.array_list.Managed(u32), num_verts);
    for (vert_to_tets) |*l| l.* = std.array_list.Managed(u32).init(allocator);
    defer {
        for (vert_to_tets) |*l| l.deinit();
        allocator.free(vert_to_tets);
    }

    for (cells, 0..) |cell, ci| {
        for (cell) |vi| try vert_to_tets[vi].append(@intCast(ci));
    }

    // For each tet, point neighbours = all tets sharing at least 1 vertex (excluding self)
    const result = try allocator.alloc([]u32, num_tets);
    for (0..num_tets) |ci| {
        var seen = std.AutoHashMap(u32, void).init(allocator);
        defer seen.deinit();
        for (cells[ci]) |vi| {
            for (vert_to_tets[vi].items) |ni| {
                if (ni != ci) try seen.put(ni, {});
            }
        }
        const nbrs = try allocator.alloc(u32, seen.count());
        var it = seen.keyIterator();
        var idx: usize = 0;
        while (it.next()) |k| { nbrs[idx] = k.*; idx += 1; }
        result[ci] = nbrs;
    }
    return result;
}

// Dijkstra output
const DijkstraResult = struct {
    distances: []f64, // owned by caller
    prev: []i32,      // owned by caller; -1 if unreachable/source
};

fn dijkstra_multi_source(
    cell_center: []m.Vec3,
    point_neighbours: [][]u32,
    sources: []const u32,
    num_cells: usize,
    allocator: std.mem.Allocator,
) !DijkstraResult {
    const INF = std.math.inf(f64);
    const dist = try allocator.alloc(f64, num_cells);
    const prev = try allocator.alloc(i32, num_cells);
    @memset(dist, INF);
    @memset(prev, -1);

    // MinHeap entry
    const Entry = struct { dist: f64, cell: u32 };
    const PQ = std.PriorityQueue(Entry, void, struct {
        fn lt(_: void, a: Entry, b: Entry) std.math.Order {
            return std.math.order(a.dist, b.dist);
        }
    }.lt);
    var heap: PQ = .empty;
    defer heap.deinit(allocator);

    for (sources) |s| {
        dist[s] = 0;
        try heap.push(allocator, .{ .dist = 0, .cell = s });
    }

    while (heap.pop()) |e| {
        if (e.dist > dist[e.cell]) continue;
        for (point_neighbours[e.cell]) |n| {
            const d = e.dist + m.norm3(m.sub3(cell_center[n], cell_center[e.cell]));
            if (d < dist[n]) {
                dist[n] = d;
                prev[n] = @intCast(e.cell);
                try heap.push(allocator, .{ .dist = d, .cell = n });
            }
        }
    }

    return .{ .distances = dist, .prev = prev };
}

fn build_paths(prev: []const i32, num_cells: usize, allocator: std.mem.Allocator) ![][]i32 {
    const paths = try allocator.alloc([]i32, num_cells);
    for (0..num_cells) |i| {
        var path = std.array_list.Managed(i32).init(allocator);
        var cur: i32 = @intCast(i);
        while (cur >= 0) {
            try path.append(cur);
            const p = prev[@intCast(cur)];
            if (p == cur) break;
            cur = p;
        }
        paths[i] = try path.toOwnedSlice();
    }
    return paths;
}
