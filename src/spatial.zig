// Grid-based spatial hash for O(1) average-case containing-cell lookup.
// Replaces pyvista's find_containing_cell which is O(n) per query.
const std = @import("std");
const m = @import("math.zig");
const mesh = @import("mesh.zig");

pub const SpatialIndex = struct {
    grid_min: m.Vec3,
    cell_size: f64,
    dims: [3]u32,
    // Each bucket: list of tet indices that overlap it
    buckets: []std.array_list.Managed(u32),
    tet: *const mesh.TetMesh,
    allocator: std.mem.Allocator,

    pub fn deinit(self: *SpatialIndex) void {
        for (self.buckets) |*b| b.deinit();
        self.allocator.free(self.buckets);
    }

    fn bucket_idx(self: SpatialIndex, ix: u32, iy: u32, iz: u32) usize {
        return @as(usize, iz) * self.dims[1] * self.dims[0] +
               @as(usize, iy) * self.dims[0] +
               @as(usize, ix);
    }

    fn cell_coords(self: SpatialIndex, p: m.Vec3) [3]i32 {
        return .{
            @intFromFloat(@floor((p[0] - self.grid_min[0]) / self.cell_size)),
            @intFromFloat(@floor((p[1] - self.grid_min[1]) / self.cell_size)),
            @intFromFloat(@floor((p[2] - self.grid_min[2]) / self.cell_size)),
        };
    }

    // Return index of tet containing point, or -1.
    pub fn find_containing(self: SpatialIndex, p: m.Vec3) i32 {
        const gc = self.cell_coords(p);
        // Search a small neighbourhood in case point is on boundary
        var best: i32 = -1;
        var best_sum: f64 = std.math.inf(f64);

        for ([_]i32{ -1, 0, 1 }) |dz| {
            for ([_]i32{ -1, 0, 1 }) |dy| {
                for ([_]i32{ -1, 0, 1 }) |dx| {
                    const ix = gc[0] + dx; const iy = gc[1] + dy; const iz = gc[2] + dz;
                    if (ix < 0 or iy < 0 or iz < 0) continue;
                    if (ix >= self.dims[0] or iy >= self.dims[1] or iz >= self.dims[2]) continue;
                    const bi = self.bucket_idx(@intCast(ix), @intCast(iy), @intCast(iz));
                    for (self.buckets[bi].items) |ti| {
                        const cell = self.tet.cells[ti];
                        const a = self.tet.points[cell[0]]; const b = self.tet.points[cell[1]];
                        const c = self.tet.points[cell[2]]; const d = self.tet.points[cell[3]];
                        const bary = m.barycentric(a, b, c, d, p);
                        const bsum = bary[0] + bary[1] + bary[2] + bary[3];
                        // Inside if all >= -eps and sum ~1
                        if (bary[0] >= -0.01 and bary[1] >= -0.01 and
                            bary[2] >= -0.01 and bary[3] >= -0.01 and
                            @abs(bsum - 1.0) < 0.05)
                        {
                            return @intCast(ti);
                        }
                        // Track closest (for fallback)
                        const err = @abs(bsum - 1.0);
                        if (err < best_sum) { best_sum = err; best = @intCast(ti); }
                    }
                }
            }
        }
        return -1; // not found; caller should use closest
    }

    // Find closest tet to a point (brute force over all tets — only called for edge cases)
    pub fn find_closest(self: SpatialIndex, p: m.Vec3) u32 {
        var best_dist: f64 = std.math.inf(f64);
        var best_idx: u32 = 0;
        for (self.tet.cell_center, 0..) |cc, i| {
            const d = m.norm3(m.sub3(cc, p));
            if (d < best_dist) { best_dist = d; best_idx = @intCast(i); }
        }
        return best_idx;
    }
};

pub fn build(tet: *const mesh.TetMesh, allocator: std.mem.Allocator) !SpatialIndex {
    var xmin: f64 = std.math.inf(f64); var xmax: f64 = -std.math.inf(f64);
    var ymin: f64 = std.math.inf(f64); var ymax: f64 = -std.math.inf(f64);
    var zmin: f64 = std.math.inf(f64); var zmax: f64 = -std.math.inf(f64);
    for (tet.points) |p| {
        if (p[0] < xmin) xmin = p[0]; if (p[0] > xmax) xmax = p[0];
        if (p[1] < ymin) ymin = p[1]; if (p[1] > ymax) ymax = p[1];
        if (p[2] < zmin) zmin = p[2]; if (p[2] > zmax) zmax = p[2];
    }

    // Aim for ~5 tets per bucket on average
    const avg_tet_vol = ((xmax - xmin) * (ymax - ymin) * (zmax - zmin)) / @as(f64, @floatFromInt(tet.cells.len));
    const cell_size = @max(0.5, std.math.pow(f64, avg_tet_vol * 5.0, 1.0 / 3.0));

    const margin = cell_size;
    const gmin = m.Vec3{ xmin - margin, ymin - margin, zmin - margin };
    const dims = [3]u32{
        @intFromFloat(@ceil((xmax - xmin + 2 * margin) / cell_size)),
        @intFromFloat(@ceil((ymax - ymin + 2 * margin) / cell_size)),
        @intFromFloat(@ceil((zmax - zmin + 2 * margin) / cell_size)),
    };

    const total = @as(usize, dims[0]) * dims[1] * dims[2];
    const buckets = try allocator.alloc(std.array_list.Managed(u32), total);
    for (buckets) |*b| b.* = std.array_list.Managed(u32).init(allocator);

    // Insert each tet into the buckets its AABB overlaps
    for (tet.cells, 0..) |cell, ti| {
        var tmin = m.Vec3{ std.math.inf(f64), std.math.inf(f64), std.math.inf(f64) };
        var tmax = m.Vec3{ -std.math.inf(f64), -std.math.inf(f64), -std.math.inf(f64) };
        for (cell) |vi| {
            const p = tet.points[vi];
            inline for (0..3) |ax| {
                if (p[ax] < tmin[ax]) tmin[ax] = p[ax];
                if (p[ax] > tmax[ax]) tmax[ax] = p[ax];
            }
        }

        const lo = [3]i32{
            @intFromFloat(@floor((tmin[0] - gmin[0]) / cell_size)),
            @intFromFloat(@floor((tmin[1] - gmin[1]) / cell_size)),
            @intFromFloat(@floor((tmin[2] - gmin[2]) / cell_size)),
        };
        const hi = [3]i32{
            @intFromFloat(@ceil((tmax[0] - gmin[0]) / cell_size)),
            @intFromFloat(@ceil((tmax[1] - gmin[1]) / cell_size)),
            @intFromFloat(@ceil((tmax[2] - gmin[2]) / cell_size)),
        };

        var iz = @max(lo[2], 0);
        while (iz <= @min(hi[2], @as(i32, @intCast(dims[2])) - 1)) : (iz += 1) {
            var iy = @max(lo[1], 0);
            while (iy <= @min(hi[1], @as(i32, @intCast(dims[1])) - 1)) : (iy += 1) {
                var ix = @max(lo[0], 0);
                while (ix <= @min(hi[0], @as(i32, @intCast(dims[0])) - 1)) : (ix += 1) {
                    const bi = @as(usize, @intCast(iz)) * dims[1] * dims[0] +
                               @as(usize, @intCast(iy)) * dims[0] +
                               @as(usize, @intCast(ix));
                    try buckets[bi].append(@intCast(ti));
                }
            }
        }
    }

    return .{
        .grid_min = gmin,
        .cell_size = cell_size,
        .dims = dims,
        .buckets = buckets,
        .tet = tet,
        .allocator = allocator,
    };
}
