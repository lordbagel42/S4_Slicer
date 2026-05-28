const std = @import("std");
const math = @import("math.zig");

pub const Mesh = struct {
    vertices: []math.Vec3,
    triangles: [][3]u32,
    allocator: std.mem.Allocator,

    pub fn deinit(self: *Mesh) void {
        self.allocator.free(self.vertices);
        self.allocator.free(self.triangles);
    }
};

pub fn load(path: []const u8, io: std.Io, allocator: std.mem.Allocator) !Mesh {
    var file = try std.Io.Dir.cwd().openFile(io, path, .{});
    defer file.close(io);

    const stat = try file.stat(io);
    const size: usize = @intCast(stat.size);
    const data = try allocator.alloc(u8, size);
    defer allocator.free(data);

    const fd = file.handle;
    var got: usize = 0;
    while (got < size) {
        const n = try std.posix.read(fd, data[got..]);
        if (n == 0) break;
        got += n;
    }

    if (detect_binary(data)) {
        return load_binary(data, allocator);
    } else {
        return load_ascii(data, allocator);
    }
}

fn detect_binary(data: []const u8) bool {
    if (data.len < 84) return false;
    const num_tris = std.mem.readInt(u32, data[80..84], .little);
    const expected_len: usize = 84 + @as(usize, num_tris) * 50;
    return data.len == expected_len;
}

const VertexMap = std.HashMap(
    [3]f64,
    u32,
    struct {
        pub fn hash(_: @This(), key: [3]f64) u64 {
            var h: u64 = 14695981039346656037;
            for (key) |f| {
                const bits: u64 = @bitCast(f);
                h ^= bits;
                h *%= 1099511628211;
            }
            return h;
        }
        pub fn eql(_: @This(), a: [3]f64, b: [3]f64) bool {
            return a[0] == b[0] and a[1] == b[1] and a[2] == b[2];
        }
    },
    std.hash_map.default_max_load_percentage,
);

fn load_binary(data: []const u8, allocator: std.mem.Allocator) !Mesh {
    if (data.len < 84) return error.InvalidStl;
    const num_tris = std.mem.readInt(u32, data[80..84], .little);

    // First pass: collect raw vertices (3 per triangle) without dedup.
    const raw_verts = try allocator.alloc(math.Vec3, num_tris * 3);
    defer allocator.free(raw_verts);
    var offset: usize = 84;
    for (0..num_tris) |ti| {
        if (offset + 50 > data.len) break;
        offset += 12; // skip normal
        for (0..3) |k| {
            const fx: f32 = @bitCast(std.mem.readInt(u32, data[offset..][0..4], .little));
            const fy: f32 = @bitCast(std.mem.readInt(u32, data[offset + 4 ..][0..4], .little));
            const fz: f32 = @bitCast(std.mem.readInt(u32, data[offset + 8 ..][0..4], .little));
            offset += 12;
            raw_verts[ti * 3 + k] = .{ fx, fy, fz };
        }
        offset += 2;
    }

    // Second pass: sort-based dedup using indirect indices.
    const order = try allocator.alloc(u32, raw_verts.len);
    defer allocator.free(order);
    for (order, 0..) |*o, i| o.* = @intCast(i);
    const lessThan = struct {
        fn lt(verts_ref: []const math.Vec3, a: u32, b: u32) bool {
            const va = verts_ref[a]; const vb = verts_ref[b];
            if (va[0] != vb[0]) return va[0] < vb[0];
            if (va[1] != vb[1]) return va[1] < vb[1];
            return va[2] < vb[2];
        }
    }.lt;
    std.mem.sort(u32, order, @as([]const math.Vec3, raw_verts), lessThan);

    const remap = try allocator.alloc(u32, raw_verts.len);
    defer allocator.free(remap);
    var verts = std.array_list.Managed(math.Vec3).init(allocator);
    defer verts.deinit();
    try verts.ensureTotalCapacity(raw_verts.len);

    var i: usize = 0;
    while (i < order.len) {
        const v = raw_verts[order[i]];
        const idx: u32 = @intCast(verts.items.len);
        try verts.append(v);
        remap[order[i]] = idx;
        var j: usize = i + 1;
        while (j < order.len) : (j += 1) {
            const w = raw_verts[order[j]];
            if (w[0] != v[0] or w[1] != v[1] or w[2] != v[2]) break;
            remap[order[j]] = idx;
        }
        i = j;
    }

    var tris = std.array_list.Managed([3]u32).init(allocator);
    defer tris.deinit();
    try tris.ensureTotalCapacity(num_tris);
    for (0..num_tris) |ti| {
        try tris.append(.{
            remap[ti * 3 + 0],
            remap[ti * 3 + 1],
            remap[ti * 3 + 2],
        });
    }

    return .{
        .vertices = try verts.toOwnedSlice(),
        .triangles = try tris.toOwnedSlice(),
        .allocator = allocator,
    };
}

fn load_ascii(data: []const u8, allocator: std.mem.Allocator) !Mesh {
    var vmap = VertexMap.init(allocator);
    defer vmap.deinit();
    var verts = std.array_list.Managed(math.Vec3).init(allocator);
    defer verts.deinit();
    var tris = std.array_list.Managed([3]u32).init(allocator);
    defer tris.deinit();

    var it = std.mem.tokenizeAny(u8, data, "\n\r");
    var tri: [3]u32 = undefined;
    var vcount: usize = 0;

    while (it.next()) |line| {
        const trimmed = std.mem.trim(u8, line, " \t");
        if (std.mem.startsWith(u8, trimmed, "vertex")) {
            var parts = std.mem.tokenizeScalar(u8, trimmed[6..], ' ');
            const x = try std.fmt.parseFloat(f64, parts.next() orelse return error.InvalidStl);
            const y = try std.fmt.parseFloat(f64, parts.next() orelse return error.InvalidStl);
            const z = try std.fmt.parseFloat(f64, parts.next() orelse return error.InvalidStl);
            const v: math.Vec3 = .{ x, y, z };
            const gop = try vmap.getOrPut(v);
            if (!gop.found_existing) {
                gop.value_ptr.* = @intCast(verts.items.len);
                try verts.append(v);
            }
            tri[vcount] = gop.value_ptr.*;
            vcount += 1;
            if (vcount == 3) { try tris.append(tri); vcount = 0; }
        }
    }

    return .{
        .vertices = try verts.toOwnedSlice(),
        .triangles = try tris.toOwnedSlice(),
        .allocator = allocator,
    };
}
