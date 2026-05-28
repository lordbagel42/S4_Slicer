const std = @import("std");
const m = @import("math.zig");

pub const Command = enum { G00, G01 };

pub const GcodePoint = struct {
    position: m.Vec3,
    command: Command,
    extrusion: ?f64,
    inv_time_feed: ?f64,
    feed: f64,
    move_length: f64,
    after_retract: bool,
};

pub const OutputPoint = struct {
    position: m.Vec3,
    rotation: f64,
    command: Command,
    extrusion: ?f64,
    inv_time_feed: ?f64,
    extrusion_multiplier: f64,
    feed: f64,
    travelling: bool,
};

const SEG_SIZE: f64 = 0.6;

pub fn parse(path: []const u8, io: std.Io, allocator: std.mem.Allocator) ![]GcodePoint {
    var file = try std.Io.Dir.cwd().openFile(io, path, .{});
    defer file.close(io);

    var read_buf: [65536]u8 = undefined;
    var r = file.readerStreaming(io, &read_buf);
    const raw = try r.interface.allocRemaining(allocator, .unlimited);
    defer allocator.free(raw);

    var points = std.array_list.Managed(GcodePoint).init(allocator);

    var pos = m.Vec3{ 0, 0, 20 };
    var feed: f64 = 5000;

    var line_it = std.mem.splitScalar(u8, raw, '\n');
    while (line_it.next()) |raw_line| {
        const line_no_comment = if (std.mem.indexOfScalar(u8, raw_line, ';')) |ci| raw_line[0..ci] else raw_line;
        const stripped = std.mem.trim(u8, line_no_comment, " \r\t");
        if (stripped.len == 0) continue;

        var it = std.mem.tokenizeScalar(u8, stripped, ' ');
        var cmd: ?Command = null;
        var nx = pos[0]; var ny = pos[1]; var nz = pos[2];
        var extrusion: ?f64 = null;
        var new_feed: ?f64 = null;

        while (it.next()) |word| {
            if (word.len < 2) continue;
            const letter = word[0];
            const val_str = word[1..];
            switch (letter) {
                'G', 'g' => {
                    const gn = std.fmt.parseInt(u32, val_str, 10) catch continue;
                    if (gn == 0) cmd = .G00;
                    if (gn == 1) cmd = .G01;
                },
                'X', 'x' => nx = std.fmt.parseFloat(f64, val_str) catch continue,
                'Y', 'y' => ny = std.fmt.parseFloat(f64, val_str) catch continue,
                'Z', 'z' => nz = std.fmt.parseFloat(f64, val_str) catch continue,
                'F', 'f' => new_feed = std.fmt.parseFloat(f64, val_str) catch continue,
                'E', 'e' => extrusion = std.fmt.parseFloat(f64, val_str) catch continue,
                else => {},
            }
        }

        if (new_feed) |f| feed = f;
        if (cmd == null) continue;

        const new_pos = m.Vec3{ nx, ny, nz };
        const prev_pos = pos;
        pos = new_pos;

        const delta = m.sub3(new_pos, prev_pos);
        const dist = m.norm3(delta);

        if (dist > 0) {
            const num_segs: usize = @max(1, @as(usize, @intFromFloat(@ceil(dist / SEG_SIZE))));
            const seg_dist = dist / @as(f64, @floatFromInt(num_segs));
            const inv_time: ?f64 = if (feed > 0) feed / seg_dist else null;

            for (0..num_segs) |si| {
                const t = @as(f64, @floatFromInt(si + 1)) / @as(f64, @floatFromInt(num_segs));
                try points.append(.{
                    .position = m.add3(prev_pos, m.scale3(delta, t)),
                    .command = cmd.?,
                    .extrusion = if (extrusion) |e| e / @as(f64, @floatFromInt(num_segs)) else null,
                    .inv_time_feed = inv_time,
                    .feed = feed,
                    .move_length = seg_dist,
                    .after_retract = false,
                });
            }
        } else {
            try points.append(.{
                .position = new_pos,
                .command = cmd.?,
                .extrusion = extrusion,
                .inv_time_feed = null,
                .feed = feed,
                .move_length = 0,
                .after_retract = false,
            });
        }
    }

    return points.toOwnedSlice();
}

pub const WriteParams = struct {
    nozzle_offset: f64 = 42.0,
    retraction_length: f64 = 1.0,
    rotation_avg_alpha: f64 = 0.2,
    rotation_max_delta_deg: f64 = 1.0,
    max_extrusion_multiplier: f64 = 10.0,
    min_rotation_deg: f64 = -130.0,
    max_rotation_deg: f64 = 30.0,
};

pub fn write_polar(
    path: []const u8,
    points: []const OutputPoint,
    params: WriteParams,
    io: std.Io,
    allocator: std.mem.Allocator,
) !void {
    _ = allocator;
    var file = try std.Io.Dir.cwd().createFile(io, path, .{ .truncate = true });
    defer file.close(io);

    var write_buf: [65536]u8 = undefined;
    var w = file.writerStreaming(io, &write_buf);

    try w.interface.writeAll("G94 ; mm/min feed\n");
    try w.interface.writeAll("G28 ; home\n");
    try w.interface.writeAll("M83 ; relative extrusion\n");
    try w.interface.writeAll("G1 E10 ; prime extruder\n");
    try w.interface.writeAll("G94 ; mm/min feed\n");
    try w.interface.writeAll("G90 ; absolute positioning\n");
    try w.interface.writeAll("G0 C0 X0 Z20 B0 ; go to start\n");
    try w.interface.writeAll("G93 ; inverse time feed\n");

    var theta_accum: f64 = 0;
    var prev_theta: f64 = 0;
    var is_travelling = false;

    for (points) |pt| {
        const pos = pt.position;
        const rotation = std.math.clamp(
            pt.rotation,
            std.math.degreesToRadians(params.min_rotation_deg),
            std.math.degreesToRadians(params.max_rotation_deg),
        );

        if (std.math.isNan(pos[0]) or pos[2] < 0) continue;

        const z_hop: f64 = if (is_travelling) 1.0 else 0.0;
        const r = @sqrt(pos[0] * pos[0] + pos[1] * pos[1]);
        const theta = std.math.atan2(pos[1], pos[0]);
        const z = pos[2];
        const r_adj = r - @sin(rotation) * (params.nozzle_offset + z_hop);
        const z_adj = z + (@cos(rotation) - 1.0) * (params.nozzle_offset + z_hop) + z_hop;

        var delta_theta = theta - prev_theta;
        if (delta_theta > std.math.pi) delta_theta -= 2.0 * std.math.pi;
        if (delta_theta < -std.math.pi) delta_theta += 2.0 * std.math.pi;
        theta_accum += delta_theta;

        const cmd_str: []const u8 = if (pt.command == .G00) "G00" else "G01";
        try w.interface.print("{s} C{d:.5} X{d:.5} Z{d:.5} B{d:.5}", .{
            cmd_str,
            std.math.radiansToDegrees(theta_accum),
            r_adj, z_adj,
            std.math.radiansToDegrees(rotation),
        });

        if (pt.extrusion) |e| {
            const mult = std.math.clamp(pt.extrusion_multiplier, 0, params.max_extrusion_multiplier);
            try w.interface.print(" E{d:.4}", .{e * mult});
        }

        if (pt.inv_time_feed) |f| {
            try w.interface.print(" F{d:.4}\n", .{f});
        } else {
            try w.interface.writeAll(" F20000\nG94\nG93\n");
        }

        prev_theta = theta;
        if (pt.extrusion) |e| {
            if (e == -params.retraction_length) is_travelling = true;
            if (e == params.retraction_length) is_travelling = false;
        }
    }
    try w.flush();
}
