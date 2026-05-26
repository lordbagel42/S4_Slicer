const std = @import("std");

const version = "0.0.0-m0";

const Args = struct {
    input_stl: ?[]const u8 = null,
    input_gcode: ?[]const u8 = null,
    output_gcode: ?[]const u8 = null,
    show_help: bool = false,
    show_version: bool = false,
    show_gpu: bool = false,
};

const help_text =
    \\s4slicer — non-planar slicer for 4-axis printers
    \\
    \\Usage:
    \\  s4slicer <input.stl> <input.gcode> -o <output.gcode> [options]
    \\
    \\Options:
    \\  -o, --output <path>   Output G-code path
    \\  --gpu-info            Print available GPU adapter and exit
    \\  -h, --help            Show this help
    \\  -V, --version         Show version
    \\
;

fn parseArgs(allocator: std.mem.Allocator) !Args {
    var args = Args{};
    var it = try std.process.argsWithAllocator(allocator);
    defer it.deinit();
    _ = it.next();

    var positional: [3]?[]const u8 = .{ null, null, null };
    var pos_count: usize = 0;

    while (it.next()) |arg| {
        if (std.mem.eql(u8, arg, "-h") or std.mem.eql(u8, arg, "--help")) {
            args.show_help = true;
        } else if (std.mem.eql(u8, arg, "-V") or std.mem.eql(u8, arg, "--version")) {
            args.show_version = true;
        } else if (std.mem.eql(u8, arg, "--gpu-info")) {
            args.show_gpu = true;
        } else if (std.mem.eql(u8, arg, "-o") or std.mem.eql(u8, arg, "--output")) {
            const v = it.next() orelse return error.MissingOutputArg;
            args.output_gcode = try allocator.dupe(u8, v);
        } else if (std.mem.startsWith(u8, arg, "-")) {
            std.debug.print("unknown flag: {s}\n", .{arg});
            return error.UnknownFlag;
        } else if (pos_count < positional.len) {
            positional[pos_count] = try allocator.dupe(u8, arg);
            pos_count += 1;
        } else {
            return error.TooManyArgs;
        }
    }

    if (positional[0]) |p| args.input_stl = p;
    if (positional[1]) |p| args.input_gcode = p;
    return args;
}

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    const args = parseArgs(allocator) catch |err| {
        std.debug.print("error: {s}\n\n{s}", .{ @errorName(err), help_text });
        std.process.exit(2);
    };

    const stdout = std.io.getStdOut().writer();

    if (args.show_version) {
        try stdout.print("s4slicer {s}\n", .{version});
        return;
    }
    if (args.show_help) {
        try stdout.writeAll(help_text);
        return;
    }
    if (args.show_gpu) {
        try stdout.writeAll("gpu: wgpu-native adapter probe — not wired yet (M3)\n");
        return;
    }

    if (args.input_stl == null or args.input_gcode == null or args.output_gcode == null) {
        try stdout.writeAll(help_text);
        std.process.exit(2);
    }

    std.debug.print(
        "s4slicer {s}: pipeline not implemented yet (M0 scaffold).\n" ++
            "  stl:    {s}\n  gcode:  {s}\n  out:    {s}\n",
        .{ version, args.input_stl.?, args.input_gcode.?, args.output_gcode.? },
    );
    std.process.exit(1);
}

test "version constant is non-empty" {
    try std.testing.expect(version.len > 0);
}
