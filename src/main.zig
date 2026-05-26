const std = @import("std");

const version = "0.0.0-m0";

const Parsed = struct {
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

const ParseError = error{ UnknownFlag, MissingOutputArg, TooManyArgs } ||
    std.mem.Allocator.Error ||
    std.process.Args.Iterator.InitError;

fn parseArgs(args: std.process.Args, gpa: std.mem.Allocator) ParseError!Parsed {
    var it = try std.process.Args.Iterator.initAllocator(args, gpa);
    defer it.deinit();
    _ = it.skip(); // program name

    var parsed: Parsed = .{};
    var positional: [3]?[]const u8 = .{ null, null, null };
    var pos_count: usize = 0;

    while (it.next()) |arg| {
        if (std.mem.eql(u8, arg, "-h") or std.mem.eql(u8, arg, "--help")) {
            parsed.show_help = true;
        } else if (std.mem.eql(u8, arg, "-V") or std.mem.eql(u8, arg, "--version")) {
            parsed.show_version = true;
        } else if (std.mem.eql(u8, arg, "--gpu-info")) {
            parsed.show_gpu = true;
        } else if (std.mem.eql(u8, arg, "-o") or std.mem.eql(u8, arg, "--output")) {
            const v = it.next() orelse return error.MissingOutputArg;
            parsed.output_gcode = try gpa.dupe(u8, v);
        } else if (std.mem.startsWith(u8, arg, "-")) {
            return error.UnknownFlag;
        } else if (pos_count < positional.len) {
            positional[pos_count] = try gpa.dupe(u8, arg);
            pos_count += 1;
        } else {
            return error.TooManyArgs;
        }
    }

    if (positional[0]) |p| parsed.input_stl = p;
    if (positional[1]) |p| parsed.input_gcode = p;
    return parsed;
}

fn writeStdout(io: std.Io, bytes: []const u8) !void {
    try std.Io.File.stdout().writeStreamingAll(io, bytes);
}

pub fn main(init: std.process.Init) !void {
    const gpa = init.gpa;

    const parsed = parseArgs(init.minimal.args, gpa) catch |err| {
        std.debug.print("error: {s}\n\n{s}", .{ @errorName(err), help_text });
        std.process.exit(2);
    };

    if (parsed.show_version) {
        var buf: [64]u8 = undefined;
        const msg = try std.fmt.bufPrint(&buf, "s4slicer {s}\n", .{version});
        try writeStdout(init.io, msg);
        return;
    }
    if (parsed.show_help) {
        try writeStdout(init.io, help_text);
        return;
    }
    if (parsed.show_gpu) {
        try writeStdout(init.io, "gpu: wgpu-native adapter probe — not wired yet (M3)\n");
        return;
    }

    if (parsed.input_stl == null or parsed.input_gcode == null or parsed.output_gcode == null) {
        try writeStdout(init.io, help_text);
        std.process.exit(2);
    }

    std.debug.print(
        "s4slicer {s}: pipeline not implemented yet (M0 scaffold).\n" ++
            "  stl:    {s}\n  gcode:  {s}\n  out:    {s}\n",
        .{ version, parsed.input_stl.?, parsed.input_gcode.?, parsed.output_gcode.? },
    );
    std.process.exit(1);
}

test "version constant is non-empty" {
    try std.testing.expect(version.len > 0);
}

test "help text mentions usage" {
    try std.testing.expect(std.mem.indexOf(u8, help_text, "Usage:") != null);
}
