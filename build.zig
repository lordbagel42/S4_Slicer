const std = @import("std");

pub fn build(b: *std.Build) void {
    const target   = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    // ── TetGen C++ bridge ──────────────────────────────────────────────────
    const tetgen_bridge = b.addObject(.{
        .name = "tetgen_bridge",
        .root_module = b.createModule(.{
            .target = target,
            .optimize = optimize,
        }),
    });
    tetgen_bridge.root_module.addCSourceFiles(.{
        .files = &.{
            "src/tetgen_bridge.cpp",
            "third_party/tetgen.cxx",
            "third_party/predicates.cxx",
        },
        .flags = &.{ "-DTETLIBRARY", "-w", "-O2", "-fno-sanitize=undefined" },
    });
    tetgen_bridge.root_module.addIncludePath(b.path("src"));
    tetgen_bridge.root_module.addIncludePath(b.path("third_party"));
    tetgen_bridge.root_module.link_libcpp = true;

    // ── Leaf modules (no deps) ─────────────────────────────────────────────
    const math_mod = b.createModule(.{
        .root_source_file = b.path("src/math.zig"),
        .target = target,
        .optimize = optimize,
    });
    const sparse_mod = b.createModule(.{
        .root_source_file = b.path("src/sparse.zig"),
        .target = target,
        .optimize = optimize,
    });

    // ── lsmr depends on sparse ─────────────────────────────────────────────
    const lsmr_mod = b.createModule(.{
        .root_source_file = b.path("src/lsmr.zig"),
        .target = target,
        .optimize = optimize,
    });
    lsmr_mod.addImport("sparse", sparse_mod);

    // ── stl depends on math ────────────────────────────────────────────────
    const stl_mod = b.createModule(.{
        .root_source_file = b.path("src/stl.zig"),
        .target = target,
        .optimize = optimize,
    });
    stl_mod.addImport("math", math_mod);

    // ── mesh depends on math ───────────────────────────────────────────────
    const mesh_mod = b.createModule(.{
        .root_source_file = b.path("src/mesh.zig"),
        .target = target,
        .optimize = optimize,
    });
    mesh_mod.addImport("math", math_mod);

    // ── gcode depends on math ──────────────────────────────────────────────
    const gcode_mod = b.createModule(.{
        .root_source_file = b.path("src/gcode.zig"),
        .target = target,
        .optimize = optimize,
    });
    gcode_mod.addImport("math", math_mod);

    // ── spatial depends on math + mesh ────────────────────────────────────
    const spatial_mod = b.createModule(.{
        .root_source_file = b.path("src/spatial.zig"),
        .target = target,
        .optimize = optimize,
    });
    spatial_mod.addImport("math", math_mod);
    spatial_mod.addImport("mesh", mesh_mod);

    // ── rotation depends on math + mesh + sparse + lsmr ───────────────────
    const rotation_mod = b.createModule(.{
        .root_source_file = b.path("src/rotation.zig"),
        .target = target,
        .optimize = optimize,
    });
    rotation_mod.addImport("math", math_mod);
    rotation_mod.addImport("mesh", mesh_mod);
    rotation_mod.addImport("sparse", sparse_mod);
    rotation_mod.addImport("lsmr", lsmr_mod);

    // ── deformation depends on math + mesh + sparse + lsmr + rotation ─────
    const deformation_mod = b.createModule(.{
        .root_source_file = b.path("src/deformation.zig"),
        .target = target,
        .optimize = optimize,
    });
    deformation_mod.addImport("math", math_mod);
    deformation_mod.addImport("mesh", mesh_mod);
    deformation_mod.addImport("sparse", sparse_mod);
    deformation_mod.addImport("lsmr", lsmr_mod);
    deformation_mod.addImport("rotation", rotation_mod);

    // ── Root module: main.zig ──────────────────────────────────────────────
    const exe_mod = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });
    exe_mod.addImport("math",        math_mod);
    exe_mod.addImport("stl",         stl_mod);
    exe_mod.addImport("mesh",        mesh_mod);
    exe_mod.addImport("rotation",    rotation_mod);
    exe_mod.addImport("deformation", deformation_mod);
    exe_mod.addImport("gcode",       gcode_mod);
    exe_mod.addImport("spatial",     spatial_mod);
    exe_mod.addImport("sparse",      sparse_mod);
    exe_mod.addImport("lsmr",        lsmr_mod);
    exe_mod.addObject(tetgen_bridge);
    exe_mod.link_libcpp = true;

    const exe = b.addExecutable(.{
        .name = "s4slicer",
        .root_module = exe_mod,
    });
    b.installArtifact(exe);

    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());
    if (b.args) |args| run_cmd.addArgs(args);
    b.step("run", "Run s4slicer").dependOn(&run_cmd.step);

    const tests = b.addTest(.{ .root_module = exe_mod });
    b.step("test", "Run tests").dependOn(&b.addRunArtifact(tests).step);
}
