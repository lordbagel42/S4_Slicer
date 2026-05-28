# S4 Slicer
A generic non-planar slicer, that can print almost any part without support.

Please use the [dicussions tab](https://github.com/jyjblrd/S4_Slicer/discussions) to ask questions and help others.

[Try it now](https://colab.research.google.com/github/jyjblrd/S4_Slicer) on Google Colab! (note: colab free tier is only powerful enough to slice very simple models)

[![Watch the video](https://github.com/jyjblrd/S4_Slicer/blob/main/thumnail.jpeg?raw=true)](https://www.youtube.com/watch?v=M51bMMVWbC8)

Check out my [YouTube video](https://youtu.be/M51bMMVWbC8?si=pfud7bHgjYDnO2_z) for more details!

Thank you to JLCCNC for helping create the extruder mount and build plate for my [4 Axis Core R-Theta Printer](https://github.com/jyjblrd/Core-R-Theta-4-Axis-Printer).

## Native port (in progress)

A Zig + WebGPU (wgpu-native) rewrite is in progress to replace the Python notebook for desktop slicing with GPU acceleration. See the milestone notes below; the notebook (`main.ipynb`) remains the reference implementation until parity is reached.

Requires **Zig 0.16+** (uses the new `std.process.Init` / `std.Io` APIs).

Build:

```
zig build            # produces zig-out/bin/s4slicer
zig build test
zig build run -- input_models/<name>.stl input_gcode/<name>.gcode -o output_gcode/<name>.gcode
```

Status: **M0 — scaffold.** CLI parses args; pipeline stages land milestone-by-milestone (see `/root/.claude/plans/right-now-this-is-spicy-sunset.md` if working with Claude Code, or the commit history).



Bibtex Citation:
```
@software{Bird_S4_Slicer,
author = {Bird, Joshua},
license = {GPL-3.0},
title = {{S4 Slicer}},
url = {https://github.com/jyjblrd/S4_Slicer}
}
```
