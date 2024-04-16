# JoltC
C wrapper for [Jolt Physics](https://github.com/jrouwe/JoltPhysics).

## Building
Use CMake:

```bash
cmake -B build
cmake --build build
```

## Other C Wrapper
Other C wrappers for Jolt Physics include:
- "JoltC", part of the [zphysics] Zig library started by [Michal Ziulek][michal-ziulek]
- "JoltC", part of the [jolt-rs] Rust library started by [cohaereo] and a fork of the zphysics C wrapper
- "joltc", part of the [JoltPhysicsSharp] C# library started by [Amer Koleci][amerkoleci]

The goal of this project is to be the first C wrapper around Jolt Physics that is not part of a larger binding project and to eliminate sources of unsoundness. It's intended to be useful for any other language-specific bindings and to reduce the need to duplicate work.

## License
See [LICENSE](LICENSE) for more details.

[zphysics]: https://github.com/zig-gamedev/zig-gamedev/tree/main/libs/zphysics
[jolt-rs]: https://github.com/cohaereo/jolt-rs
[JoltPhysicsSharp]: https://github.com/amerkoleci/JoltPhysicsSharp
[michal-ziulek]: https://github.com/michal-z
[amerkoleci]: https://github.com/amerkoleci
[cohaereo]: https://github.com/cohaereo