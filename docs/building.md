# Building from source

**Prerequisite:** a built Open Ephys GUI (v0.6.0+, `main` branch) — follow
[Compiling the GUI](https://open-ephys.github.io/gui-docs/Developer-Guide/Compiling-the-GUI.html).
This plugin compiles against its headers, so it has to exist first.

Then three commands. Only the generator and the install step differ by platform;
substitute from the table below.

### 1. Configure — once per build directory

```bash
cmake -B Build -G <generator> -DGUI_BASE_DIR=/path/to/plugin-GUI
```

`GUI_BASE_DIR` is where you built the GUI. It can also come from an environment
variable of the same name, and defaults to `../../plugin-GUI` if you leave it
out — but passing it explicitly is the one form that always works, so start
there.

### 2. Build

```bash
cmake --build Build --config Release
```

Use `cmake --build` rather than the generator's own tool (`make`, `xcodebuild`,
MSBuild) so the command is the same everywhere.

### 3. Install

```bash
cmake --build Build --config Release --target install
```

## Per platform

| | Windows | Linux | macOS |
|---|---|---|---|
| **needs** | [Visual Studio](https://visualstudio.microsoft.com/) + [CMake](https://cmake.org/install/) | [CMake](https://cmake.org/install/) | [Xcode](https://developer.apple.com/xcode/) + [CMake](https://cmake.org/install/) |
| **`<generator>`** | `"Visual Studio 17 2022" -A x64` | `"Unix Makefiles"` | `"Xcode"` |
| **build type** | `--config Release` at step 2 | **`-DCMAKE_BUILD_TYPE=Release` at step 1** | `--config Release` at step 2 |
| **install target** | `INSTALL` (capitalized) | `install` | `install`, but see below |
| **lands in** | the GUI build's `plugins/` | the GUI build's `plugins/` | `~/Library/Application Support/open-ephys/plugins-api10` |
| **artifact** | `glance-neuro-plugin.dll` | `glance-neuro-plugin.so` | `glance-neuro-plugin.bundle` |

**Why Linux needs `-DCMAKE_BUILD_TYPE` and the others do not.** `Unix Makefiles`
is a *single-config* generator: the build type is baked into the makefiles when
you configure, and `--config` at build time is silently ignored. Visual Studio
and Xcode are *multi-config* — they emit every configuration up front and you
pick one when you build.

Leaving it off is not neutral. `CMakeLists.txt` explicitly defaults Linux to
**Debug**, which defines `DEBUG=1` / `_DEBUG=1` and leaves `NDEBUG` undefined —
the mismatch warned about below. (Optimization is not what differs: `-O3` is
forced on Linux for both build types.)

**macOS**: the default install location is the shared plugin folder, which is
usually *not* what you want while developing — it will be picked up by any GUI
build on the machine. To pair the plugin with the GUI you actually launch, copy
it into that app bundle instead:

```bash
cp -R Build/Release/glance-neuro-plugin.bundle \
   "/path/to/plugin-GUI/Build/Release/Open Ephys GUI.app/Contents/PlugIns/"
```

## Two things that bite

**Match the build type to the GUI you launch.** A Debug plugin loaded into a
Release GUI (or the reverse) can crash on load rather than fail cleanly.

**A wrong `GUI_BASE_DIR` does not fail at configure time.** CMake happily
generates against a path that does not exist; you find out at compile time, as a
missing JUCE header. If you get one of those, check the path before anything
else.

> The plugin is named after **this repository's directory**, so a checkout in a
> folder called `glance-neuro-plugin` produces `glance-neuro-plugin.*`. Renaming
> the folder renames the artifact.

Tuning for closed-loop work is separate — see [latency.md](latency.md).
