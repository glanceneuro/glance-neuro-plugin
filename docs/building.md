# Building from source

First, follow the instructions on
[this page](https://open-ephys.github.io/gui-docs/Developer-Guide/Compiling-the-GUI.html)
to build the Open Ephys GUI (v0.6.0+, `main` branch).

Then, clone this repository into a directory at the same level as the
`plugin-GUI`, e.g.:

```
Code
├── plugin-GUI
│   ├── Build
│   ├── Source
│   └── ...
├── OEPlugins
│   └── ephys-socket
│       ├── Build
│       ├── Source
│       └── ...
```

If your `plugin-GUI` is **not** one level up under an `OEPlugins/` sibling
(e.g. it sits right next to this repo in a flat directory), pass its location
explicitly to CMake with `-DGUI_BASE_DIR=/path/to/plugin-GUI` in any of the
commands below.

Drive the build with `cmake --build` rather than the underlying generator's
tool (`xcodebuild`, `make`, MSBuild) — keeps the command identical across
platforms.

## Windows

**Requirements:** [Visual Studio](https://visualstudio.microsoft.com/) and
[CMake](https://cmake.org/install/)

From the `Build` directory:

```bash
cmake -G "Visual Studio 17 2022" -A x64 ..
cmake --build . --config Release        # or Debug
cmake --build . --config Release --target INSTALL
```

The `INSTALL` target copies the `.dll` into the GUI's `plugins` directory.

## Linux

**Requirements:** [CMake](https://cmake.org/install/)

From the `Build` directory:

```bash
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release ..   # or Debug
cmake --build .
cmake --build . --target install
```

`install` copies `ephys-socket.so` into the GUI build's `plugins` directory
for the matching build type — build the plugin with the **same**
`CMAKE_BUILD_TYPE` as the GUI you launch (mixing Debug/Release can crash on
load).

## macOS

**Requirements:** [Xcode](https://developer.apple.com/xcode/) and
[CMake](https://cmake.org/install/)

From the `Build` directory:

```bash
cmake -G "Xcode" ..
cmake --build . --config Debug          # or Release
```

The default `install` target writes to
`~/Library/Application Support/open-ephys/plugins-api10`. To install into
the GUI app bundle instead (recommended — pairs the plugin with the matching
GUI build), copy by hand:

```bash
cp -R Debug/ephys-socket.bundle \
   "/path/to/plugin-GUI/Build/Debug/Open Ephys GUI.app/Contents/PlugIns/"
```

(swap `Debug` for `Release` as appropriate).
