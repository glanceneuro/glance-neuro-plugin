# Building from source

First, follow the instructions on
[this page](https://open-ephys.github.io/gui-docs/Developer-Guide/Compiling-the-GUI.html)
to build the Open Ephys GUI (v0.6.0+, `main` branch).

## Telling the build where `plugin-GUI` is

The build needs the GUI's headers and, on Windows, its import library. It looks
in three places, in this order:

1. **`-DGUI_BASE_DIR=/path/to/plugin-GUI`** on the `cmake` command line — wins
   over everything, and is what CI uses.
2. **the `GUI_BASE_DIR` environment variable** — set it once per shell (or in
   your profile) and every `cmake` invocation picks it up. This is usually what
   you want on a development machine, because it survives deleting and
   recreating `Build/`:

   ```bash
   export GUI_BASE_DIR=$HOME/Code/plugin-GUI     # bash/zsh
   $env:GUI_BASE_DIR = "C:\Code\plugin-GUI"     # PowerShell
   ```

3. **`../../plugin-GUI`**, the default, which assumes this layout:

   ```
   Code
   ├── plugin-GUI
   │   ├── Build
   │   ├── Source
   │   └── ...
   └── OEPlugins
       └── glance-neuro-plugin
           ├── Build
           ├── Source
           └── ...
   ```

If the GUI is anywhere else — including right beside this repo rather than one
level up — use (1) or (2). A wrong or missing path usually shows up as a
compile error about a missing JUCE header rather than as a clear message from
CMake.

> The built plugin is named after **this repository's directory**, so a checkout
> in a folder called `glance-neuro-plugin` produces `glance-neuro-plugin.so` /
> `.bundle` / `.dll`. Renaming the folder renames the artifact.

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

`install` copies `glance-neuro-plugin.so` into the GUI build's `plugins` directory
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
cp -R Debug/glance-neuro-plugin.bundle \
   "/path/to/plugin-GUI/Build/Debug/Open Ephys GUI.app/Contents/PlugIns/"
```

(swap `Debug` for `Release` as appropriate).
