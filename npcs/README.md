# The improved NPC component

open.mp's own NPC component (`components/NPCs.so`, `NPCs.dll` on Windows) built from
its source at the tag in `CMakeLists.txt`, with `patches/*.patch` applied. What the
patch changes is listed in the main [README](../README.md#the-improved-npc-component);
every change is marked `DiamondNav` in the code.

## Building

```sh
# Linux
cmake -S npcs -B build/npcs -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS=-m32 -DCMAKE_CXX_FLAGS=-m32
cmake --build build/npcs                     # -> build/npcs/NPCs.so

# Windows
cmake -S npcs -B build/npcs -A Win32 -T ClangCL
cmake --build build/npcs --config Release    # -> build/npcs/Release/NPCs.dll
```

The configure step clones open.mp at the tag and applies the patches; a patch that
no longer applies fails it.

## Another open.mp release

The component has to come from the same release as the server. Change `OMP_TAG`
and the version numbers in `CMakeLists.txt`, and the three submodule commits to
what that tag pins (`git ls-tree <tag> SDK Shared/Network lib/cpp-httplib` in an
open.mp clone), build, and fix the patch if it no longer applies.

## Editing the patch

Clone open.mp at the tag, `git apply` the patch, change the code, and write
`git diff` back to `patches/0001-drive-bys-and-driving.patch`.
