# Third-party code

DiamondNav's binaries include code from these projects, fetched at build time
(CMakeLists.txt pins the exact commits):

## Recast & Detour

https://github.com/recastnavigation/recastnavigation (v1.6.0), linked into
DiamondNav, navbake and navquery.

```
Copyright (c) 2009 Mikko Mononen memon@inside.org

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
```

## open.mp

https://github.com/openmultiplayer/open.mp, its SDK
(https://github.com/openmultiplayer/open.mp-sdk) and pawn-natives
(https://github.com/openmultiplayer/pawn-natives): Mozilla Public License 2.0.
`extras/NPCs` in the releases is open.mp's NPC component built from its source with
`npcs/patches/*.patch` applied; that source is available from the open.mp
repository at the tag named in `npcs/CMakeLists.txt`.

## Pawn

The Pawn headers (amx.h) from https://github.com/openmultiplayer/compiler:
zlib-style license, Copyright (c) ITB CompuPhase.
