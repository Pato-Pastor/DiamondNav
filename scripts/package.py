#!/usr/bin/env python3
"""Packs a release archive from a build: what a server owner unpacks into their
server folder.

    python3 scripts/package.py BUILD_DIR NPCS_BUILD_DIR OUT_DIR linux|windows

DiamondNav-<version>-<platform>/
  components/DiamondNav.so|.dll     the component
  qawno/include/DiamondNav.inc      the Pawn include
  navmesh/                          where the baked files go (empty)
  navtools/                         navbake, navquery, dnav.py and its library
  extras/NPCs.so|.dll               open.mp's NPC component with our driving and
                                    drive-by patches (optional; open.mp v1.6.7.3218)
  README.md, LICENSE
"""

from __future__ import annotations

import re
import shutil
import sys
import tarfile
import zipfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent


def version() -> str:
    m = re.search(r"project\(DiamondNav[^)]*VERSION (\d+\.\d+\.\d+)", (REPO / "CMakeLists.txt").read_text())
    return m[1]


def main():
    build, npcs, out, plat = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]), sys.argv[4]
    lib, exe = (".dll", ".exe") if plat == "windows" else (".so", "")
    name = f"DiamondNav-{version()}-{plat}"
    root = out / name
    if root.exists():
        shutil.rmtree(root)

    def put(src: Path, rel: str):
        dst = root / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)

    def first(*cands: Path) -> Path:
        for c in cands:
            if c.exists():
                return c
        raise SystemExit(f"missing: {cands[0]}")

    put(first(build / f"DiamondNav{lib}", build / "Release" / f"DiamondNav{lib}"), f"components/DiamondNav{lib}")
    for tool in ("navbake", "navquery"):
        put(first(build / f"{tool}{exe}", build / "Release" / f"{tool}{exe}"), f"navtools/{tool}{exe}")
    put(first(npcs / f"NPCs{lib}", npcs / "Release" / f"NPCs{lib}"), f"extras/NPCs{lib}")
    put(REPO / "include" / "DiamondNav.inc", "qawno/include/DiamondNav.inc")
    put(REPO / "tools" / "dnav.py", "navtools/dnav.py")
    for p in sorted((REPO / "tools" / "dnavlib").glob("*.py")):
        put(p, f"navtools/dnavlib/{p.name}")
    put(REPO / "README.md", "README.md")
    put(REPO / "LICENSE", "LICENSE")
    put(REPO / "THIRD_PARTY.md", "THIRD_PARTY.md")
    (root / "navmesh").mkdir()
    (root / "navmesh" / "PUT_BAKED_FILES_HERE.txt").write_text(
        "world.nav and world_car.nav (navbake), roads.dnr (dnav.py roads) and vehicles.dnv\n"
        "(dnav.py vehicles) go here. See README.md.\n")

    if plat == "windows":
        archive = out / f"{name}.zip"
        with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as z:
            for p in sorted(root.rglob("*")):
                z.write(p, p.relative_to(out))
    else:
        archive = out / f"{name}.tar.gz"
        with tarfile.open(archive, "w:gz") as t:
            t.add(root, arcname=name)
    print(archive)


if __name__ == "__main__":
    main()
