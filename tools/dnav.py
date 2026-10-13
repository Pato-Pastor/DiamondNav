#!/usr/bin/env python3
"""DiamondNav's offline tools.

    dnav.py world   build the world's collision for navbake (stock map + your maps)
    dnav.py roads   build roads.dnr, the road graph NPC drivers follow
    dnav.py preview draw a baked navmesh, its checked routes and the roads as a PNG

Run `dnav.py <command> --help` for each. A typical bake of the whole map:

    python3 tools/dnav.py world --gta "C:/Games/GTA San Andreas" --map gamemodes/maps out/world
    navbake out/world.tris navmesh/world.nav --seeds out/world.seeds.txt --water out/world.water.txt
    navbake out/world.tris navmesh/world_car.nav --agent car --water out/world.water.txt
    python3 tools/dnav.py roads --gta "C:/Games/GTA San Andreas" navmesh/roads.dnr

and the server loads everything in its navmesh/ folder at start.
Needs Python 3.9+; world also needs numpy, preview Pillow. roads needs nothing else.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from dnavlib import gta, roads  # noqa: E402  (world and preview need numpy: imported when used)


def area_arg(v):
    try:
        x0, y0, x1, y1 = (float(k) for k in v.replace(",", " ").split())
    except ValueError:
        raise argparse.ArgumentTypeError('an area is "X0 Y0 X1 Y1"')
    return (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))


def cmd_world(a):
    from dnavlib import world
    rep = world.build(a.out, gta_dir=a.gta, cadb_path=a.colandreas, maps=a.map or [], model_dirs=a.models or [],
                      area=a.area, water_dat=a.water)
    for line in rep.lines():
        print(line)
    print(f"wrote {a.out.with_suffix('.tris')}")
    nb = [f"navbake {a.out.with_suffix('.tris')} navmesh/world.nav"]
    if rep.seeds:
        nb.append(f"--seeds {a.out.with_suffix('.seeds.txt')}")
    if rep.water:
        nb.append(f"--water {a.out.with_suffix('.water.txt')}")
    print("next: " + " ".join(nb))
    car = [f"navbake {a.out.with_suffix('.tris')} navmesh/world_car.nav --agent car"]
    if rep.water:
        car.append(f"--water {a.out.with_suffix('.water.txt')}")
    print("and, for cars off the road (Nav_Chase): " + " ".join(car))
    return 0


def cmd_roads(a):
    if a.help_format:
        print(roads.ROADS_HELP)
        return 0
    if a.out is None:
        raise SystemExit("give the output file (navmesh/roads.dnr)")
    files = gta.node_files(a.gta, a.nodes or [])
    n, e = roads.build(a.out, files, a.roads or [], area=a.area, boats=not a.no_boats)
    print(f"wrote {a.out}: {n} road nodes, {e // 2} links (from {len(files)} GTA path files, "
          f"{len(a.roads or [])} roads files)")
    return 0


def cmd_preview(a):
    from dnavlib import preview
    p = a.prefix
    preview.draw(a.out, p.with_suffix(".polys.obj") if p else None, p.with_suffix(".paths.txt") if p else None,
                 a.roads, a.scale)
    print(f"wrote {a.out}")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog="dnav.py", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    w = sub.add_parser("world", help="build the world's collision for navbake",
                       description="Build the world's collision for navbake (dnavlib/world.py says more).",
                       formatter_class=argparse.RawDescriptionHelpFormatter)
    w.add_argument("out", type=Path, help="output prefix: writes OUT.tris, OUT.seeds.txt, OUT.water.txt")
    w.add_argument("--gta", type=Path, help="a GTA San Andreas folder (the one players use is best)")
    w.add_argument("--colandreas", type=Path, help="ColAndreas's colandreas.cadb (instead of, or with, --gta)")
    w.add_argument("--map", type=Path, action="append",
                   help="a map file or folder (.pwn/.inc/.txt with CreateObject, CreateDynamicObject, "
                        "RemoveBuildingForPlayer, AddSimpleModel; or an MTA .map). Repeat for more")
    w.add_argument("--models", type=Path, action="append",
                   help="a folder with custom models (DFFs with collision, .col files, artconfig.txt), "
                        "usually the server's models/. Repeat for more")
    w.add_argument("--area", type=area_arg, help='only this box, "X0 Y0 X1 Y1" (a test, or a quicker bake)')
    w.add_argument("--water", type=Path, help="a water.dat to use (without --gta there is no water otherwise)")
    w.set_defaults(fn=cmd_world)

    r = sub.add_parser("roads", help="build the road graph (roads.dnr)", description=roads.__doc__,
                       formatter_class=argparse.RawDescriptionHelpFormatter)
    r.add_argument("out", type=Path, nargs="?", help="output file, normally navmesh/roads.dnr")
    r.add_argument("--gta", type=Path, help="a GTA San Andreas folder: its path files (nodes*.dat in gta3.img)")
    r.add_argument("--nodes", type=Path, action="append", help="loose nodes*.dat files or a folder of them")
    r.add_argument("--roads", type=Path, action="append", help="a roads file of your own (--help-format)")
    r.add_argument("--area", type=area_arg, help='only the stock roads in this box, "X0 Y0 X1 Y1"')
    r.add_argument("--no-boats", action="store_true", help="leave out the water routes")
    r.add_argument("--help-format", action="store_true", help="how a roads file is written")
    r.set_defaults(fn=cmd_roads)

    p = sub.add_parser("preview", help="draw a navmesh, its routes and the roads as a PNG")
    p.add_argument("out", type=Path, help="the PNG to write")
    p.add_argument("--bake", dest="prefix", type=Path,
                   help="navbake output prefix: reads PREFIX.polys.obj (--dump) and PREFIX.paths.txt (--paths)")
    p.add_argument("--roads", type=Path, help="a roads.dnr")
    p.add_argument("--scale", type=float, help="pixels per metre (default: fit in 4096 px, at most 6)")
    p.set_defaults(fn=cmd_preview)

    a = ap.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    raise SystemExit(main())
