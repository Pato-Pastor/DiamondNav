#!/usr/bin/env python3
"""Boots a real open.mp server with DiamondNav on the made-up yard and checks the
NPCs walk and drive where they should (dnav_test.pwn), then again on roads alone
for the chases a server with no car navmesh drives (dnav_chase.pwn).

    python3 tests/server/run.py SERVER_DIR BUILD_DIR [--npcs NPCs.so]

SERVER_DIR is an unpacked open.mp server release (omp-server, components/,
qawno/); it is copied, not changed. BUILD_DIR has DiamondNav.so and navbake.
--npcs swaps in another NPC component (ours, from npcs/) for the run.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "tools" / "tests"))

import fake_gta as fg                       # noqa: E402
import test_tools as tt                      # noqa: E402
from dnavlib import gta, roads, vehicles, world  # noqa: E402


def chase_roads() -> bytes:
    """dnav_chase.pwn's roads, one lane each way: a bridge 15 m up and the road 30 m
    north of it below, joined only by a ramp past the bridge's east end; and four long
    straight roads to the south."""
    x, y, z = tt.YX, tt.YY, 10.0
    nodes = [(x - 150, y, z + 15), (x, y, z + 15), (x + 150, y, z + 15),       # the bridge
             (x + 220, y + 15, z + 7.5),                                       # the ramp
             (x + 150, y + 30, z), (x, y + 30, z), (x - 150, y + 30, z),       # the road below
             (x - 320, y - 200, z), (x + 200, y - 200, z), (x + 750, y - 200, z),
             (x - 120, y - 400, z), (x + 120, y - 400, z),
             (x - 150, y - 600, z), (x + 150, y - 600, z),
             (x - 150, y - 800, z), (x + 150, y - 800, z)]
    pairs = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (7, 8), (8, 9), (10, 11), (12, 13), (14, 15)]
    links = {}
    for a, b in pairs:
        links.setdefault(a, []).append((0, b))
        links.setdefault(b, []).append((0, a))
    return fg.nodes_dat(nodes, links)


def serve(srv: Path, script: str, exe: str, patched: bool) -> bool:
    """Compiles tests/server/<script>.pwn, boots the server on it and reads its result."""
    pawncc = srv / "qawno" / f"pawncc{exe}"
    res = subprocess.run([str(pawncc), str(HERE / f"{script}.pwn"), f"-i{srv / 'qawno' / 'include'}",
                          f"-i{REPO / 'include'}", f"-o{srv / 'gamemodes' / f'{script}.amx'}", "-d1", "-O1", "-;+", "-(+"]
                         + (["PATCHED_NPCS=1"] if patched else []),
                         capture_output=True, text=True, cwd=srv / "qawno")
    print(res.stdout + res.stderr)
    if res.returncode:
        return False
    (srv / "config.json").write_text(json.dumps({
        "max_players": 10, "max_bots": 10,
        "pawn": {"main_scripts": [f"{script} 1"], "side_scripts": [], "legacy_plugins": []},
        "network": {"port": 7797, "bind": "127.0.0.1"},
        "logging": {"enable": True, "use_prefix": True, "use_timestamp": False},
        "rcon": {"enable": False, "password": "unused-test-only"},
    }, indent=1))
    try:
        run = subprocess.run([str(srv / f"omp-server{exe}")], cwd=srv, capture_output=True, text=True, timeout=150)
        out = run.stdout + run.stderr
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or b"").decode(errors="replace") if isinstance(e.stdout, bytes) else (e.stdout or "")
        out += "\n[dnav-test] FAIL: the server never exited"
    lines = [line for line in out.splitlines() if "[dnav-test]" in line or "[nav]" in line or "rror" in line]
    print("\n".join(lines))
    ok = any(line.rstrip().endswith("[dnav-test] PASS") for line in out.splitlines())
    if not ok:
        print(out[-4000:])
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("server", type=Path)
    ap.add_argument("build", type=Path)
    ap.add_argument("--npcs", type=Path)
    ap.add_argument("--keep", action="store_true", help="keep the server folder and print where it is")
    ap.add_argument("--only", choices=("yard", "chase"), help="run one of the two")
    a = ap.parse_args()
    exe = ".exe" if sys.platform == "win32" else ""
    lib = ".dll" if sys.platform == "win32" else ".so"

    tmp = Path(tempfile.mkdtemp(prefix="dnav-server-"))
    try:
        srv = tmp / "server"
        shutil.copytree(a.server, srv, ignore=shutil.ignore_patterns("gamemodes", "filterscripts", "logs", "log.txt",
                                                                     "config.json", "plugins", "scriptfiles"))
        for d in ("gamemodes", "filterscripts", "scriptfiles", "navmesh"):
            (srv / d).mkdir(exist_ok=True)
        shutil.copy(a.build / f"DiamondNav{lib}", srv / "components" / f"DiamondNav{lib}")
        if a.npcs:
            shutil.copy(a.npcs, srv / "components" / f"NPCs{lib}")
        # Only the stock components the test needs, plus ours: no legacy plugins.
        keep = {"Pawn", "NPCs", "Vehicles", "Timers", "Console", "Classes", "DiamondNav", "LegacyConfig", "Variables"}
        for p in (srv / "components").iterdir():
            if p.suffix == lib and p.stem not in keep:
                p.unlink()

        yard = tmp / "yard"
        tt.make_install(yard / "gta")
        world.build(yard / "world", gta_dir=yard / "gta", area=(tt.YX - 200, tt.YY - 200, tt.YX + 200, tt.YY + 200))
        res = subprocess.run([str(a.build / f"navbake{exe}"), str(yard / "world.tris"), str(srv / "navmesh" / "world.nav"),
                              "--seeds", str(yard / "world.seeds.txt"), "--water", str(yard / "world.water.txt")],
                             capture_output=True, text=True)
        if res.returncode:
            print(res.stdout + res.stderr)
            return 1
        # The same yard for a car: where the chase may leave the road.
        res = subprocess.run([str(a.build / f"navbake{exe}"), str(yard / "world.tris"), str(srv / "navmesh" / "world_car.nav"),
                              "--agent", "car", "--water", str(yard / "world.water.txt")],
                             capture_output=True, text=True)
        if res.returncode:
            print(res.stdout + res.stderr)
            return 1
        roads.build(srv / "navmesh" / "roads.dnr", gta.node_files(yard / "gta", []))
        vehicles.build(srv / "navmesh" / "vehicles.dnv", yard / "gta")

        ok = a.only == "chase" or serve(srv, "dnav_test", exe, a.npcs is not None)
        if a.only == "yard":
            return 0 if ok else 1

        # Chases on roads alone, as a server with no car navmesh has them.
        for f in (srv / "navmesh").iterdir():
            f.unlink()
        roads.build(srv / "navmesh" / "roads.dnr", {"nodes0.dat": chase_roads()})
        vehicles.build(srv / "navmesh" / "vehicles.dnv", yard / "gta")
        ok = serve(srv, "dnav_chase", exe, a.npcs is not None) and ok
        return 0 if ok else 1
    finally:
        if a.keep:
            print(f"server folder kept: {tmp}")
        else:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
