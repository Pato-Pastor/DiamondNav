#!/usr/bin/env python3
"""Boots a real open.mp server with DiamondNav on the made-up yard and checks the
NPCs walk and drive where they should.

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

import test_tools as tt                      # noqa: E402
from dnavlib import gta, roads, world       # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("server", type=Path)
    ap.add_argument("build", type=Path)
    ap.add_argument("--npcs", type=Path)
    ap.add_argument("--keep", action="store_true", help="keep the server folder and print where it is")
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

        pawncc = srv / "qawno" / f"pawncc{exe}"
        res = subprocess.run([str(pawncc), str(HERE / "dnav_test.pwn"), f"-i{srv / 'qawno' / 'include'}",
                              f"-i{REPO / 'include'}", f"-o{srv / 'gamemodes' / 'dnav_test.amx'}", "-d1", "-O1", "-;+", "-(+"],
                             capture_output=True, text=True, cwd=srv / "qawno")
        print(res.stdout + res.stderr)
        if res.returncode:
            return 1
        (srv / "config.json").write_text(json.dumps({
            "max_players": 10, "max_bots": 10,
            "pawn": {"main_scripts": ["dnav_test 1"], "side_scripts": [], "legacy_plugins": []},
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
        return 0 if ok else 1
    finally:
        if a.keep:
            print(f"server folder kept: {tmp}")
        else:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
