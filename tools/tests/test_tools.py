#!/usr/bin/env python3
"""Tests the offline tools on a made-up San Andreas (tests/fake_gta.py): the world
from a GTA install and from a ColAndreas database, custom maps and models, the road
graph, and, when the C++ tools are built, a bake and route check with them.

    python3 tools/tests/test_tools.py [BUILD_DIR]      (BUILD_DIR has navbake, navquery)
"""

from __future__ import annotations

import math
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))

import fake_gta as fg                                   # noqa: E402
from dnavlib import col, preview, roads, sampmap, world  # noqa: E402

BUILD = Path(os.environ.get("DNAV_BUILD", HERE.parent.parent / "build"))
# A yard far from anything, 60 x 60 m.
YX, YY, YZ = -2000.0, 2000.0, 10.0


def read_tris(path: Path):
    return np.frombuffer(path.read_bytes()[8:], dtype="<f4").reshape(-1, 3, 3)


def make_install(root: Path):
    maps = root / "data" / "maps" / "yard"
    maps.mkdir(parents=True)
    (maps / "yard.ide").write_text("""objs
9001, yard_ground, yard, 300, 0
9002, yard_wall, yard, 300, 0
9003, LODyard_ground, yard, 1000, 0
9004, yard_slab, yard, 300, 0
9005, yard_pit, yard, 300, 0
9006, yard_shed, yard, 300, 0
end
""")
    qx, qy, qz, qw = fg.zquat_ipl(90)
    (maps / "yard.ipl").write_text(f"""# a made-up yard
inst
9001, yard_ground, 0, {YX}, {YY}, {YZ}, 0, 0, 0, 1, -1
9003, LODyard_ground, 0, {YX}, {YY}, {YZ}, 0, 0, 0, 1, -1
9002, yard_wall, 0, {YX}, {YY}, {YZ}, {qx}, {qy}, {qz}, {qw}, -1
9005, yard_pit, 0, {YX + 20}, {YY + 20}, {YZ}, 0, 0, 0, 1, -1
9006, yard_shed, 0, {YX - 20}, {YY - 20}, {YZ}, 0, 0, 0, 1, -1
end
""")
    # How four made-up vehicles drive, and a boat that does not (vehicles.dnv). The
    # lines carry every handling.cfg column; the sultan's DFF is in gta3.img.
    (root / "data" / "handling.cfg").write_text("""; a made-up handling.cfg
SULTAN 1400.0 3400.0 2.4 0.0 0.1 -0.1 75 0.80 0.80 0.50 5 200.0 28.0 5.0 4 P 10.0 0.50 0 30.0 1.2 0.15 0.0 0.28 -0.20 0.50 0.30 0.25 0.60 35000 2800 4000002 1 1 0
LINERUN 3800.0 19953.2 5.0 0.0 0.0 -0.2 90 0.95 0.65 0.40 5 120.0 25.0 30.0 R D 8.0 0.30 0 25.0 1.6 0.06 0.0 0.40 -0.20 0.50 0.00 0.65 0.25 35000 6000 200 0 1 2
SANDKING 2000.0 4000.0 2.5 0.0 0.0 -0.6 80 0.75 0.85 0.50 5 170.0 30.0 5.0 4 P 7.0 0.45 0 35.0 1.0 0.08 0.0 0.35 -0.30 0.50 0.30 0.44 0.35 40000 28 200000 0 1 0
NRG500 400.0 200.0 4.0 0.0 0.08 -0.09 103 1.80 0.90 0.48 5 190.0 60.0 5.0 R P 15.0 0.50 0 35.0 0.85 0.15 0.0 0.15 -0.16 0.50 0.00 0.00 0.15 10000 1002000 2 1 1 4
REEFER 5000.0 30000.0 0.8 0.0 0.0 0.0 7 2.2 15.0 0.65 5 190.0 0.2 5.0 R P 0.04 0.80 0 26.0 1.0 0.1 0.0 0.1 -0.1 0.5 0.0 0.0 0.1 10000 8000000 0 0 0 0
% REEFER 0.5 3.5 0.7 7.0 0.6 4.0 0.7 1.0 2.0 1.0
! NRG500 0.35 0.15 0.34 0.10 45.0 38.0 0.93 0.70 0.50 0.10 35.0 -40.0 -0.009 0.70 0.60
""")
    (root / "data" / "vehicles.ide").write_text("""# made-up vehicles
cars
560, sultan, sultan, car, SULTAN, SULTAN, null, richfamily, 10, 0, 0, -1, 0.7, 0.7, 2
403, linerun, linerun, car, LINERUN, LINERUN, null, worker, 10, 0, 0, -1, 1.0, 1.0, -1
495, sandking, sandking, car, SANDKING, SANDKING, null, normal, 10, 0, 0, -1, 1.0, 1.0, -1
522, nrg500, nrg500, bike, NRG500, NRG500, bikes, motorbike, 10, 0, 0, 16, 0.5, 0.5, -1
453, reefer, reefer, boat, REEFER, REEFER, null, ignore, 10, 0, 0
end
""")
    (root / "data" / "surface.dat").write_text("""; made-up adhesive limits
;\trubber\thard\troad\tloose\tsand\twet
RUBBER\t6.0
HARD\t3.6\t2.0
ROAD\t4.5\t3.0\t6.0
LOOSE\t3.2\t3.5\t2.0\t1.0
SAND\t3.0\t4.0\t2.0\t1.0\t1.0
WET\t2.8\t2.0\t1.0\t1.0\t1.0\t0.5
""")
    (root / "data" / "water.dat").write_text(
        "processed\n" + " ".join(f"{x} {y} {YZ - 1.0} 0 0 0 0" for x, y in
                                 [(YX + 15, YY + 15), (YX + 25, YY + 15), (YX + 15, YY + 25), (YX + 25, YY + 25)]) + " 1\n")
    ground = fg.col3("yard_ground", fg.box_tris((-30, -30, -1), (30, 30, 0)))
    wall = fg.col2_boxes("yard_wall", [((-10, -0.25, 0), (10, 0.25, 4))])
    slab = fg.col1("yard_slab", (-8, -8, -0.5), (8, 8, 0))
    pit = fg.col1("yard_pit", (-5, -5, -4), (5, 5, -3))
    shed = fg.col1("yard_shed", (-3, -3, 0), (3, 3, 3))
    # Roads: a two-way street west to east through the yard, two lanes each way and
    # a 4 m median, then a one-way street north from its east end, in another area.
    street = [(YX - 100, YY - 40, YZ), (YX, YY - 40, YZ), (YX + 100, YY - 40, YZ)]
    a0 = fg.nodes_dat(
        street + [(YX + 100, YY + 200, YZ, 1 << 8)],                    # an emergency-only node
        {0: [(0, 1)], 1: [(0, 0), (0, 2)], 2: [(0, 1), (1, 0), (0, 3)], 3: [(0, 2)], 4: [(0, 0)]},
        navi=[(YX - 50, YY - 40, 0, 1, 1.0, 0.0, 2, 2, 32), (YX + 50, YY - 40, 0, 2, -1.0, 0.0, 2, 2, 32)],
        navi_of={0: [(0, 0)], 1: [(0, 0), (0, 1)], 2: [(0, 1), (1, 0), (0x3F, 0x3FF)], 3: [(0x3F, 0x3FF)]},
        ped=[(YX - 25, YY + 5, YZ)])                                    # a footpath in the yard
    a1 = fg.nodes_dat(
        [(YX + 100, YY + 60, YZ)],
        {0: [(0, 2)]},
        navi=[(YX + 100, YY + 10, 0, 2, 0.0, 1.0, 1, 0, 0)],          # one lane along the direction
        navi_of={0: [(1, 0)]})
    fg.write_img(root / "models" / "gta3.img", {
        "yard.col": ground + wall,
        "yard2.col": slab + pit + shed,
        "yard_stream0.ipl": fg.binary_ipl([(YX + 100, YY, YZ, 0, 0, 0, 1, 9004)]),
        "nodes0.dat": a0,
        "nodes1.dat": a1,
        # The sultan: wheels under a chassis frame 0.1 m up, 2.6 m apart and 1.6 m
        # wide, in a 4.6 x 2.0 x 1.4 m collision box.
        "sultan.dff": fg.vehicle_dff([
            ("sultan", (0, 0, 0), -1),
            ("chassis_dummy", (0, 0, 0.1), 0),
            ("wheel_rf_dummy", (0.8, 1.35, -0.4), 1),
            ("wheel_lf_dummy", (-0.8, 1.35, -0.4), 1),
            ("wheel_rb_dummy", (0.8, -1.25, -0.4), 1),
            ("wheel_lb_dummy", (-0.8, -1.25, -0.4), 1),
        ], fg.col3("sultan", fg.box_tris((-1.0, -2.3, -0.65), (1.0, 2.3, 0.75)))),
    })


def make_maps(root: Path):
    """A custom map: removes the shed, adds a rotated stock wall, and a custom model."""
    models = root / "models"
    (models / "yard").mkdir(parents=True)
    (models / "yard" / "kiosk.dff").write_bytes(fg.dff_with_col(fg.col1("kiosk", (-1, -1, 0), (1, 1, 2.5))))
    (models / "artconfig.txt").write_text('AddSimpleModel(-1, 19379, -2001, "yard/kiosk.dff", "yard/kiosk.txd");\n')
    (root / "map.pwn").write_text(f"""
// the yard, reworked
public OnGameModeInit()
{{
    CreateObject(9002, {YX - 15}, {YY + 10}, {YZ}, 0.0, 0.0, 90.0);   // a second wall, along y
    tmp = CreateDynamicObject(-2001, {YX + 5}, {YY - 5}, {YZ}, 0.0, 0.0, 0.0, -1, -1);
    /* CreateObject(9002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); commented out */
    CreateObject(9002, x, y, z, 0.0, 0.0, 0.0);                       // variables: skipped
    return 1;
}}
public OnPlayerConnect(playerid)
{{
    RemoveBuildingForPlayer(playerid, 9006, {YX - 20}, {YY - 20}, {YZ}, 2.0);
    return 1;
}}
""")
    return models


class Rotations(unittest.TestCase):
    def test_open_mp_formula(self):
        # GTAQuat(0, 0, 90): half-angles of -45 degrees about z.
        qx, qy, qz, qw = world.stored_quat(0, 0, 90)
        self.assertAlmostEqual(qz, -math.sqrt(0.5), places=6)
        self.assertAlmostEqual(qw, math.sqrt(0.5), places=6)

    def test_object_turns_left(self):
        # rz = 90 turns an object anticlockwise seen from above: its x axis points north.
        m = world.object_matrix(0, 0, 90)
        np.testing.assert_allclose(m @ [1, 0, 0], [0, 1, 0], atol=1e-6)
        m = world.object_matrix(90, 0, 0)
        np.testing.assert_allclose(m @ [0, 1, 0], [0, 0, 1], atol=1e-6)

    def test_ipl_matches_objects(self):
        # An IPL's stored rotation and the same rotation given to CreateObject agree.
        qx, qy, qz, qw = fg.zquat_ipl(30)
        np.testing.assert_allclose(world.quat_matrix(-qx, -qy, -qz, qw), world.object_matrix(0, 0, 30), atol=1e-6)


class Maps(unittest.TestCase):
    def test_pawn(self):
        m = sampmap.parse_pawn("""
            CreateObject(1337, 1.0, 2.0, 3.0, 10.0, 20.0, 30.0, 300.0);
            CreateDynamicObject(1338, -1.5, .5, 3f, 0, 0, 0); // CreateObject(1, 0,0,0)
            CreateDynamicObjectEx(1339, 4.0, 5.0, 6.0, 0.0, 0.0, 45.0, .streamdistance = 300.0);
            CreatePlayerObject(playerid, 1340, 7.0, 8.0, 9.0, 0.0, 0.0, 0.0);
            SetDynamicObjectMaterial(CreateDynamicObject(1341, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0), 0, 1, "a", "b");
            CreateObject(GetModel(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            RemoveBuildingForPlayer(playerid, -1, 10.0, 20.0, 30.0, 50.0);
            AddSimpleModelTimed(-1, 19379, -3000, "a\\\\b.dff", "a.txd", 0, 12);
            print("CreateObject(9, 9.0, 9.0, 9.0)");
        """)
        models = [o.model for o in m.objects]
        self.assertIn(1337, models)
        self.assertEqual(sorted(models)[:5], [1337, 1338, 1339, 1340, 1341])
        o = next(o for o in m.objects if o.model == 1337)
        self.assertEqual((o.x, o.y, o.z, o.rx, o.ry, o.rz), (1.0, 2.0, 3.0, 10.0, 20.0, 30.0))
        self.assertEqual(next(o for o in m.objects if o.model == 1340).z, 9.0)
        self.assertEqual(m.removals[0].model, -1)
        self.assertEqual(m.removals[0].radius, 50.0)
        self.assertEqual(m.models[-3000].baseid, 19379)
        self.assertGreaterEqual(m.skipped, 1)
        self.assertNotIn(1, models)            # commented out

    def test_mta(self):
        m = sampmap.parse_mta("""<map edf:definitions="editor_main">
            <object id="o1" model="3095" posX="1.5" posY="2" posZ="3" rotX="0" rotY="0" rotZ="90" interior="0"/>
            <removeWorldObject id="r1" radius="10" interior="0" model="4000" lodModel="4001" posX="5" posY="6" posZ="7"/>
            <object id="broken" model="x"/>
        </map>""")
        self.assertEqual([(o.model, o.rz) for o in m.objects], [(3095, 90.0)])
        self.assertEqual(sorted(r.model for r in m.removals), [4000, 4001])
        self.assertEqual(m.skipped, 1)


class World(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        root = Path(cls.tmp.name)
        make_install(root / "gta")
        cls.models = make_maps(root / "server")
        cls.root = root
        cls.area = (YX - 200, YY - 200, YX + 200, YY + 200)
        cls.rep = world.build(root / "out" / "world", gta_dir=root / "gta", maps=[root / "server" / "map.pwn"],
                              model_dirs=[cls.models], area=cls.area)
        cls.tris = read_tris(root / "out" / "world.tris")

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_counts(self):
        r = self.rep
        # ground, wall, pit and the binary IPL's slab; the shed removed, the LOD skipped
        self.assertEqual((r.stock, r.removed, r.lods, r.no_collision), (4, 1, 1, 0))
        self.assertEqual(r.custom, 2)            # the second wall and the kiosk
        self.assertEqual(r.custom_no_collision, [])
        self.assertEqual(r.skipped_calls, 1)
        self.assertEqual(r.triangles, 12 * 6)

    def test_ipl_wall_runs_along_y(self):
        near = self.tris[(np.abs(self.tris[:, :, 0] - YX) < 1).all(axis=1) & (self.tris[:, :, 2] > YZ + 1).any(axis=1)]
        span = near.reshape(-1, 3).max(axis=0) - near.reshape(-1, 3).min(axis=0)
        self.assertAlmostEqual(span[0], 0.5, places=2)
        self.assertAlmostEqual(span[1], 20.0, places=2)

    def test_map_wall_runs_along_y(self):
        near = self.tris[(np.abs(self.tris[:, :, 0] - (YX - 15)) < 1).all(axis=1)]
        span = near.reshape(-1, 3).max(axis=0) - near.reshape(-1, 3).min(axis=0)
        self.assertAlmostEqual(span[1], 20.0, places=2)

    def test_custom_model_from_dff(self):
        k = self.tris[(np.abs(self.tris[:, :, 0] - (YX + 5)) <= 1.01).all(axis=1)
                      & (np.abs(self.tris[:, :, 1] - (YY - 5)) <= 1.01).all(axis=1)]
        self.assertAlmostEqual(float(k[:, :, 2].max()), YZ + 2.5, places=3)

    def test_removed_shed(self):
        self.assertFalse(((np.abs(self.tris[:, :, 0] - (YX - 20)) < 3.1).all(axis=1)
                          & (self.tris[:, :, 2] > YZ + 2).any(axis=1)).any())

    def test_seeds_and_water(self):
        seeds = (self.root / "out" / "world.seeds.txt").read_text().split("\n")
        self.assertEqual(self.rep.seeds, 6 + 2)  # every path node (5 vehicle + 1 ped) and each map object
        self.assertTrue(seeds[0])
        self.assertEqual(self.rep.water, 1)

    def test_colandreas_gives_the_same_world(self):
        """The same yard as a ColAndreas database: the same triangles land."""
        qx, qy, qz, qw = fg.zquat_ipl(90)
        db = fg.cadb({
            9001: ([], [(0, 0, -0.5, 30, 30, 0.5)], []),
            9002: ([], [(0, 0, 2, 10, 0.25, 2)], []),
            9004: ([], [(0, 0, -0.25, 8, 8, 0.25)], []),
            9005: ([], [(0, 0, -3.5, 5, 5, 0.5)], []),
            9006: ([], [(0, 0, 1.5, 3, 3, 1.5)], []),
            19379: ([(0, 0, 0, 1.0)], [], [((0, 0, 0), (1, 0, 0), (0, 1, 0))]),
        }, [
            (9001, YX, YY, YZ, 0, 0, 0, 1),
            (9002, YX, YY, YZ, -qx, -qy, -qz, qw),       # ColAndreas stores it ready to use
            (9005, YX + 20, YY + 20, YZ, 0, 0, 0, 1),
            (9006, YX - 20, YY - 20, YZ, 0, 0, 0, 1),
            (9004, YX + 100, YY, YZ, 0, 0, 0, 1),
        ])
        path = self.root / "colandreas.cadb"
        path.write_bytes(db)
        parsed = col.read_cadb(path)
        self.assertEqual(len(parsed.models[19379]), 12 + 1)    # the sphere as a box, and the face
        rep = world.build(self.root / "out" / "ca", cadb_path=path, maps=[self.root / "server" / "map.pwn"],
                          model_dirs=[self.models], area=self.area)
        self.assertEqual((rep.stock, rep.removed, rep.custom), (4, 1, 2))
        self.assertEqual(rep.seeds, 0)                         # no install: no path nodes
        a = self.tris.reshape(-1, 3)
        b = read_tris(self.root / "out" / "ca.tris").reshape(-1, 3)
        np.testing.assert_allclose(np.sort(a, axis=0), np.sort(b, axis=0), atol=0.01)

    def test_bake_and_walk(self):
        navbake = BUILD / ("navbake.exe" if os.name == "nt" else "navbake")
        if not navbake.exists():
            self.skipTest(f"no navbake in {BUILD}")
        out = self.root / "out"
        routes = out / "routes.txt"
        routes.write_text(f"across the yard\t{YX - 25} {YY} {YZ + 1}\t{YX + 25} {YY} {YZ + 1}\n")
        res = subprocess.run([str(navbake), str(out / "world.tris"), str(out / "world.nav"), "--routes", str(routes),
                              "--paths", str(out / "world.paths.txt"), "--seeds", str(out / "world.seeds.txt"),
                              "--water", str(out / "world.water.txt"), "--dump", str(out / "world.polys.obj")],
                             capture_output=True, text=True)
        self.assertEqual(res.returncode, 0, res.stdout + res.stderr)
        cols = (out / "world.paths.txt").read_text().split("\t")
        pts = [tuple(float(v) for v in c.split()) for c in cols[2:]]
        length = sum(math.dist(pts[i - 1][:2], pts[i][:2]) for i in range(1, len(pts)))
        self.assertGreater(length, 51.0)                        # round the wall, not through it
        zs = [float(line.split()[3]) for line in (out / "world.polys.obj").read_text().splitlines() if line.startswith("v ")]
        self.assertGreater(min(zs), YZ - 1.5)                   # nothing under the water
        try:
            import PIL  # noqa: F401
        except ImportError:
            return
        preview.draw(out / "preview.png", out / "world.polys.obj", out / "world.paths.txt")
        self.assertTrue((out / "preview.png").stat().st_size > 0)


class Roads(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.tmp.name)
        make_install(cls.root / "gta")
        from dnavlib import gta
        cls.files = gta.node_files(cls.root / "gta", [])

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_gta_paths(self):
        g = roads.from_gta(self.files)
        self.assertEqual(len(g.nodes), 5)                       # vehicle nodes only, both areas
        idx = {(round(n.x), round(n.y)): i for i, n in enumerate(g.nodes)}
        w, m, e = idx[(YX - 100, YY - 40)], idx[(YX, YY - 40)], idx[(YX + 100, YY - 40)]
        north = idx[(YX + 100, YY + 60)]
        self.assertEqual(g.links[(min(w, m), max(w, m))], (2, 2, 4.0))
        self.assertEqual(g.links[(min(m, e), max(m, e))], (2, 2, 4.0))
        # one-way north: lanes from the east end to the north node, none back
        ab, ba, _ = g.links[(min(e, north), max(e, north))]
        self.assertEqual((ab, ba) if e < north else (ba, ab), (1, 0))
        self.assertTrue(g.nodes[idx[(YX + 100, YY + 200)]].flags & roads.EMERGENCY_ONLY)
        self.assertEqual(len(g.links), 4)
        # GTA's traffic level comes along (full traffic here) for the road classes.
        self.assertTrue(all(n.flags & roads.DETAIL and not n.flags & 0x30 for n in g.nodes))

    def test_custom_roads(self):
        text = f"""
        # a side street off the middle node, and a car park loop
        node gate {YX} {YY - 100} {YZ}
        node park {YX + 30} {YY - 100} {YZ}
        road {YX},{YY - 40},{YZ} gate 1 1
        road gate park 1 0
        remove {YX + 90} {YY + 190} {YX + 110} {YY + 210}
        """
        out = self.root / "roads.dnr"
        rf = self.root / "my_roads.txt"
        rf.write_text(text)
        n, e = roads.build(out, self.files, [rf])
        self.assertEqual(n, 5 - 1 + 2)                          # the emergency node removed
        self.assertEqual(e, 2 * 5)
        drawn = preview.load_roads(out)
        self.assertEqual(len(drawn), 5)
        with self.assertRaises(SystemExit):
            roads.apply_roads_file(roads.Graph(), "road a b", "bad")
        with self.assertRaises(SystemExit):
            roads.apply_roads_file(roads.from_gta(self.files), "road 0,0,0 1,1,1", "far")
        g = roads.Graph()
        roads.apply_roads_file(g, "node a 0 0 0 main\nnode b 9 0 0 minor emergency\nroad a b", "classes")
        self.assertEqual((g.nodes[0].flags, g.nodes[1].flags), (roads.MAIN, roads.MINOR | roads.EMERGENCY_ONLY))
        with self.assertRaises(SystemExit):
            roads.apply_roads_file(roads.Graph(), "node a 0 0 0 main highway", "two classes")

    def test_route_with_navquery(self):
        navquery = BUILD / ("navquery.exe" if os.name == "nt" else "navquery")
        if not navquery.exists():
            self.skipTest(f"no navquery in {BUILD}")
        folder = self.root / "navmesh"
        roads.build(folder / "roads.dnr", self.files)
        res = subprocess.run([str(navquery), str(folder), "route", str(YX - 100), str(YY - 40), str(YZ),
                              str(YX + 100), str(YY + 60), str(YZ)], capture_output=True, text=True)
        self.assertEqual(res.returncode, 0, res.stderr)
        pts = [tuple(map(float, line.split())) for line in res.stdout.splitlines()]
        # east on the right-hand side of a road with a 4 m median: south of the centre line
        self.assertLess(pts[0][1], YY - 40 - 2.0)
        self.assertAlmostEqual(pts[-1][1], YY + 60, delta=0.5)
        # back west is refused at the one-way street
        res = subprocess.run([str(navquery), str(folder), "route", str(YX + 100), str(YY + 60), str(YZ),
                              str(YX - 100), str(YY - 40), str(YZ)], capture_output=True, text=True)
        self.assertEqual(res.returncode, 1)


class Vehicles(unittest.TestCase):
    HEAD = ("top", "brake", "brake_off", "coast", "drag", "grip", "grip_off", "radius", "yaw", "reverse",
            "length", "width", "height", "wheelbase", "ride")

    def read(self, path):
        from dnavlib import vehicles
        lines = path.read_text().splitlines()
        self.assertEqual(lines[0], "DNVH 3")
        rows = {}
        for line in lines[1:]:
            if line.startswith("#"):
                continue
            t = line.split("#")[0].split()
            nums = [float(v) for v in t[1:]]
            self.assertEqual(len(nums), len(self.HEAD) + 2 * vehicles.CURVE)
            row = dict(zip(self.HEAD, nums))
            row["accel"] = nums[len(self.HEAD):len(self.HEAD) + vehicles.CURVE]
            row["accel_off"] = nums[len(self.HEAD) + vehicles.CURVE:]
            rows[int(t[0])] = row
        return rows

    def test_from_handling(self):
        from dnavlib import vehicles
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            make_install(root / "gta")
            out = root / "vehicles.dnv"
            self.assertEqual(vehicles.build(out, root / "gta"), 4)          # not the boat
            rows = self.read(out)
            self.assertEqual(sorted(rows), [403, 495, 522, 560])
            sultan, truck, sand, bike = rows[560], rows[403], rows[495], rows[522]
            self.assertGreater(sultan["top"], truck["top"])
            self.assertGreater(sultan["accel"][0], truck["accel"][0] * 1.4)
            self.assertGreater(sultan["grip"], 10.0)
            self.assertLess(bike["radius"], sultan["radius"])
            self.assertLess(sultan["radius"], truck["radius"])
            self.assertLess(truck["yaw"], sultan["yaw"])                     # a heavy turn mass
            self.assertGreater(truck["reverse"], 0.0)
            for r in rows.values():
                self.assertGreater(r["accel"][0], 0.0)
                self.assertEqual(r["accel"][-1], 0.0)                       # nothing left at the top
                for a, o in zip(r["accel"], r["accel_off"]):
                    self.assertLessEqual(o, a + 1e-6)                       # dirt never pulls harder
                self.assertLessEqual(r["grip_off"], r["grip"] + 1e-6)
            # The sultan's size comes from its DFF: wheels 2.6 m apart under a chassis
            # frame, its collision box.
            self.assertAlmostEqual(sultan["wheelbase"], 2.6, places=2)
            self.assertAlmostEqual(sultan["length"], 4.6, places=1)
            self.assertAlmostEqual(sultan["width"], 2.0, places=1)
            # It sits on its front wheel (z -0.3 under the chassis, 0.7 m across) with its springs (0.28 up,
            # -0.20 down, force 1.2) holding its weight: about 0.76 m over the road.
            self.assertAlmostEqual(sultan["ride"], 0.02 + 0.83 * (1.0 - 1.0 / 9.6), places=2)
            self.assertEqual(truck["ride"], 0.0)                             # no DFF: left to the component
            # Dirt: tyres grip it 3.2 / 4.5 as well as a road (surface.dat); the
            # Sandking's off-road ability puts back 40% of that.
            self.assertAlmostEqual(sultan["grip_off"] / sultan["grip"], 3.2 / 4.5, places=2)
            self.assertAlmostEqual(sand["grip_off"] / sand["grip"], 3.2 * 1.4 / 4.5, places=2)
            # The bike leans no further than its bike line's 45 degrees allow.
            self.assertLessEqual(bike["grip"], vehicles.GRAVITY * vehicles.TO_MS2 * 1.0 + 0.01)
        with self.assertRaises(SystemExit):
            vehicles.build(Path(tmp) / "x.dnv", Path(tmp) / "nowhere")

    def test_game_physics(self):
        """A car with the Infernus's numbers tops out where the game's does (222 km/h
        measured in game), well short of its handling.cfg's 240, and its gears pull
        hardest in first."""
        from dnavlib import vehicles
        line = ("INFERNUS 1400.0 2725.3 1.5 0.0 0.0 -0.25 70 0.70 0.80 0.50 5 240.0 30.0 10.0 4 P 11.0 0.51 0 30.0 "
                "1.2 0.19 0.0 0.25 -0.10 0.50 0.40 0.37 0.72 95000 40002004 C04000 1 1 1")
        ide = "cars\n411, infernus, infernus, car, INFERNUS, INFERNUS, null, executive, 10, 0, 0, -1, 0.7, 0.7, -1\nend\n"
        (_model, _name, m), = vehicles.models(line, ide)
        self.assertAlmostEqual(m.top * 3.6, 222.0, delta=4.0)
        self.assertGreater(m.accel[0], m.accel[8])
        self.assertTrue(2.0 < m.zero100 < 5.0)
        # With the USE_MAXSP_LIMIT flag its gears run to handling.cfg's own top speed.
        flagged = line.replace("C04000", "1C04000")
        (_m2, _n2, f), = vehicles.models(flagged, ide)
        self.assertAlmostEqual(f.top * 3.6, 240.0, delta=2.0)


if __name__ == "__main__":
    if len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
        BUILD = Path(sys.argv.pop(1))
    unittest.main(verbosity=2)
