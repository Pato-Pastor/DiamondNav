"""The road graph (roads.dnr) for NPC drivers: GTA's own vehicle paths, plus roads you
add for custom maps.

GTA San Andreas keeps its traffic paths in 64 files, nodes0.dat .. nodes63.dat
(inside models/gta3.img), one per 750 m square. Each vehicle node is a point on a
road's centre line; its links go to the next nodes, and each link has a "navi" node
saying how many lanes run each way and how wide the median is.

Custom roads come from a text file (see ROADS_HELP), so a custom map's streets can
be driven, joined to the stock roads or replacing them.

The .dnr layout (little-endian), read by src/roads.cpp:
  u32 magic "DNRD", u32 version, u32 node count, u32 edge count
  node: float x, y, z, u32 first edge, u16 edge count, u16 flags       (20 bytes)
  edge: u32 to, float median, u8 lanes, u8 lanes back, u16 flags       (12 bytes)
"""

from __future__ import annotations

import math
import re
import struct
from dataclasses import dataclass, field
from pathlib import Path

ROADS_MAGIC = 0x44524E44             # "DNRD"
ROADS_VERSION = 1

HIGHWAY, EMERGENCY_ONLY, BOATS = 1, 2, 4
DETAIL = 8                           # bits 4-5 hold GTA's traffic level
TRAFFIC_SHIFT = 4                    # 0 full .. 3 low
MINOR, MAIN = 0x40, 0x80             # a custom road's class, when its node says
NODE_WORDS = {"highway": HIGHWAY, "main": MAIN, "minor": MINOR, "emergency": EMERGENCY_ONLY, "boats": BOATS}
SNAP = 15.0                          # how far "x,y,z" in a roads file may be from a node

ROADS_HELP = """\
A roads file, one statement per line ('#' starts a comment):

  node NAME X Y Z [highway|main|minor] [emergency] [boats]
  road A B [LANES [LANES_BACK [MEDIAN]]]
  remove X0 Y0 X1 Y1

node    a point on a road's centre line, Z on the road surface
road    joins A and B. A and B are node names, or "X,Y,Z" for the nearest
        existing node (stock or yours) within 15 m. LANES run from A to B,
        LANES_BACK from B to A (0 makes it one-way); 1 and 1 when left out.
        MEDIAN is the gap in metres between the two directions (default 0).
remove  drops every stock road node inside the box (and its links), for a map
        that builds over a road. Applied before your nodes are added.

A road's class sets its speed limit (Nav_SetRoadSpeed): highway between two
highway nodes, main with two or more lanes either way (or between two main
nodes), minor if either node is minor, a service road if either is emergency,
otherwise a street.
"""


@dataclass
class Node:
    x: float
    y: float
    z: float
    flags: int = 0


@dataclass
class Graph:
    nodes: list[Node] = field(default_factory=list)
    # (a, b) -> (lanes a->b, lanes b->a, median); stored once per pair, a < b
    links: dict[tuple[int, int], tuple[int, int, float]] = field(default_factory=dict)

    def add_node(self, n: Node) -> int:
        self.nodes.append(n)
        return len(self.nodes) - 1

    def link(self, a: int, b: int, ab: int, ba: int, median: float = 0.0):
        if a == b:
            return
        if a > b:
            a, b, ab, ba = b, a, ba, ab
        self.links.setdefault((a, b), (ab, ba, median))

    def nearest(self, x, y, z, reach=SNAP) -> int:
        best, bd = -1, reach * reach
        for i, n in enumerate(self.nodes):
            d = (n.x - x) ** 2 + (n.y - y) ** 2 + (n.z - z) ** 2
            if d < bd:
                best, bd = i, d
        return best

    def keep(self, want):
        """Only the nodes where want(index, node) holds, and the links between them."""
        remap, nodes = {}, []
        for i, n in enumerate(self.nodes):
            if want(i, n):
                remap[i] = len(nodes)
                nodes.append(n)
        self.links = {(remap[a], remap[b]): v for (a, b), v in self.links.items() if a in remap and b in remap}
        self.nodes = nodes

    def drop_unlinked(self):
        used = {i for pair in self.links for i in pair}
        self.keep(lambda i, n: i in used)


# ---------------------------------------------------------------------------
# GTA's nodes*.dat
# ---------------------------------------------------------------------------

NODE = struct.Struct("<2I3hhHHHBBI")         # 28 bytes
NAVI = struct.Struct("<hhHHbbI")             # 14 bytes


@dataclass
class Area:
    vehicle: int
    nodes: list            # (x, y, z, link start, link count, flags)
    links: list            # (area, node)
    navi: list             # (dirx, diry, flags)
    navi_links: list       # (area, navi id)


def parse_area(d: bytes) -> Area:
    n, nveh, _nped, nnavi, nlinks = struct.unpack_from("<5I", d, 0)
    at = 20
    nodes = []
    for k in range(n):
        _, _, x, y, z, _, link, _, _, _, _, flags = NODE.unpack_from(d, at + k * 28)
        nodes.append((x / 8.0, y / 8.0, z / 8.0, link, flags & 0xF, flags))
    at += n * 28
    navi = []
    for k in range(nnavi):
        _, _, _, _, dx, dy, flags = NAVI.unpack_from(d, at + k * 14)
        navi.append((dx / 100.0, dy / 100.0, flags))
    at += nnavi * 14
    links = [struct.unpack_from("<HH", d, at + k * 4) for k in range(nlinks)]
    at += nlinks * 4 + 768
    navi_links = []
    for k in range(nlinks):
        v = struct.unpack_from("<H", d, at + k * 2)[0] if at + k * 2 + 2 <= len(d) else 0xFFFF
        navi_links.append((v >> 10, v & 0x3FF))
    return Area(nveh, nodes, links, navi, navi_links)


def node_flags(f: int) -> int:
    out = DETAIL | (((f >> 4) & 3) << TRAFFIC_SHIFT)
    if f & (1 << 13):
        out |= HIGHWAY
    if f & (1 << 8):
        out |= EMERGENCY_ONLY
    if f & (1 << 7):
        out |= BOATS
    return out


def from_gta(files: dict[str, bytes]) -> Graph:
    """The vehicle path network of every nodes<N>.dat given (name -> bytes)."""
    areas = {}
    for name, d in files.items():
        areas[int(re.search(r"\d+", name)[0])] = parse_area(d)
    g = Graph()
    index = {}
    for a in sorted(areas):
        for k in range(areas[a].vehicle):
            x, y, z, _, _, f = areas[a].nodes[k]
            index[(a, k)] = g.add_node(Node(x, y, z, node_flags(f)))
    for a in sorted(areas):
        ar = areas[a]
        for k in range(ar.vehicle):
            x, y, z, start, count, _ = ar.nodes[k]
            me = index[(a, k)]
            for li in range(start, start + count):
                if li >= len(ar.links):
                    break
                other = index.get(tuple(ar.links[li]))
                if other is None:
                    continue            # a ped node, or an area that was not given
                ab = ba = 1
                median = 0.0
                na, nid = ar.navi_links[li]
                navi = areas[na].navi[nid] if na in areas and nid < len(areas[na].navi) else None
                if navi is not None:
                    dx, dy, f = navi
                    left, right = (f >> 8) & 7, (f >> 11) & 7
                    if left or right:
                        o = g.nodes[other]
                        # The "left" lanes run along the navi node's direction.
                        # Checked on the real map: this way round, 92% of split
                        # carriageways have the opposite one on the driver's left,
                        # as they must where traffic keeps right.
                        if dx * (o.x - x) + dy * (o.y - y) >= 0:
                            ab, ba = left, right
                        else:
                            ab, ba = right, left
                    median = (f & 0xFF) / 8.0
                g.link(me, other, ab, ba, median)
    return g


# ---------------------------------------------------------------------------
# custom roads
# ---------------------------------------------------------------------------

def apply_roads_file(g: Graph, text: str, source: str = "roads"):
    names: dict[str, int] = {}
    statements = []
    for no, raw in enumerate(text.splitlines(), 1):
        t = raw.split("#")[0].split()
        if t:
            statements.append((no, t))

    def fail(no, msg):
        raise SystemExit(f"{source}:{no}: {msg}")

    # Removals first, so a map can rebuild the roads it removes.
    for no, t in statements:
        if t[0].lower() == "remove":
            if len(t) != 5:
                fail(no, "remove takes X0 Y0 X1 Y1")
            x0, y0, x1, y1 = map(float, t[1:])
            x0, x1, y0, y1 = min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1)
            g.keep(lambda i, n: not (x0 <= n.x <= x1 and y0 <= n.y <= y1))

    def ref(no, s):
        if s in names:
            return names[s]
        parts = s.split(",")
        if len(parts) == 3:
            try:
                x, y, z = map(float, parts)
            except ValueError:
                fail(no, f"bad point {s!r}")
            i = g.nearest(x, y, z)
            if i < 0:
                fail(no, f"no road node within {SNAP:.0f} m of {s}")
            return i
        fail(no, f"unknown node {s!r} (declare it with 'node' before the road)")

    for no, t in statements:
        kw = t[0].lower()
        if kw == "remove":
            continue
        if kw == "node":
            if len(t) < 5:
                fail(no, "node takes NAME X Y Z [highway|main|minor] [emergency] [boats]")
            flags = 0
            for word in t[5:]:
                w = word.lower()
                if w not in NODE_WORDS:
                    fail(no, f"unknown node flag {word!r}")
                flags |= NODE_WORDS[w]
            if bin(flags & (HIGHWAY | MAIN | MINOR)).count("1") > 1:
                fail(no, "a node is one of highway, main or minor")
            if t[1] in names:
                fail(no, f"node {t[1]!r} declared twice")
            try:
                names[t[1]] = g.add_node(Node(float(t[2]), float(t[3]), float(t[4]), flags))
            except ValueError:
                fail(no, "node coordinates must be numbers")
        elif kw == "road":
            if not 3 <= len(t) <= 6:
                fail(no, "road takes A B [LANES [LANES_BACK [MEDIAN]]]")
            a, b = ref(no, t[1]), ref(no, t[2])
            try:
                ab = int(t[3]) if len(t) > 3 else 1
                ba = int(t[4]) if len(t) > 4 else 1
                median = float(t[5]) if len(t) > 5 else 0.0
            except ValueError:
                fail(no, "lanes are whole numbers and the median a number")
            if not (0 <= ab <= 7 and 0 <= ba <= 7) or ab + ba == 0:
                fail(no, "lanes are 0 to 7 each way, and at least one way has a lane")
            # A road statement replaces a link that is already there.
            key = (a, b) if a < b else (b, a)
            g.links.pop(key, None)
            g.link(a, b, ab, ba, median)
        else:
            fail(no, f"unknown statement {t[0]!r}")


# ---------------------------------------------------------------------------
# output
# ---------------------------------------------------------------------------

def write_dnr(g: Graph, path: Path) -> tuple[int, int]:
    per = [[] for _ in g.nodes]
    for (a, b), (ab, ba, median) in g.links.items():
        per[a].append((b, median, ab, ba))
        per[b].append((a, median, ba, ab))
    node_bytes, edge_bytes, count = bytearray(), bytearray(), 0
    for n, edges in zip(g.nodes, per):
        node_bytes += struct.pack("<3fIHH", n.x, n.y, n.z, count, len(edges), n.flags)
        for to, median, lanes, back in edges:
            edge_bytes += struct.pack("<IfBBH", to, median, lanes, back, 0)
        count += len(edges)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(struct.pack("<4I", ROADS_MAGIC, ROADS_VERSION, len(g.nodes), count) + node_bytes + edge_bytes)
    return len(g.nodes), count


def build(out: Path, node_files: dict[str, bytes], roads_files: list[Path] = (), area=None,
          boats=True) -> tuple[int, int]:
    g = from_gta(node_files) if node_files else Graph()
    if area is not None:
        g.keep(lambda i, n: area[0] <= n.x <= area[2] and area[1] <= n.y <= area[3])
    if not boats:
        g.keep(lambda i, n: not n.flags & BOATS)
    for p in roads_files:
        apply_roads_file(g, p.read_text(encoding="utf-8"), p.name)
    g.drop_unlinked()
    if not g.nodes:
        raise SystemExit("no roads: give a GTA folder (--gta), nodes*.dat files (--nodes) or a roads file (--roads)")
    return write_dnr(g, out)


def length_km(g: Graph) -> float:
    return sum(math.dist((g.nodes[a].x, g.nodes[a].y), (g.nodes[b].x, g.nodes[b].y)) for a, b in g.links) / 1000.0
