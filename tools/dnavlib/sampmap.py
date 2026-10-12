"""Custom SA-MP maps: the objects a script places and the stock buildings it removes,
read from the script itself (.pwn, .inc, .txt: any file with the Pawn calls) or from
an MTA map (.map XML, which most map editors export), and custom models
(AddSimpleModel in a script or in models/artconfig.txt).

Only literal numbers are read. A call whose arguments are variables or expressions
can't be placed and is counted as skipped (the build says how many).
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

# The argument each call's model is at; position and rotation follow it.
# CreatePlayerObject and RemoveBuildingForPlayer take the player first.
OBJECT_CALLS = {
    "createobject": 0,
    "createdynamicobject": 0,
    "createdynamicobjectex": 0,
    "createplayerobject": 1,
}
REMOVE_CALL = "removebuildingforplayer"

CALL_RE = re.compile(r"\b(CreateObject|CreateDynamicObjectEx|CreateDynamicObject|CreatePlayerObject|"
                     r"RemoveBuildingForPlayer|AddSimpleModelTimed|AddSimpleModel)\s*\(", re.IGNORECASE)
NUM_RE = re.compile(r"[-+]?(\d+\.?\d*|\.\d+)(e[-+]?\d+)?f?$", re.IGNORECASE)


@dataclass
class MapObject:
    model: int
    x: float
    y: float
    z: float
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0
    source: str = ""


@dataclass
class Removal:
    model: int          # -1: every model
    x: float
    y: float
    z: float
    radius: float


@dataclass
class SimpleModel:
    newid: int
    baseid: int
    dff: str            # as written: relative to the models folder
    source: str = ""


@dataclass
class MapData:
    objects: list[MapObject] = field(default_factory=list)
    removals: list[Removal] = field(default_factory=list)
    models: dict[int, SimpleModel] = field(default_factory=dict)
    skipped: int = 0

    def extend(self, other: "MapData"):
        self.objects += other.objects
        self.removals += other.removals
        self.models.update(other.models)
        self.skipped += other.skipped


def _strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


def _args(text: str, start: int):
    """The arguments of the call whose '(' is just before `start`, split at the top
    level, or None if the call never closes."""
    depth, cur, out, i, quote = 0, [], [], start, None
    while i < len(text):
        c = text[i]
        if quote:
            cur.append(c)
            if c == "\\":
                cur.append(text[i + 1] if i + 1 < len(text) else "")
                i += 1
            elif c == quote:
                quote = None
        elif c in "\"'":
            quote = c
            cur.append(c)
        elif c in "([{":
            depth += 1
            cur.append(c)
        elif c in ")]}":
            if depth == 0:
                out.append("".join(cur).strip())
                return out
            depth -= 1
            cur.append(c)
        elif c == "," and depth == 0:
            out.append("".join(cur).strip())
            cur = []
        else:
            cur.append(c)
        i += 1
    return None


def _num(s: str):
    s = s.strip()
    if not NUM_RE.match(s):
        return None
    return float(s.rstrip("fF"))


def _str(s: str):
    s = s.strip()
    return s[1:-1] if len(s) >= 2 and s[0] == s[-1] == '"' else None


STRING_RE = re.compile(r'"(?:\\.|[^"\\\n])*"')


def parse_pawn(text: str, source: str = "") -> MapData:
    out = MapData()
    text = _strip_comments(text)
    strings = [m.span() for m in STRING_RE.finditer(text)]
    for m in CALL_RE.finditer(text):
        if any(a <= m.start() < b for a, b in strings):
            continue                    # inside a string: print("CreateObject(...)")
        name = m[1].lower()
        args = _args(text, m.end())
        if args is None:
            out.skipped += 1
            continue
        if name in OBJECT_CALLS:
            at = OBJECT_CALLS[name]
            vals = [_num(a) for a in args[at:at + 7]]
            if len(vals) < 4 or any(v is None for v in vals[:4]):
                out.skipped += 1
                continue
            rot = [v if v is not None else 0.0 for v in vals[4:7]] + [0.0] * (3 - len(vals[4:7]))
            out.objects.append(MapObject(int(vals[0]), vals[1], vals[2], vals[3], *rot, source=source))
        elif name == REMOVE_CALL:
            vals = [_num(a) for a in args[1:6]]
            if len(vals) < 5 or any(v is None for v in vals):
                out.skipped += 1
                continue
            out.removals.append(Removal(int(vals[0]), vals[1], vals[2], vals[3], vals[4]))
        else:   # AddSimpleModel(virtualworld, baseid, newid, dff, txd[, on, off])
            if len(args) < 5:
                out.skipped += 1
                continue
            base, new, dff = _num(args[1]), _num(args[2]), _str(args[3])
            if base is None or new is None or dff is None:
                out.skipped += 1
                continue
            out.models[int(new)] = SimpleModel(int(new), int(base), dff, source)
    return out


TAG_RE = re.compile(r"<\s*(object|removeWorldObject)\b([^>]*)>", re.IGNORECASE)
ATTR_RE = re.compile(r'([\w:.-]+)\s*=\s*"([^"]*)"')


def parse_mta(text: str, source: str = "") -> MapData:
    """An MTA map: <object model posX posY posZ rotX rotY rotZ/> and
    <removeWorldObject model lodModel radius posX posY posZ/>. Read tag by tag, as
    editors write namespaced attributes (edf:definitions) an XML parser rejects."""
    out = MapData()
    text = re.sub(r"<!--.*?-->", " ", text, flags=re.S)
    for m in TAG_RE.finditer(text):
        tag = m[1].lower()
        a = {k.lower(): v for k, v in ATTR_RE.findall(m[2])}
        try:
            if tag == "object":
                out.objects.append(MapObject(int(float(a["model"])), float(a["posx"]), float(a["posy"]), float(a["posz"]),
                                             float(a.get("rotx", 0)), float(a.get("roty", 0)), float(a.get("rotz", 0)),
                                             source=source))
            else:
                for key in ("model", "lodmodel"):
                    if key in a and int(float(a[key])) > 0:
                        out.removals.append(Removal(int(float(a[key])), float(a["posx"]), float(a["posy"]),
                                                    float(a["posz"]), float(a["radius"])))
        except (KeyError, ValueError):
            out.skipped += 1
    return out


def read_map(path: Path) -> MapData:
    text = path.read_text(encoding="utf-8", errors="ignore")
    if path.suffix.lower() == ".map" and text.lstrip().startswith("<"):
        return parse_mta(text, path.name)
    return parse_pawn(text, path.name)


def read_maps(paths: list[Path]) -> MapData:
    """Every map file given; a folder means every .pwn, .inc, .map and .txt in it."""
    out = MapData()
    for p in paths:
        if p.is_dir():
            files = sorted(f for f in p.rglob("*") if f.suffix.lower() in (".pwn", ".inc", ".map", ".txt"))
        elif p.exists():
            files = [p]
        else:
            raise SystemExit(f"{p}: no such map file")
        for f in files:
            out.extend(read_map(f))
    return out
