"""Build the four-way magnetic coupling and domed shell variant of GyroRoll R1B."""

import argparse
from contextlib import chdir
import json
import math
from pathlib import Path
import struct
import sys

from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_Transform,
)
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism, BRepPrimAPI_MakeSphere
from OCP.StlAPI import StlAPI_Writer
from OCP.gp import gp_Ax1, gp_Ax2, gp_Dir, gp_Elips, gp_Pnt, gp_Trsf, gp_Vec

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent
OUTPUT = HERE.parents[1] / "cad" / "magnetic"
STEP_NAME = "GyroRoll_R1B_magnetic_dome.step"

DOCK_Z = 295
FACE_OFFSET = 160
MAGNET_RADIUS = 16
MAGNET_DEPTH = 18
MAGNET_MASS = 62
PIVOT_X = -128
PIVOT_LIMIT = 30
ROOF_EQUATOR = 400
ROOF_RADIUS = 77
CORNER_RADIUS = 12
WALL = 2.5
CLEAR_STORAGE = (170, 116, 79)
FRONT_OPENING = (313, 398)
DOOR_HALF_WIDTH = 63
SKIN = "#507d91"
ARM = "#426176"
MAGNET_CASE = "#4a555e"
POLE_FACE = "#9dabb2"
INSULATOR = "#cc874f"
DOCKS = [
    ("左", 0, -77, -90),
    ("右", 0, 77, 90),
    ("前", 90, 0, 0),
    ("后", -90, 0, 180),
]


def place(shape, x=0, y=0, z=0, yaw=0, pivot=(0, 0, 0)):
    transform = gp_Trsf()
    if yaw:
        transform.SetRotation(gp_Ax1(gp_Pnt(*pivot), gp_Dir(0, 0, 1)), math.radians(yaw))
    transform.SetTranslationPart(gp_Vec(
        transform.TranslationPart().X() + x,
        transform.TranslationPart().Y() + y,
        transform.TranslationPart().Z() + z,
    ))
    return BRepBuilderAPI_Transform(shape, transform, False).Shape()


def rounded_outline(a, b, rx, ry, z):
    """Counterclockwise wire with corresponding straight and elliptical edges at every height."""
    wire = BRepBuilderAPI_MakeWire()
    for index, (cx, cy) in enumerate(((a-rx, b-ry), (-a+rx, b-ry),
                                       (-a+rx, -b+ry), (a-rx, -b+ry))):
        angle = index * math.pi / 2
        ellipse = gp_Elips(gp_Ax2(gp_Pnt(cx, cy, z), gp_Dir(0, 0, 1)), rx, ry)
        arc = BRepBuilderAPI_MakeEdge(ellipse, angle, angle + math.pi / 2)
        wire.Add(arc.Edge())
        end = gp_Pnt(cx + rx * math.cos(angle + math.pi / 2),
                     cy + ry * math.sin(angle + math.pi / 2), z)
        nx, ny = ((-a+rx, b-ry), (-a+rx, -b+ry), (a-rx, -b+ry), (a-rx, b-ry))[index]
        start = gp_Pnt(nx + rx * math.cos(angle + math.pi / 2),
                       ny + ry * math.sin(angle + math.pi / 2), z)
        wire.Add(BRepBuilderAPI_MakeEdge(end, start).Edge())
    return wire.Wire()


def dome_volume(base, inset, bottom):
    a, b = 90-inset, 77-inset
    corner = CORNER_RADIUS-inset
    radius = ROOF_RADIUS-inset
    ceiling = ROOF_EQUATOR-inset
    outline = rounded_outline(a, b, corner, corner, bottom)
    cabin = BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(outline).Face(),
                                  gp_Vec(0, 0, ceiling-bottom)).Shape()
    hemisphere = BRepPrimAPI_MakeSphere(
        gp_Pnt(0, 0, ROOF_EQUATOR), radius, 0, math.pi/2,
    ).Shape()
    if not inset:
        return base.fuse(cabin, hemisphere)

    # The void opens through the flat ledge into the dome.
    opening = base.cylinder((0, 0, ceiling), base.Z, radius, inset)
    return base.fuse(cabin, opening, hemisphere)


def build_cabin(base):
    box, cyl, cut, fuse = base.box, base.cylinder, base.cut, base.fuse
    x_axis, z_axis = base.X, base.Z
    lower, upper = FRONT_OPENING
    floor = base.cut(
        BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(
            rounded_outline(90, 77, CORNER_RADIUS, CORNER_RADIUS, base.CABIN_BOTTOM)).Face(),
            gp_Vec(0, 0, base.CABIN_FLOOR-base.CABIN_BOTTOM)).Shape(),
        *(cyl((x, y, base.CABIN_BOTTOM), z_axis, 2.75, 4) for x in (-56, 56) for y in (-40, 40)),
        *(cyl((x, y, base.CABIN_BOTTOM), z_axis, 2.25, 4) for x in (-82, 82) for y in (-70, 70)),
    )
    base.add("body", "圆角储物底板", floor, SKIN)
    outer = dome_volume(base, 0, base.CABIN_FLOOR)
    inner = dome_volume(base, WALL, base.CABIN_FLOOR-1)
    inside = cut(outer, inner)
    hatch_cut = box(86.9, -DOOR_HALF_WIDTH, lower,
                    4, 2*DOOR_HALF_WIDTH, upper-lower)
    fastener_z = (325, 384)
    bosses = [
        cut(box(82.8, 59.5 if side == 1 else -66, z-4, 4.7, 6.5, 8),
            cyl((82.8, side*60.4, z), x_axis, 1.25, 4.7))
        for side in (-1, 1) for z in fastener_z
    ]
    mounting_pads = [
        cut(cyl((x, y, 312), z_axis, 4.2, 9), cyl((x, y, 312), z_axis, 2.25, 9))
        for x in (-82, 82) for y in (-70, 70)
    ]
    shell = cut(inside, hatch_cut)
    base.add("roof", "拱顶与圆角储物舱外壁", fuse(shell, *bosses, *mounting_pads), SKIN)
    grip_z = 372
    grip = fuse(box(87.5, -10, grip_z-4, 2.5, 20, 8),
                *(cyl((87.5, y, grip_z), x_axis, 4, 2.5) for y in (-10, 10)))
    hatch = cut(
        box(87.75, -DOOR_HALF_WIDTH+.3, lower+.3, 2.25,
            2*DOOR_HALF_WIDTH-.6, upper-lower-.6),
        grip,
        *(cyl((87.5, y, z), x_axis, 1.7, 2.5)
          for y in (-60.4, 60.4) for z in fastener_z),
    )
    base.add("door", "齐平储物舱门 · 内嵌指槽", hatch, "#58869a")

    for x in (-82, 82):
        for y in (-70, 70):
            base.add("frame", "M4×20 贯通螺杆", cyl((x, y, 301), z_axis, 2, 20), "#8393a1")


def dock_fasteners(base, x0, length):
    return [base.cylinder((x0, y, z), base.X, 2.15, length)
            for y in (-13, 13) for z in (293.7, 304.9)]


def install_dock_anchors(base):
    box, cut, fuse = base.box, base.cut, base.fuse
    anchors = [place(box(-5, -20, 285.8, 5, 40, 16.2), x, y, yaw=yaw)
               for _, x, y, yaw in DOCKS]
    holes = [place(hole, x, y, yaw=yaw)
             for _, x, y, yaw in DOCKS for hole in dock_fasteners(base, -6.1, 7)]
    panel_openings = [place(box(-2.6, -20.25, 285.55, 3, 40.5, 16.45), x, y, yaw=yaw)
                      for _, x, y, yaw in DOCKS]
    new_parts = []
    for group, name, shape, color in base.PARTS:
        if name.endswith("连接口遮盖"):
            continue
        if name.startswith("180×154 齐平上沿承力框"):
            shape = cut(fuse(shape, *anchors), *holes)
        if group == "cover":
            shape = cut(shape, *panel_openings)
        new_parts.append((group, name, shape, color))
    base.PARTS[:] = new_parts


def add_magnet(base, transform, side, front_station, group):
    cylinder, ring = base.cylinder, base.ring
    axis = base.X
    x = front_station - MAGNET_DEPTH
    housing = cylinder((x, 0, DOCK_Z), axis, MAGNET_RADIUS, MAGNET_DEPTH-1.4)
    face_x = front_station-1.4
    shell = ring((face_x, 0, DOCK_Z), axis, MAGNET_RADIUS, 11.9, 1.4)
    center = cylinder((face_x, 0, DOCK_Z), axis, 9.5, 1.4)
    isolation = ring((face_x, 0, DOCK_Z), axis, 11.9, 9.5, 1.10)
    for detail, solid, color in [
        ("电磁铁本体", housing, MAGNET_CASE),
        ("电磁铁磁极面", base.compound((shell, center)), POLE_FACE),
        ("电磁铁隔磁环", isolation, INSULATOR),
    ]:
        base.add(group, side+detail, transform(solid), color)


def build_docks(base):
    box, cut, cyl, fuse, ring = base.box, base.cut, base.cylinder, base.fuse, base.ring
    build_face = cut(box(0, -20.5, 284, 7, 41, 25), *dock_fasteners(base, 0, 7))
    for side, x, y, yaw in DOCKS:
        def transform(solid):
            return place(solid, x, y, yaw=yaw)

        face_station = FACE_OFFSET - max(abs(x), abs(y))
        magnet_mount = face_station-MAGNET_DEPTH
        arm_width = 17
        if side != "后":
            stem = box(5, -arm_width/2, DOCK_Z-9, magnet_mount-10, arm_width, 18)
            boss = cyl((magnet_mount-5, 0, DOCK_Z), base.X, 13.5, 5)
            base.add("dock", side+"电磁连接承力支臂", transform(fuse(build_face, stem, boss)), ARM)
            add_magnet(base, transform, side, face_station, "dock")
        else:
            pivot = abs(PIVOT_X)-abs(x)
            stem = box(5, -arm_width/2, DOCK_Z-11, pivot-17, arm_width, 22)
            ears = []
            for z in (DOCK_Z-11, DOCK_Z+6):
                lug = fuse(box(pivot-13, -9, z, 13, 18, 5),
                           cyl((pivot, 0, z), base.Z, 9, 5))
                ears.append(cut(lug, cyl((pivot, 0, z), base.Z, 3.2, 5)))
            bracket = fuse(build_face, stem, *ears)
            base.add("dock", "后电磁连接承力支臂 · 固定叉耳", transform(bracket), ARM)
            swivel_z = DOCK_Z-5.65
            head = fuse(cyl((pivot, 0, swivel_z), base.Z, 8.5, 11.3),
                        box(pivot, -6, swivel_z, magnet_mount-pivot, 12, 11.3),
                        cyl((magnet_mount-3, 0, DOCK_Z), base.X, 12, 3))
            head = cut(head, cyl((pivot, 0, swivel_z), base.Z, 3.2, 11.3))
            base.add("rearHead", "后磁头摆臂 · 可绕竖轴转动", transform(head), ARM)
            add_magnet(base, transform, side, face_station, "rearHead")
            pin = fuse(cyl((pivot, 0, DOCK_Z-11.8), base.Z, 3, 23.6),
                       cyl((pivot, 0, DOCK_Z+11), base.Z, 4.7, 1.4))
            base.add("dock", "后磁头 Ø6 竖直钢销", transform(pin), "#c0a375")
            base.add("dock", "后磁头轴端防脱环",
                     transform(ring((pivot, 0, DOCK_Z-12.6), base.Z, 4.4, 3, 1.35)), "#8a9aa1")
        for across in (-13, 13):
            for z in (293.7, 304.9):
                bolt = fuse(cyl((-4.8, across, z), base.X, 2, 11.8),
                            cyl((7, across, z), base.X, 3.4, 2.5))
                base.add("dock", side+"连接座 M4 紧固件", transform(bolt), "#8296a2")


def matches_print(name):
    return ("承力支臂" in name or "磁头摆臂" in name or
            "储物舱外壁" in name or "圆角储物底板" in name or "储物舱门" in name)


def mass_with_docks(base, mass_module):
    estimate_items = [
        ("frame" if "承力支臂" in name or "磁头摆臂" in name else group, name, shape, color)
        for group, name, shape, color in base.PARTS
    ]
    data = mass_module.estimate(estimate_items)
    magnet_budget = {
        "电磁铁本体": MAGNET_MASS-8, "电磁铁磁极面": 7.5, "电磁铁隔磁环": .5,
    }
    for (group, name, shape, _), row in zip(base.PARTS, data["rows"]):
        row["group"] = group
        if "电磁铁" in name:
            row["mass_g"] = next(weight for part, weight in magnet_budget.items() if name.endswith(part))
            row["basis"] = "Ø32×18 通用电磁铁占位，单件合计 62 g 预算"
        elif name.endswith(("紧固件", "竖直钢销", "轴端防脱环")):
            row["mass_g"] = base.volume(shape) * mass_module.STEEL
            row["basis"] = "钢件按实体体积估算"
    data.update(mass_module.total(data["rows"]))
    return data


def volume_overlap(base, first, second, b1, b2):
    if any(min(b1[k+3], b2[k+3])-max(b1[k], b2[k]) < .01 for k in range(3)):
        return 0
    common = BRepAlgoAPI_Common(first, second)
    if not common.IsDone():
        raise ValueError("Intersection check failed")
    return base.volume(common.Shape())


def check_mating_pair(base, records, transform):
    """Report solid overlaps across the contact plane between two assembled robots."""
    fixed = [(name, shape, base.shape_bounds(shape)) for _, name, shape, _ in records]
    other = []
    for _, name, shape, _ in base.PARTS:
        moved = transform(shape)
        other.append((name, moved, base.shape_bounds(moved)))
    overlaps = []
    for n1, a, aabb in fixed:
        for n2, b, bbb in other:
            if (v := volume_overlap(base, a, b, aabb, bbb)) > .15:
                overlaps.append((n1, n2, round(v, 4)))
    return overlaps


def validate(base, original):
    collisions = base.check_assembly(base.PARTS)
    if collisions:
        raise ValueError(f"Static assembly overlaps: {collisions}")
    print("Static interference check passed.", flush=True)
    motion = {}
    fixed = [entry for entry in base.PARTS if entry[0] != "rearHead"]
    moving = [entry for entry in base.PARTS if entry[0] == "rearHead"]
    fixed_bounds = [(name, shape, base.shape_bounds(shape)) for _, name, shape, _ in fixed]
    for angle in range(-PIVOT_LIMIT, PIVOT_LIMIT+1, 5):
        rotated = [(name, place(shape, yaw=angle, pivot=(PIVOT_X, 0, 0)))
                   for _, name, shape, _ in moving]
        overlaps = []
        for n1, a in rotated:
            aabb = base.shape_bounds(a)
            for n2, b, bbb in fixed_bounds:
                if (v := volume_overlap(base, a, b, aabb, bbb)) > .15:
                    overlaps.append((n1, n2, round(v, 4)))
        if overlaps:
            raise ValueError(f"Rear yaw {angle} degree overlap: {overlaps}")
        motion[str(angle)] = overlaps

    print("Rear magnet sweep check passed.", flush=True)
    pitch = 2*FACE_OFFSET
    pairs = {"side_by_side": check_mating_pair(base, base.PARTS, lambda s: place(s, y=pitch))}
    pivot = (PIVOT_X, 0, 0)
    for angle in (-PIVOT_LIMIT, -15, 0, 15, PIVOT_LIMIT):
        articulated = [*fixed, *(("rearHead", name, place(shape, yaw=angle, pivot=pivot), color)
                                 for _, name, shape, color in moving)]
        theta = math.radians(angle)
        dx, dy = -pitch*math.cos(theta), -pitch*math.sin(theta)
        pairs[f"front_to_rear_{angle:+}"] = check_mating_pair(
            base, articulated, lambda s: place(s, dx, dy, yaw=angle, pivot=pivot))
    if any(pairs.values()):
        raise ValueError(f"Mated robot overlaps: {pairs}")

    clear_x, clear_y, clear_z = CLEAR_STORAGE
    front_path = base.box(-clear_x/2, -clear_y/2, FRONT_OPENING[0]+1,
                          clear_x/2+FACE_OFFSET+6, clear_y, clear_z)
    path_bounds = base.shape_bounds(front_path)
    obstructed = []
    for group, name, shape, _ in base.PARTS:
        if group != "door" and (v := volume_overlap(
                base, shape, front_path, base.shape_bounds(shape), path_bounds)) > .15:
            obstructed.append((name, round(v, 4)))
    if obstructed:
        raise ValueError(f"Storage approach overlaps: {obstructed}")

    rotor_bounds = base.shape_bounds(base.compound(shape for group, _, shape, _ in original
                                                 if group in {"fly", "flyDrive"}))
    overhang = min(FACE_OFFSET + rotor_bounds[1], FACE_OFFSET - rotor_bounds[4])
    if overhang < 15:
        raise ValueError("Magnetic mounting faces have insufficient lateral overhang")
    return {"static_interferences": collisions, "rear_yaw_deg": motion,
            "rotating_module_lateral_extent_mm": [rotor_bounds[1], rotor_bounds[4]],
            "lateral_face_overhang_mm": overhang,
            "front_storage_approach_interferences": obstructed,
            "mated_robot_interferences": pairs,
            "straight_coupled_axle_pitch_mm": pitch,
            "base_ratio": base.flywheel_dimensions()["ratio"]}


def strip_collapsed_facets(path):
    # OCCT can emit a facet with two identical vertices at an analytic sphere pole.
    stl = path.read_bytes()
    facets = []
    for (record,) in struct.iter_unpack("<50s", stl[84:]):
        vertices = struct.unpack_from("<9f", record, 12)
        if len({vertices[:3], vertices[3:6], vertices[6:]}) == 3:
            facets.append(record)
    if len(facets) != struct.unpack_from("<I", stl, 80)[0]:
        path.write_bytes(stl[:80] + struct.pack("<I", len(facets)) + b"".join(facets))


def export_print_parts(base, output):
    directory = output / "print_fit_check"
    directory.mkdir(exist_ok=True)
    rows = ["文件\t零件\t尺寸 mm"]
    for index, (_, name, shape, _) in enumerate(base.PARTS, 1):
        if not matches_print(name):
            continue
        if "储物舱门" in name:
            rotation = gp_Trsf()
            rotation.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(0, 1, 0)), -math.pi/2)
            shape = BRepBuilderAPI_Transform(shape, rotation, False).Shape()
        xmin, ymin, zmin, xmax, ymax, zmax = base.shape_bounds(shape)
        local = place(shape, -xmin, -ymin, -zmin)
        BRepMesh_IncrementalMesh(local, .12)
        stl = StlAPI_Writer()
        stl.ASCIIMode = False
        file = f"part_{index:03d}.stl"
        # OCCT's STL writer needs an ASCII filename on Windows, even in a Unicode project directory.
        with chdir(directory):
            written = stl.Write(local, file)
        if not written:
            raise ValueError(f"STL write failed: {name}")
        if "拱顶" in name:
            strip_collapsed_facets(directory/file)
        size = " × ".join(f"{v:.2f}" for v in (xmax-xmin, ymax-ymin, zmax-zmin))
        rows.append(f"{file}\t{name}\t{size}")
    (directory/"parts.tsv").write_text("\n".join(rows)+"\n", encoding="utf-8-sig")


def main(source, output):
    # Preserve the source assembly. The variant reuses the original mechanical subassemblies.
    source = source.resolve()
    sys.path.insert(0, str(source))
    import build as base
    import mass as mass_module

    if not (source/"DD9015_supplier.step").is_file():
        raise FileNotFoundError("Download the DD9015 STEP from SteadyWin. See the repository README.")
    output.mkdir(parents=True, exist_ok=True)
    base.configure_flywheel_transmission(20, 40, 273, 8)
    base.PARTS.clear()
    base.GROUPS.update({
        "roof": ("圆角储物机身与拱顶", SKIN, (0, 0, 100)),
        "dock": ("四向电磁连接件", ARM, (0, 55, 10)),
        "rearHead": ("后侧转动磁头", ARM, (-52, 0, 10)),
    })
    base.make_wheel()
    base.make_drive()
    base.make_frame()
    base.make_flywheels(base.stock_flywheel_parts())
    base.make_electronics()
    original = list(base.PARTS)
    base.make_cargo_body()
    reference = mass_module.estimate(base.PARTS)
    reference.pop("rows")
    base.PARTS[:] = original
    base.GROUPS.pop("deck")
    install_dock_anchors(base)
    build_cabin(base)
    build_docks(base)

    for _, name, shape, _ in base.PARTS:
        if not BRepCheck_Analyzer(shape).IsValid() or base.volume(shape) <= 0:
            raise ValueError(f"Invalid CAD solid: {name}")
    print("All solids valid. Checking assembled clearances.", flush=True)
    checks = validate(base, original)
    print("Docking and storage checks passed. Exporting CAD and meshes.", flush=True)
    assembly = base.compound(entry[2] for entry in base.PARTS)
    base.write_step(output/STEP_NAME, "GyroRoll R1B magnetic couplers and dome", assembly, base.PARTS)
    mass = mass_with_docks(base, mass_module)
    (output/"mass_properties.json").write_text(json.dumps(
        {"units": {"mass": "g", "position": "mm"}, "before": reference, "after": mass},
        ensure_ascii=False, indent=2), encoding="utf-8")
    (output/"clearance.json").write_text(json.dumps(checks, ensure_ascii=False, indent=2), encoding="utf-8")

    data = {
        "groups": {gid: {"title": name, "color": color, "explode": vector}
                   for gid, (name, color, vector) in base.GROUPS.items()},
        "parts": [dict(group=g, name=n, color=c, **base.mesh(s)) for g, n, s, c in base.PARTS],
        "bounds": base.shape_bounds(assembly),
        "wheelCenterDistance": round(base.WHEEL_C, 2),
        "wheelBeltLength": base.WHEEL_BELT_LENGTH,
        "beltWidth": base.BELT_WIDTH,
        "groundZ": base.WHEEL_Z-base.WHEEL_DIAMETER/2,
        "flywheel": base.flywheel_dimensions(),
        "baselineMass": reference,
        "mass": {key: value for key, value in mass.items() if key != "rows"},
        "battery": next(row for row in mass["rows"] if row["part"].startswith("Tattu")),
        "cargo": {"clear": CLEAR_STORAGE, "floor": base.CABIN_FLOOR, "roofRadius": ROOF_RADIUS,
                  "roofEquator": ROOF_EQUATOR},
        "dock": {"pivot": [PIVOT_X, 0, DOCK_Z], "yawLimit": PIVOT_LIMIT,
                 "faceOffset": FACE_OFFSET, "overhang": checks["lateral_face_overhang_mm"],
                 "diameter": 2*MAGNET_RADIUS, "depth": MAGNET_DEPTH},
    }
    template = (HERE/"viewer.html.in").read_text(encoding="utf-8")
    lib = source/"viewer_lib"
    three = (lib/"three.min.js").read_text(encoding="utf-8")
    license_text = (lib/"three-LICENSE.txt").read_text(encoding="utf-8")
    html = template.replace("/* THREE_BUNDLE */", "/*\n"+license_text+"\n*/\n"+three)
    html = html.replace("/* CAD_DATA */", json.dumps(data, ensure_ascii=False, separators=(",", ":")))
    (output/"index.html").write_text(html, encoding="utf-8")
    export_print_parts(base, output)
    print(json.dumps({"parts": len(base.PARTS), "mass_g": mass["mass_g"], "cog_mm": mass["center_mm"],
                      "bounds_mm": data["bounds"], "checks": checks}, ensure_ascii=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", type=Path, default=SOURCE)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    main(args.base, args.output.resolve())
