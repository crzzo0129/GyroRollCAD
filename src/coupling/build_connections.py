
"""Build the R1B coupling prototype and a four-robot assembly in millimetres."""
import argparse
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
OUTPUT = HERE.parent.parent / "cad" / "coupling"
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--base", type=Path, default=HERE.parent)
args = parser.parse_args()
base_path = args.base.resolve()
sys.path.insert(0, str(base_path))

import build as base  # The original R1B builder supplies the robot geometry.
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec

base.HERE = base_path
from build import X, Y, Z, box, compound, cylinder, cut, fuse, ring

HALF_TRACK = 170
HALF_ROW_SPACING = 225
BEAM_HEIGHT = 342
SEAT_TOP = 360
RACK_BOTTOM = 447
FOLD_LIMIT = 36
BLUE, BLUE_DARK = "#208bb7", "#196084"
ORANGE, ORANGE_LIGHT = "#dc853a", "#f6c36c"
PURPLE, TEAL, PIN = "#9063af", "#3a9b92", "#e2c480"
FRONT, REAR = "front", "rear"


def part(name, shape, color):
    return name, shape, color


def transform(angle=0, shift=(0, 0, 0)):
    trsf = gp_Trsf()
    if angle:
        trsf.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(*Z)), angle)
    trsf.SetTranslationPart(gp_Vec(*shift))
    return trsf


def placed(shape, trsf):
    return BRepBuilderAPI_Transform(shape, trsf, False).Shape()


def place_records(records, angle=0, shift=(0, 0, 0), prefix=""):
    trsf = transform(angle, shift)
    return [(prefix + name, placed(shape, trsf), color) for name, shape, color in records]


def build_robot():
    base.make_wheel()
    base.make_drive()
    base.make_frame()
    base.make_flywheels(base.stock_flywheel_parts())
    base.make_electronics()
    base.make_cargo_body()
    base.verify_storage_access()
    records = []
    for group, name, shape, color in base.PARTS:
        if name.endswith("连接口遮盖"):
            continue
        if name in ("左叉臂 · 6 mm 侧板", "右叉臂 · 6 mm 侧板"):
            y = -55 if name.startswith("左") else 49
            shape = cut(shape, *(cylinder((x, y, 300), Y, 2.75, 6) for x in (-29, 29)))
        records.append(part(name, shape, color))
    return records


def cross_tube(y0, y1):
    z = BEAM_HEIGHT
    return cut(box(-12, y0, z-12, 24, y1-y0, 24),
               box(-10, y0, z-10, 20, y1-y0, 20))


def mounting():
    z = BEAM_HEIGHT
    right = fuse(
        cut(box(-36, 55, 292, 72, 7, 16),
            *(cylinder((x, 55, 300), Y, 2.75, 7) for x in (-29, 29))),
        box(-16, 62, 296, 32, 29, 10),
        box(-13, 80, 306, 26, 11, 48),
    )
    hardware = [part("右横梁根部承力支架 · 与金属叉臂连接", right, BLUE_DARK),
                part("左横梁根部承力支架 · 与金属叉臂连接",
                     placed(right, transform(math.pi)), BLUE_DARK),
                part("左半横梁 · 24×24×2 方管", cross_tube(-158, -91), BLUE),
                part("右半横梁 · 24×24×2 方管", cross_tube(91, 152), BLUE)]
    for x in (-29, 29):
        right_bolt = fuse(cylinder((x, 46.8, 300), Y, 2.5, 15.2),
                          cylinder((x, 62, 300), Y, 4.25, 5))
        hardware.extend([part("右承力支架 M5 固定螺钉", right_bolt, PIN),
                         part("左承力支架 M5 固定螺钉",
                              placed(right_bolt, transform(math.pi)), PIN)])
    tongue = cut(fuse(box(-9.9, -158, z-9.9, 19.8, 10, 19.8),
                      box(-7.8, -175, z-7.8, 15.6, 17, 15.6)),
                 cylinder((0, -167, z-7.8), Z, 3.2, 15.6))
    socket = cut(box(-18, 152, z-18, 36, 30, 36),
                 box(-8.2, 161, z-8.2, 16.4, 21, 16.4),
                 cylinder((0, 173, z-18), Z, 3.2, 36),
                 cylinder((0, 173, z+16), Z, 5.2, 2))
    hardware.extend([
        part("左横梁矩形连接舌 · Ø6 插销孔", tongue, BLUE_DARK),
        part("右侧承力座 · 邻机连接舌插口", socket, BLUE),
        part("横梁锁止销 Ø6", fuse(cylinder((0, 173, z-18), Z, 3, 34),
                                  cylinder((0, 173, z+16), Z, 5, 2)), PIN),
    ])
    # Separate forward and rear guide sleeves attach to the same seat.
    for side, y_direction in ((FRONT, 1), (REAR, -1)):
        sleeve = cut(box(18, -13, z-13, 76, 26, 26),
                     box(18, -11.1, z-11.1, 76, 22.2, 22.2))
        hardware.append(part(f"{'前' if side == FRONT else '后'}纵杆独立导套",
                             placed(sleeve, transform(0 if y_direction > 0 else math.pi,
                                                      (0, HALF_TRACK, 0))), ORANGE))
    return hardware


def drawbar(side, extended):
    z = BEAM_HEIGHT
    start1, start2, start3 = (76, 129, 179) if extended else (24, 24, 59)
    endpoint = HALF_ROW_SPACING if extended else 105
    endpoint_z = z-5.5

    def tube(start, length, outer, inner):
        return cut(box(start, -outer/2, z-outer/2, length, outer, outer),
                   box(start, -inner/2, z-inner/2, length, inner, inner))

    items = [part("伸缩纵杆 一级方管", tube(start1, 70, 21, 17), ORANGE_LIGHT),
             part("伸缩纵杆 二级方管", tube(start2, 69, 16, 12), ORANGE),
             part("伸缩纵杆 三级内杆", box(start3, -5.5, endpoint_z, 34, 11, 11), ORANGE_LIGHT)]
    if side == FRONT:
        head = cut(fuse(box(endpoint-12, -5.5, z-5, 12, 11, 10),
                        cylinder((endpoint, 0, z-5), Z, 9.5, 10)),
                   cylinder((endpoint, 0, z-5), Z, 4.2, 10))
        items.append(part("前纵杆刚性插头 · 中央铰接舌", head, ORANGE))
    else:
        lower = fuse(box(endpoint-11, -7, z-10, 11, 14, 4),
                     cylinder((endpoint, 0, z-10), Z, 11, 4))
        upper = fuse(box(endpoint-11, -7, z+6, 11, 14, 4),
                     cylinder((endpoint, 0, z+6), Z, 11, 4))
        lock = cut(fuse(lower, upper, box(endpoint-11, -7, z-10, 1.5, 14, 20),
                        box(endpoint-12, -5.5, z-5.5, 1, 11, 11)),
                   cylinder((endpoint, 0, z-10), Z, 4.2, 20))
        items.extend([
            part("后纵杆铰接锁头 · 双耳叉", lock, ORANGE),
            part("H 中央竖直 Ø8 铰轴" if extended else "收纳的竖直 Ø8 铰轴",
                 fuse(cylinder((endpoint, 0, z-11), Z, 4, 22),
                      cylinder((endpoint, 0, z+11), Z, 6, 2)), PIN),
        ])
    return place_records(items, 0 if side == FRONT else math.pi, (0, HALF_TRACK, 0))


def rack_support(sliding):
    prefix = "后排" if sliding else "前排"
    x, y = 0, HALF_TRACK
    return [
        part(prefix+"货架承力立柱", cylinder((x, y, SEAT_TOP), Z, 10, 66), BLUE_DARK),
        part(prefix+"转盘下座", ring((x, y, 426), Z, 23, 4.3, 6), PURPLE),
        part(prefix+"转盘上座", ring((x, y, 432), Z, 14 if sliding else 22, 4.3, 5), PURPLE),
        part(prefix+"转盘竖轴", cylinder((x, y, 426), Z, 4, 11), PIN),
    ]


def rack_parts():
    items = [
        part("刚性货架面板", box(-322, -235, RACK_BOTTOM, 644, 470, 6), "#aa8dc7"),
        part("前转盘固定座", box(201, -21, 437, 48, 42, 10), PURPLE),
    ]
    for y in (-195, 185):
        items.append(part("货架纵向加劲梁", box(-298, y, 435, 596, 10, 12), PURPLE))
    for sign in (-1, 1):
        wall_y = 21 if sign > 0 else -27
        ledge_y = 15 if sign > 0 else -27
        items.append(part("后转盘纵向滑轨", fuse(
            box(-285, wall_y, 435, 140, 6, 12),
            box(-285, ledge_y, 435, 140, 12, 2)), TEAL))
    for x in (-287, -145):
        items.append(part("滑轨行程止块", box(x, -20.7, 437, 2, 41.4, 8), TEAL))
    return items


def slider():
    return [part("后转盘滑块", box(-249, -20.6, 437, 48, 41.2, 8), TEAL)]


def pose(degrees):
    half = math.radians(degrees/2)
    centers = {FRONT: (HALF_ROW_SPACING*math.cos(half), HALF_ROW_SPACING*math.sin(half)),
               REAR: (-HALF_ROW_SPACING*math.cos(half), HALF_ROW_SPACING*math.sin(half))}
    return centers, {FRONT: half, REAR: -half}


def robot_origin(center, angle, side):
    y = side * HALF_TRACK
    return (center[0] - math.sin(angle)*y, center[1] + math.cos(angle)*y, 0)


def build_configuration(degrees, robot, hardware, rods, supports, rack, carriage):
    centers, angles = pose(degrees)
    units = []
    extras = []
    for index, (row, side) in enumerate(((FRONT, -1), (FRONT, 1), (REAR, -1), (REAR, 1)), 1):
        angle = angles[row]
        origin = robot_origin(centers[row], angle, side)
        front_extended = index == 3
        rear_extended = index == 1
        connections = hardware + rods[FRONT, front_extended] + rods[REAR, rear_extended]
        if side == -1:
            connections += supports[row]
        prefix = f"{index}号 "
        units.append((place_records(robot, angle, origin, prefix),
                      place_records(connections, angle, origin, prefix)))
    x_front, y_front = centers[FRONT]
    x_rear, y_rear = centers[REAR]
    extras += place_records(rack, shift=(x_front-HALF_ROW_SPACING, y_front, 0))
    extras += place_records(carriage, shift=(x_rear+HALF_ROW_SPACING, y_rear, 0))
    return units, extras


def intersection_test(collections):
    """Check new components and pairs between machines, retaining the baseline CAD checks."""
    boxed = [[(name, shape, base.shape_bounds(shape)) for name, shape, _ in sub]
             for sub in collections]

    def compare(group_a, group_b, within=False):
        intersections = []
        for i, (aname, ashape, a) in enumerate(group_a):
            for bname, bshape, b in (group_b[i+1:] if within else group_b):
                if any(min(a[k+3], b[k+3])-max(a[k], b[k]) <= .009 for k in range(3)):
                    continue
                operation = BRepAlgoAPI_Common(ashape, bshape)
                if not operation.IsDone():
                    raise ValueError(f"Interference test failed: {aname} / {bname}")
                overlap = base.volume(operation.Shape())
                if overlap > .15:
                    intersections.append((aname, bname, round(overlap, 3)))
        return intersections

    errors = []
    for i, group in enumerate(boxed):
        # Even groups below 8 are the unchanged baseline solids, with fork holes.
        if i % 2 or i == 8:
            errors.extend(compare(group, group, within=True))
        for later in boxed[i+1:]:
            errors.extend(compare(group, later))
    return errors


class StepAssembly:
    def __init__(self, name):
        self.doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
        self.tool = XCAFDoc_DocumentTool.ShapeTool_s(self.doc.Main())
        self.root = self.new(name)

    @staticmethod
    def name(label, text):
        TDataStd_Name.Set_s(label, TCollection_ExtendedString(text, True))
        return label

    def new(self, name):
        return self.name(self.tool.NewShape(), name)

    def add(self, parent, node, title, angle=0, shift=(0, 0, 0)):
        return self.name(self.tool.AddComponent(parent, node, TopLoc_Location(transform(angle, shift))), title)

    def module(self, title, records):
        node = self.new(title)
        for name, shape, _ in records:
            label = self.name(self.tool.AddShape(shape, False), name)
            self.add(node, label, name)
        return node

    def write(self, path):
        self.tool.UpdateAssemblies()
        writer = STEPCAFControl_Writer()
        writer.SetNameMode(True)
        if not writer.Transfer(self.doc, STEPControl_AsIs):
            raise RuntimeError(f"STEP transfer failed: {path}")
        if writer.Write(str(path)) != IFSelect_RetDone:
            raise RuntimeError(f"STEP write failed: {path}")
        reopened = base.read_step(path)
        expected = self.tool.GetShape_s(self.root)
        if any(abs(a-b) > .02 for a, b in zip(base.shape_bounds(reopened), base.shape_bounds(expected))):
            raise RuntimeError(f"STEP round-trip bounds mismatch: {path}")


def export_step(robot, hardware, rods, supports, rack, carriage, four):
    model = StepAssembly("GyroRoll R1B · 四机中央车钩" if four else "GyroRoll R1B · 单机连接件")
    modules = {
        "robot": model.module("R1B 机器人本体", robot),
        "hardware": model.module("左右半横梁与右侧承力座", hardware),
        "front0": model.module("前纵杆 · 收回", rods[FRONT, False]),
        "rear0": model.module("后纵杆 · 收回", rods[REAR, False]),
    }
    if not four:
        for title, node in modules.items():
            model.add(model.root, node, title)
        model.write(OUTPUT/"GyroRoll_R1B_connection_single.step")
        return
    modules.update({
        "front1": model.module("前纵杆 · 伸出", rods[FRONT, True]),
        "rear1": model.module("后纵杆 · 伸出", rods[REAR, True]),
        "front support": model.module("前转盘", supports[FRONT]),
        "rear support": model.module("后转盘", supports[REAR]),
        "rack": model.module("刚性货架与滑轨", rack),
        "slider": model.module("后转盘滑块", carriage),
    })
    for index, (x, y) in enumerate(((225, -170), (225, 170), (-225, -170), (-225, 170)), 1):
        unit = model.new(f"{index}号机器人")
        for label in ("robot", "hardware", "front1" if index == 3 else "front0",
                      "rear1" if index == 1 else "rear0"):
            model.add(unit, modules[label], label)
        if index in (1, 3):
            label = "front support" if index == 1 else "rear support"
            model.add(unit, modules[label], label)
        model.add(model.root, unit, f"{index}号机器人", shift=(x, y, 0))
    model.add(model.root, modules["rack"], "上层刚性货架")
    model.add(model.root, modules["slider"], "后转盘滑块")
    model.write(OUTPUT/"GyroRoll_R1B_connection_four.step")


def export_html(robot, hardware, rods, supports, rack, carriage):
    def meshes(records):
        return [dict(name=name, color=color, **base.mesh(shape)) for name, shape, color in records]

    data = {
        "robot": meshes(robot), "mounting": meshes(hardware),
        "front0": meshes(rods[FRONT, False]), "rear0": meshes(rods[REAR, False]),
        "front1": meshes(rods[FRONT, True]), "rear1": meshes(rods[REAR, True]),
        "frontSupport": meshes(supports[FRONT]), "rearSupport": meshes(supports[REAR]),
        "rack": meshes(rack), "slider": meshes(carriage),
        "halfTrack": HALF_TRACK, "halfSpacing": HALF_ROW_SPACING, "foldLimit": FOLD_LIMIT,
    }
    library = base_path/"viewer_lib"
    three = (library/"three.min.js").read_text(encoding="utf-8")
    license_text = (library/"three-LICENSE.txt").read_text(encoding="utf-8")
    template = (HERE/"viewer.html.in").read_text(encoding="utf-8")
    page = template.replace("/* THREE_BUNDLE */", "/*\n"+license_text+"\n*/\n"+three)
    page = page.replace("/* CAD_DATA */", json.dumps(data, ensure_ascii=False, separators=(",", ":")))
    (OUTPUT/"index.html").write_text(page, encoding="utf-8")


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    robot = build_robot()
    hardware = mounting()
    rods = {(side, extended): drawbar(side, extended) for side in (FRONT, REAR)
            for extended in (False, True)}
    supports = {FRONT: rack_support(False), REAR: rack_support(True)}
    rack, carriage = rack_parts(), slider()
    for name, shape, _ in [*robot, *hardware, *rack, *carriage,
                           *(part for group in (*rods.values(), *supports.values()) for part in group)]:
        if not BRepCheck_Analyzer(shape).IsValid() or base.volume(shape) <= 0:
            raise ValueError(f"Invalid solid: {name}")
    reports = {}
    for angle in (0, -18, 18, -FOLD_LIMIT, FOLD_LIMIT):
        units, extras = build_configuration(angle, robot, hardware, rods, supports, rack, carriage)
        reports[str(angle)] = intersection_test([sub for unit in units for sub in unit]+[extras])
        print(f"Fold {angle:+} deg: {reports[str(angle)]}", flush=True)
    (OUTPUT/"clearance.json").write_text(json.dumps(reports, ensure_ascii=False, indent=2), encoding="utf-8")
    if any(reports.values()):
        raise ValueError("Interference exceeds 0.15 mm³")
    export_step(robot, hardware, rods, supports, rack, carriage, four=False)
    export_step(robot, hardware, rods, supports, rack, carriage, four=True)
    export_html(robot, hardware, rods, supports, rack, carriage)


if __name__ == "__main__":
    main()
