"""Build GyroRoll belt-drive CAD in mm. Requires OpenCascade via cadquery-ocp-novtk."""

import json
import math
from pathlib import Path

from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Builder, BRep_Tool
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon,
    BRepBuilderAPI_MakeWire, BRepBuilderAPI_Transform,
)
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepPrimAPI import (
    BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakePrism, BRepPrimAPI_MakeTorus,
)
from OCP.GC import GC_MakeArcOfCircle
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.StlAPI import StlAPI_Writer
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.gp import gp_Ax1, gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec
from mass import estimate

HERE = Path(__file__).resolve().parent
OUTPUT = HERE.parent / "cad"
X, Y = (1, 0, 0), (0, 1, 0)
Z = (0, 0, 1)
TAU = math.tau
PITCH = 3
WHEEL_Z = 90
WHEEL_DIAMETER = 175
WHEEL_WIDTH = 50
BELT_WIDTH = 9
WHEEL_PULLEY_Y = -40
FLY_PULLEY_FACE = 12.8
FLY_PULLEY_LENGTH = 20.6
FLY_MOTOR_Z = 200
FLY_BEAM_TOP = 189
FLY_OFFSET = 90
FLY_PULLEY_TEETH = 24
FLY_BELT_LENGTH = 255
FLY_DISTANCE = (FLY_BELT_LENGTH-PITCH*FLY_PULLEY_TEETH)/2
FLY_DROP = math.sqrt(FLY_DISTANCE**2-FLY_OFFSET**2)
FLY_Z = FLY_MOTOR_Z-FLY_DROP
LOWER_SUPPORT_TOP = 170
SHELL_SEAM = 302
CABIN_BOTTOM = 308
CABIN_FLOOR = 312
DECK_BOTTOM = 420
DECK_TOP = 424
FLY_DISC_X = 5
FLY_DISC_R = 50
FLY_DISC_FACE = 5
FLY_DISC_TOTAL = 12
FLY_SHFT_R = 4
FLY_DRIVE_TRACKS = (33.6, 55.6)
STOCK_PULLEY_FACE = 15
STOCK_PULLEY_LENGTH = 23
CLEAR_STORAGE = (170, 128, 106)
DECK_SIZE = (180, 154)
WHEEL_BELT_LENGTH = 459


def static(cls, name, *args):
    method = getattr(cls, name + "_s", None) or getattr(cls, name)
    return method(*args)


def cut(shape, *tools):
    for tool in tools:
        op = BRepAlgoAPI_Cut(shape, tool)
        if not op.IsDone():
            raise ValueError("Boolean cut failed")
        shape = op.Shape()
    return shape


def fuse(*shapes):
    result = shapes[0]
    for shape in shapes[1:]:
        op = BRepAlgoAPI_Fuse(result, shape)
        if not op.IsDone():
            raise ValueError("Boolean fuse failed")
        result = op.Shape()
    return result


def compound(shapes):
    builder = BRep_Builder()
    result = TopoDS_Compound()
    builder.MakeCompound(result)
    for shape in shapes:
        builder.Add(result, shape)
    return result


def cylinder(p, axis, radius, length):
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*p), gp_Dir(*axis)), radius, length).Shape()


def box(x, y, z, dx, dy, dz):
    return BRepPrimAPI_MakeBox(gp_Pnt(x, y, z), dx, dy, dz).Shape()


def ring(p, axis, outer, inner, length):
    return cut(cylinder(p, axis, outer, length), cylinder(p, axis, inner, length))


def prism(vertices, v):
    poly = BRepBuilderAPI_MakePolygon()
    for pt in vertices:
        poly.Add(gp_Pnt(*pt))
    poly.Close()
    face = BRepBuilderAPI_MakeFace(poly.Wire()).Face()
    return BRepPrimAPI_MakePrism(face, gp_Vec(*v)).Shape()


def side_plate(profile, y0, thickness):
    return prism([(x, y0, z) for x, z in profile], (0, thickness, 0))


def slot_y(x, y, z, radius, travel, thickness):
    return fuse(
        cylinder((x, y, z-travel/2), Y, radius, thickness),
        cylinder((x, y, z+travel/2), Y, radius, thickness),
        box(x-radius, y, z-travel/2, 2*radius, thickness, travel),
    )


def bolt_circle(p, axis, pcd, count, radius, depth, clock=0):
    holes = []
    for i in range(count):
        a = clock + TAU*i/count
        u, v = pcd/2*math.cos(a), pcd/2*math.sin(a)
        x, y, z = p
        q = (x, y+u, z+v) if axis == X else (x+u, y, z+v)
        holes.append(cylinder(q, axis, radius, depth))
    return holes


def pitch_radius(teeth):
    return PITCH*teeth/TAU


def pulley(p, axis, teeth, width, bore):
    # Tooth outlines show the tooth count and envelope. Use catalog pulleys for hardware.
    verts = []
    for i in range(teeth):
        for phase, depth in ((0, .95), (.23, .3), (.58, .3), (.82, .95)):
            a = TAU*(i+phase)/teeth
            r = pitch_radius(teeth)-depth
            u, v = r*math.cos(a), r*math.sin(a)
            x, y, z = p
            verts.append((x, y+u, z+v) if axis == X else (x+u, y, z+v))
    extrusion = tuple(t*width for t in axis)
    body = prism(verts, extrusion)
    flange_r = pitch_radius(teeth)+1.4
    end = tuple(a+(width-.9)*b for a, b in zip(p, axis))
    body = fuse(body, cylinder(p, axis, flange_r, .9), cylinder(end, axis, flange_r, .9))
    return cut(body, cylinder(p, axis, bore, width))


def belt_length(c, a, b):
    d = abs(a-b)
    return 2*math.sqrt(c*c-d*d) + math.pi*(a+b) + 2*d*math.asin(d/c)


def center_distance(length, a, b):
    lo, hi = a+b, length/2
    for _ in range(50):
        mid = (lo+hi)/2
        if belt_length(mid, a, b) > length:
            hi = mid
        else:
            lo = mid
    return (lo+hi)/2


def belt_loop(map_point, distance, r1, r2, width):
    # Basis in the belt plane: first coordinate along the line of centers.
    cos_a = (r1-r2)/distance
    a = math.acos(cos_a)

    def loop(offset):
        def point(center, r, angle):
            return gp_Pnt(*map_point(center+(r+offset)*math.cos(angle), (r+offset)*math.sin(angle)))

        start1, end1 = point(0, r1, a), point(0, r1, TAU-a)
        start2, end2 = point(distance, r2, TAU-a), point(distance, r2, a)
        wire = BRepBuilderAPI_MakeWire()
        for curve in (
            BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(start1, point(0, r1, math.pi), end1).Value()),
            BRepBuilderAPI_MakeEdge(end1, start2),
            BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(start2, point(distance, r2, 0), end2).Value()),
            BRepBuilderAPI_MakeEdge(end2, start1),
        ):
            wire.Add(curve.Edge())
        face = BRepBuilderAPI_MakeFace(wire.Wire()).Face()
        return BRepPrimAPI_MakePrism(face, gp_Vec(*width)).Shape()

    return cut(loop(1.35), loop(-.28))


WHEEL_C = center_distance(WHEEL_BELT_LENGTH, pitch_radius(60), pitch_radius(20))
DRIVE_Z = WHEEL_Z + WHEEL_C

PARTS = []
GROUPS = {
    "wheel": ("滚动轮与轮轴", "#9fb8cf", (0, -35, -25)),
    "drive": ("DD9015-1 与滚动轮传动", "#ffb45a", (0, -55, 6)),
    "frame": ("叉架与承力骨架", "#476783", (0, 35, 0)),
    "fly": ("双飞轮及支承", "#d97b41", (25, 0, 26)),
    "flyDrive": ("M6C12 与飞轮传动", "#f0a055", (60, 0, 45)),
    "electronics": ("电池、树莓派与驱动", "#7ad4a2", (44, 0, 28)),
    "cover": ("下部电子舱盖", "#36647d", (68, 0, 48)),
    "body": ("中空储物机身", "#36647d", (0, 0, 64)),
    "door": ("储物舱前门", "#36647d", (78, 0, 64)),
    "deck": ("顶部载物平台", "#dfe5e3", (0, 0, 115)),
}


def add(group, name, shape, color=None):
    PARTS.append((group, name, shape, color or GROUPS[group][1]))


def read_step(path):
    reader = STEPControl_Reader()
    if reader.ReadFile(str(path)) != IFSelect_RetDone or not reader.TransferRoots():
        raise ValueError(f"Cannot import {path.name}")
    return reader.OneShape()


def bearing(p, axis, label, group, bore=6, outer=14, length=8):
    add(group, label+"外圈", ring(p, axis, outer, outer-2.6, length), "#697f91")
    add(group, label+"内圈", ring(p, axis, bore+2.1, bore, length), "#afb9bf")
    sealp = tuple(a+1.4*b for a, b in zip(p, axis))
    add(group, label+"密封圈", ring(sealp, axis, outer-2.6, bore+2.1, length-2.8), "#283843")


def make_wheel():
    half_width = WHEEL_WIDTH/2
    tread = BRepPrimAPI_MakeTorus(
        gp_Ax2(gp_Pnt(0, 0, WHEEL_Z), gp_Dir(*Y)), WHEEL_DIAMETER/2-half_width, half_width,
    ).Shape()
    add("wheel", "Ø175 × 50 圆冠橡胶胎 · 样轮外形占位",
        cut(tread, cylinder((0, -half_width-1, WHEEL_Z), Y, 64, WHEEL_WIDTH+2)), "#222e37")
    hub = fuse(
        ring((0, -half_width, WHEEL_Z), Y, 64, 57, WHEEL_WIDTH),
        ring((0, -3, WHEEL_Z), Y, 59, 6, 6),
        ring((0, -half_width, WHEEL_Z), Y, 17, 6, WHEEL_WIDTH),
    )
    wheel_key = box(-2, -16, WHEEL_Z+3.5, 4, 32, 4)
    pulley_key = box(-2, -38.5, WHEEL_Z+3.5, 4, 8, 4)
    hub = cut(hub, wheel_key, *bolt_circle((0, -4, WHEEL_Z), Y, 79, 6, 11.2, 8))
    add("wheel", "铝轮毂传扭结构 · 旧轴系占位", hub)
    add("wheel", "滚动轮黄色相位标记", box(-2, -half_width-.24, WHEEL_Z+59, 4, .24, 4), "#f3c94f")
    add("wheel", "Ø12 贯通轮轴", cut(cylinder((0, -65, WHEEL_Z), Y, 6, 130), wheel_key, pulley_key), "#b4cad5")
    add("wheel", "轮毂传扭键", wheel_key, "#d6baa1")
    add("wheel", "60T轮端带轮传扭键", pulley_key, "#d6baa1")
    for y, spacer_y, side in [(-63, -55, "左"), (55, 49, "右")]:
        add("wheel", side+"Ø12×24×8 止动环包络", ring((0, y, WHEEL_Z), Y, 12, 6, 8), "#495968")
        add("wheel", side+"轮轴定位隔套 L6", ring((0, spacer_y, WHEEL_Z), Y, 8, 6, 6), "#8da3b2")
    for y, side in [(-49, "左"), (41, "右")]:
        bearing((0, y, WHEEL_Z), Y, side+"6001轴承", "wheel")
    for y, length in [(-41, 1), (-29, 4), (25, 16)]:
        add("wheel", "轮轴定位隔套", ring((0, y, WHEEL_Z), Y, 8, 6, length), "#8da3b2")
    add("drive", "滚动轮 60T 从动轮", cut(pulley((0, WHEEL_PULLEY_Y, WHEEL_Z), Y, 60, 11, 6), pulley_key))
    add("drive", "滚动轮 20T 主动轮", pulley((0, WHEEL_PULLEY_Y, DRIVE_Z), Y, 20, 11, 5))
    add("drive", f"滚动轮 {WHEEL_BELT_LENGTH} mm 节线同步带", belt_loop(
        lambda u, v: (v, WHEEL_PULLEY_Y+1, WHEEL_Z+u),
        WHEEL_C, pitch_radius(60), pitch_radius(20), (0, BELT_WIDTH, 0)
    ), "#394654")


def make_drive():
    native = read_step(HERE/"DD9015_supplier.step")
    rot = gp_Trsf()
    rot.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(*Z)), math.pi/2)
    rot.SetTranslationPart(gp_Vec(0, 6, DRIVE_Z))
    add("drive", "SteadyWin DD9015-1 · 供应商 STEP",
        BRepBuilderAPI_Transform(native, rot, True).Shape(), "#657e93")
    mount = cut(
        side_plate([(-50, DRIVE_Z-47), (50, DRIVE_Z-47), (50, DRIVE_Z+26),
                    (35, DRIVE_Z+41), (-50, DRIVE_Z+41)], 15, 6),
        cylinder((0, 14, DRIVE_Z), Y, 26, 8),
        *bolt_circle((0, 14, DRIVE_Z), Y, 80, 4, 2.25, 8),
        *[cylinder((x, 15, DRIVE_Z-20.75), Y, 2.25, 6) for x in (-45, 45)],
    )
    add("frame", "DD9015-1 电机安装板", mount)
    adapter = cut(
        fuse(cylinder((0, -29, DRIVE_Z), Y, 32, 6), cylinder((0, -48, DRIVE_Z), Y, 5, 19)),
        *[cylinder((-u, -29, DRIVE_Z+v), Y, 2.25, 6)
          for u, v in [(-24.683, -3.968), (-3.968, 24.683), (3.968, -24.683), (24.683, 3.968)]],
    )
    add("drive", "DD9015-1 输出法兰至20T适配件", adapter, "#ecad66")
    add("frame", "滚动主动轮外支承 · 随电机同步张紧", cut(
        box(-24, -49, DRIVE_Z-15, 48, 8, 30), cylinder((0, -49, DRIVE_Z), Y, 11, 8),
        *[cylinder((x, -49, DRIVE_Z), Y, 2.25, 8) for x in (-18, 18)]))
    bearing((0, -48, DRIVE_Z), Y, "滚动主动轮6900轴承", "drive", 5, 11, 6)


def make_frame():
    outline = [(-19, 69), (22, 69), (33, 88), (30, 159), (58, 190), (61, 245),
               (61, CABIN_BOTTOM), (-61, CABIN_BOTTOM), (-61, 190), (-29, 155), (-29, 89)]
    for y, side in [(-55, "左"), (49, "右")]:
        plate = cut(
            side_plate(outline, y, 6),
            cylinder((0, y, WHEEL_Z), Y, 10, 6),
            slot_y(-15, y, 131, 4.5, 33, 6),
            slot_y(14, y, 150, 4.5, 35, 6),
            *[slot_y(x, y, DRIVE_Z-20.75, 3, 13, 6) for x in (-45, 45)],
            box(0, y, FLY_Z-57, 13.5, 6, 114),
            *(box(x, y, SHELL_SEAM, 7, 6, 6) for x in (-60, 53)),
        )
        if side == "左":
            plate = cut(plate, box(32.5, y, 174, 14, 6, 35),
                        *[slot_y(x, y, DRIVE_Z, 2.25, 8, 6) for x in (-18, 18)])
        else:
            plate = cut(plate, box(54.5, y, 174, 14, 6, 35),
                        *[box(-34, y, z, 68, 6, 2.7) for z in range(259, 294, 7)])
        add("frame", side+"叉臂 · 6 mm 侧板", plate)
    for y, side in [(-49, "左"), (41, "右")]:
        seat = cut(box(-21, y, 68, 42, 8, 44), cylinder((0, y, WHEEL_Z), Y, 14, 8))
        add("frame", side+"轮轴承座", seat)
    crossbars = [box(x, -77, SHELL_SEAM, 7, 154, 6) for x in (-60, 53)]
    side_rails = [box(x, y, SHELL_SEAM, 73, 6, 6)
                  for x in (-90, 17) for y in (-77, 71)]
    end_rails = [box(x, -77, SHELL_SEAM, 6, 154, 6) for x in (-90, 84)]
    side_tabs = [box(x-6, y, 294, 12, 3.5, 8)
                 for x in (-56, 56) for y in (-74.5, 71)]
    end_tabs = [box(x, y-6, 294, 3.5, 12, 8)
                for x in (-87.5, 84) for y in (-48, 48)]
    holes = [cylinder((x, y, SHELL_SEAM), Z, 2.1, 6) for x in (-56, 56) for y in (-40, 40)]
    holes += [cylinder((x, y, SHELL_SEAM), Z, 5.2, 6) for x in (-82, 82) for y in (-70, 70)]
    holes += [cylinder((x, y, 298), Y, 1.25, 3.5) for x in (-56, 56) for y in (-74.5, 71)]
    holes += [cylinder((x, y, 298), X, 1.25, 3.5) for x in (-87.5, 84) for y in (-48, 48)]
    add("frame", "180×154 齐平上沿承力框 · 含设备面板固定耳",
        cut(fuse(*crossbars, *side_rails, *end_rails, *side_tabs, *end_tabs), *holes), "#476477")
    for x, dx in [(-42, 8), (25, 7)]:
        add("frame", "飞轮中央横梁", box(x, -49, FLY_BEAM_TOP-7, dx, 98, 7))
        for y, rise_y, shaft_y in [(-116, -63, -90), (55, 55, 90)]:
            step = fuse(
                box(x, y, LOWER_SUPPORT_TOP-7, dx, 61, 7),
                box(x, rise_y, LOWER_SUPPORT_TOP, dx, 8, FLY_BEAM_TOP-LOWER_SUPPORT_TOP),
            )
            add("frame", "飞轮阶梯支承横梁 · M3底孔", cut(
                step,
                *[cylinder((x+3.5, shaft_y+offset, LOWER_SUPPORT_TOP-7), Z, 1.25, 7)
                  for offset in (-20, 20)],
            ))
    for x in (-49, 41):
        add("frame", "可调电机座连接柱", cut(box(x, 21, DRIVE_Z-29.75, 8, 28, 18),
            cylinder((x+4, 21, DRIVE_Z-20.75), Y, 2.25, 28)))


def shaft_block(x, y, z, foot=FLY_BEAM_TOP, outer=11, thickness=7):
    return cut(box(x, y-14, foot, thickness, 28, z+15-foot),
               cylinder((x, y, z), X, outer, thickness))


def adjustable_fly_mount(x, y):
    # Each pair of blocks can slide together by ±2 mm along the belt span.
    foot = LOWER_SUPPORT_TOP
    base = fuse(shaft_block(x, y, FLY_Z, foot=foot), box(x, y-25, foot, 7, 50, 4))
    slots = []
    for offset in (-20, 20):
        center = y+offset
        slots.append(fuse(
            box(x+1.75, center-2, foot, 3.5, 4, 4),
            cylinder((x+3.5, center-2, foot), Z, 1.75, 4),
            cylinder((x+3.5, center+2, foot), Z, 1.75, 4),
        ))
    return cut(base, cylinder((x, y, FLY_Z), X, 11, 7), *slots)


def fly_belt(pulley_x, side):
    horizontal = FLY_OFFSET/FLY_DISTANCE
    vertical = FLY_DROP/FLY_DISTANCE
    radius = pitch_radius(FLY_PULLEY_TEETH)

    def position(u, v):
        return (pulley_x+(FLY_PULLEY_FACE-BELT_WIDTH)/2,
                side*(horizontal*u+vertical*v), FLY_MOTOR_Z-vertical*u+horizontal*v)

    return belt_loop(position, FLY_DISTANCE, radius, radius, (BELT_WIDTH, 0, 0))


def fly_pulley(x, y, z):
    # Catalog 24-3M-09: 12.8 mm face stack, 20.6 mm total including the hub.
    return fuse(
        pulley((x, y, z), X, FLY_PULLEY_TEETH, FLY_PULLEY_FACE, 5),
        ring((x+FLY_PULLEY_FACE, y, z), X, 10.3, 5, FLY_PULLEY_LENGTH-FLY_PULLEY_FACE),
    )


def shaft_with_retaining_rings(x, y, z, length, clips, group, label):
    # DIN 471 clips are envelopes; machine the grooves to the chosen clip drawing.
    groove_width, groove_radius = .8, 4.7
    grooves = [ring((clip_x, y, z), X, 5, groove_radius, groove_width) for clip_x in clips]
    add(group, label, cut(cylinder((x, y, z), X, 5, length), *grooves), "#acbdc8")
    for clip_x in clips:
        gap = box(clip_x, y+groove_radius-.5, z-1, groove_width, 3.5, 2)
        add(group, "DIN 471 Ø10 轴用挡圈",
            cut(ring((clip_x, y, z), X, 6.6, groove_radius, groove_width), gap), "#495968")


def stock_fly_pulley(x, y):
    # DOLD 863916 dimensions. Tooth profiles and set-screw features are envelopes.
    return fuse(
        pulley((x, y, FLY_Z), X, FLY_PULLEY_TEETH, STOCK_PULLEY_FACE, FLY_SHFT_R),
        ring((x+STOCK_PULLEY_FACE, y, FLY_Z), X, 7, FLY_SHFT_R,
             STOCK_PULLEY_LENGTH-STOCK_PULLEY_FACE),
    )


def flywheel_window(angle):
    """Rounded, through-cut window between two 120-degree-spaced spokes."""
    spoke_half = 3.3
    inner_radius, outer_radius, corner_radius = 13.5, 29, 3
    half_sector = math.pi/3

    def side_tangencies(radius, corner_offset):
        radial = radius+corner_offset
        offset = spoke_half+corner_radius
        center = (math.sqrt(radial**2-offset**2), offset)
        return center, (center[0], spoke_half), tuple(value*radius/radial for value in center)

    def mirror(p):
        cos_angle, sin_angle = math.cos(2*half_sector), math.sin(2*half_sector)
        return cos_angle*p[0]+sin_angle*p[1], sin_angle*p[0]-cos_angle*p[1]

    def world(p):
        c, s = math.cos(angle), math.sin(angle)
        return gp_Pnt(c*p[0]-s*p[1], s*p[0]+c*p[1], 0)

    def arc(a, b, center):
        first = math.atan2(a[1]-center[1], a[0]-center[0])
        last = math.atan2(b[1]-center[1], b[0]-center[0])
        half = first + math.remainder(last-first, TAU)/2
        radius = math.dist(a, center)
        mid = (center[0]+radius*math.cos(half), center[1]+radius*math.sin(half))
        return BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(world(a), world(mid), world(b)).Value())

    ci, li, ti = side_tangencies(inner_radius, corner_radius)
    co, lo, to = side_tangencies(outer_radius, -corner_radius)
    wire = BRepBuilderAPI_MakeWire()
    for edge in (
        BRepBuilderAPI_MakeEdge(world(li), world(lo)),
        arc(lo, to, co),
        arc(to, mirror(to), (0, 0)),
        arc(mirror(to), mirror(lo), mirror(co)),
        BRepBuilderAPI_MakeEdge(world(mirror(lo)), world(mirror(li))),
        arc(mirror(li), mirror(ti), mirror(ci)),
        arc(mirror(ti), ti, (0, 0)),
        arc(ti, li, ci),
    ):
        wire.Add(edge.Edge())
    face = BRepBuilderAPI_MakeFace(wire.Wire()).Face()
    return BRepPrimAPI_MakePrism(face, gp_Vec(0, 0, FLY_DISC_FACE)).Shape()


def stock_flywheel_parts():
    # Planform is reconstructed from front/back product photos. Only OD/bore
    # are listed sizes. Plate, boss and fastener dimensions are fit estimates.
    spoke_angles = [math.pi/2+TAU*i/3 for i in range(3)]
    rim_holes = [cylinder((40*math.cos(a), 40*math.sin(a), 0), Z, 1.35, FLY_DISC_FACE)
                 for a in spoke_angles]
    bolt_xy = [(10.5*math.cos(TAU*i/4+.12), 10.5*math.sin(TAU*i/4+.12)) for i in range(4)]
    flange_top = 8.5
    plate = cut(
        ring((0, 0, 0), Z, FLY_DISC_R, FLY_SHFT_R, FLY_DISC_FACE),
        *(flywheel_window(a) for a in spoke_angles), *rim_holes,
        *(cylinder((x, y, 0), Z, 1.45, FLY_DISC_FACE) for x, y in bolt_xy),
    )
    hub = cut(fuse(
        ring((0, 0, FLY_DISC_FACE), Z, 14, FLY_SHFT_R, flange_top-FLY_DISC_FACE),
        ring((0, 0, flange_top), Z, 7.8, FLY_SHFT_R, FLY_DISC_TOTAL-flange_top),
    ), *(cylinder((x, y, FLY_DISC_FACE), Z, 1.45, flange_top-FLY_DISC_FACE)
         for x, y in bolt_xy))
    screws = compound(fuse(
        cylinder((x, y, 0), Z, 1.35, flange_top),
        cylinder((x, y, flange_top), Z, 2, 1.5),
    ) for x, y in bolt_xy)
    return (
        ("Ø100 × Ø8 成品惯性轮盘 · 三辐条", plate, "#e1e3dd"),
        ("成品惯性轮轮毂", hub, "#b09462"),
        ("成品惯性轮轮毂四颗螺钉", screws, "#383c3f"),
    )


def position_flywheel(shape, center):
    # Map local (x, y, z) to assembly (z, x, y), then place on the shaft.
    placement = gp_Trsf()
    placement.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(1, 1, 1)), TAU/3)
    placement.SetTranslationPart(gp_Vec(*center))
    return BRepBuilderAPI_Transform(shape, placement, True).Shape()


def make_flywheels(flywheel_parts):
    for sign, drive_x, length, side in zip((-1, 1), FLY_DRIVE_TRACKS, (112, 134), ("左", "右")):
        y = sign*FLY_OFFSET
        add("fly", side+"Ø8 飞轮轴 · 光轴定尺", cylinder((-54, y, FLY_Z), X, FLY_SHFT_R, length), "#acbdc8")
        for collar_x, spacer_x in ((-52, -43), (-34, -35)):
            add("fly", side+"Ø8 双片轴环 · MSP-8-F 包络",
                ring((collar_x, y, FLY_Z), X, 9, FLY_SHFT_R, 9), "#495968")
            add("fly", side+"Ø8 轴承内圈调整垫",
                ring((spacer_x, y, FLY_Z), X, 5.5, FLY_SHFT_R, 1), "#8da3b2")
        for name, shape, color in flywheel_parts:
            placed = position_flywheel(shape, (FLY_DISC_X, y, FLY_Z))
            add("fly", side+name, placed, color)
        add("fly", side+"飞轮相位标记",
            box(FLY_DISC_X+FLY_DISC_FACE, y+34, FLY_Z+19, .15, 8, 3), "#31637e")
        for x in (-42, 25):
            add("frame", side+"飞轮轴承座 · 横向张紧槽", adjustable_fly_mount(x, y))
            bearing((x, y, FLY_Z), X, side+"飞轮608轴承", "fly", FLY_SHFT_R, 11, 7)
        stock_x = drive_x + (FLY_PULLEY_FACE-STOCK_PULLEY_FACE)/2
        add("flyDrive", side+"Ø8 24T 从动带轮 · DOLD 863916", stock_fly_pulley(stock_x, y))
        add("flyDrive", side+f"{FLY_BELT_LENGTH} mm 斜置同步带", fly_belt(drive_x, sign), "#394654")

    z = FLY_MOTOR_Z
    add("flyDrive", "MAD M6C12 EEE 150KV · Ø72×33.4 外形",
        cut(cylinder((-90, 0, z), X, 36, 33.4),
            *bolt_circle((-90, 0, z), X, 32, 4, 1.65, 3),
            *bolt_circle((-59.6, 0, z), X, 22, 4, 1.25, 3)), "#526c86")
    add("flyDrive", "M6C12 端面取力联轴件", cut(
        fuse(cylinder((-56.6, 0, z), X, 15, 4.6), cylinder((-52, 0, z), X, 9, 9)),
        cylinder((-52, 0, z), X, 5, 9),
        *bolt_circle((-56.6, 0, z), X, 22, 4, 1.7, 4.6),
    ))
    shaft_with_retaining_rings(-52, 0, z, 139, (-42.8, 84),
                               "flyDrive", "飞轮主动中间轴")
    for x in FLY_DRIVE_TRACKS:
        add("flyDrive", "共用主动轴 24T 主动轮", fly_pulley(x, 0, z))
    for x in (-42, 78):
        add("frame", "主动中间轴承座", shaft_block(x, 0, z))
        bearing((x, 0, z), X, "主动轴6900轴承", "flyDrive", 5, 11, 6)
    bracket = cut(box(-95, -45, FLY_BEAM_TOP, 5, 90, z+24-FLY_BEAM_TOP),
                  cylinder((-95, 0, z), X, 8, 5),
                  *bolt_circle((-95, 0, z), X, 32, 4, 2.2, 5))
    add("frame", "M6C12 固定座", bracket)
    for y in (-45, 37):
        add("frame", "M6C12 固定座侧梁", box(-95, y, FLY_BEAM_TOP-5, 53, 8, 5))
    add("frame", "主动轴前支承延伸梁", box(32, -12, 179, 53, 24, 5))
    add("frame", "主动轴前支承垫高", box(78, -12, 184, 7, 24, 5))
    add("flyDrive", "电机轴端磁铁示意", cylinder((-97.5, 0, z), X, 3, 2.5), "#e2cd70")
    add("flyDrive", "外置编码器PCB占位", box(-101, -10, z-10, 2, 20, 20), "#79b693")


def make_electronics():
    # The battery and drives sit in the compact equipment level below the cargo floor.
    battery_x = 51.5
    add("electronics", "前置电池承力底托", box(50, -44, 226, 37, 88, 2), "#547b92")
    for y in (-49, 44):
        add("frame", "电池托盘接片", box(51, y, 226, 8, 5, 2))
    for y in (-37, 33):
        add("electronics", "电池承托条", box(50, y, 228, 38, 4, 3), "#718799")
    add("electronics", "Tattu 6S 参考电池 · 225 g", box(battery_x, -37, 231, 35, 74, 45), "#755d9e")
    add("electronics", "降压、USB-CAN 与线束占位 · 45 g", box(54, -32, 279, 30, 64, 21), "#519786")

    pi_x = (-39.0, 19.0)
    pi_z = (250.5, 299.5)
    pi = cut(box(-42.5, 27, 247, 85, 1.8, 56),
             *[cylinder((x, 27, z), Y, 1.35, 1.8) for x in pi_x for z in pi_z])
    add("electronics", "Raspberry Pi 5 PCB · 46 g 预算", pi, "#459570")
    add("electronics", "Pi 接口及主动散热包络 · 24 g 预算",
        box(-39, 28.8, 255, 76, 17.2, 38), "#a4b8bd")
    for x in pi_x:
        for z in pi_z:
            add("electronics", "Pi 支柱", ring((x, 28.8, z), Y, 2.35, 1.35, 20.2), "#708e9b")
    add("electronics", "后舱底托", box(-87, -38, 243, 36, 76, 2), "#547b92")
    for y in (-49, 38):
        add("frame", "后设备托盘接片", box(-59, y, 243, 8, 11, 2))
    add("electronics", "ODrive S1 与散热占位 · 75 g", box(-76, -33, 247, 17, 66, 50), "#337c6f")
    add("electronics", "IMU 与安装板 · 5 g", box(-87, -11, 250, 9, 22, 12), "#77b895")
    make_electronic_shell()


def make_electronic_shell():
    # Side profiles rise between the two end bays to keep the flywheel zone clear.
    profile = [(-90, 231), (-82, 224), (-51, 224), (-34, 239), (34, 239),
               (51, 224), (82, 224), (90, 231), (90, SHELL_SEAM), (-90, SHELL_SEAM)]
    for side, y in (("左", -77), ("右", 74.5)):
        cover = cut(side_plate(profile, y, 2.5),
                    box(-17, y, 293.5, 34, 2.5, SHELL_SEAM-293.5),
                    *(cylinder((x, y, 298), Y, 1.7, 2.5) for x in (-56, 56)))
        add("cover", side+"电子舱外侧板 · 内凹轮组让位", cover)
        add("cover", side+"连接口遮盖",
            box(-16.7, y, 293.8, 33.4, 2.5, CABIN_BOTTOM-293.8), "#4c6878")

    # End caps meet the sides edge to edge. Slots stay on the rear face.
    front = box(87.5, -74.5, 231, 2.5, 149, SHELL_SEAM-231)
    rear = cut(box(-90, -74.5, 231, 2.5, 149, SHELL_SEAM-231),
               box(-90, -22, 231, 2.5, 44, 7),
               *(box(-90, -29, z, 2.5, 58, 2.6) for z in (248, 255, 262, 269, 276, 283)))
    for name, x, cover in (("前检修面板", 87.5, front), ("后散热检修面板", -90, rear)):
        cover = cut(cover, *(cylinder((x, y, 298), X, 1.7, 2.5) for y in (-48, 48)))
        add("cover", name, cover)


def make_cargo_body():
    corners = [(x, y) for x in (-82, 82) for y in (-70, 70)]
    floor = cut(
        box(-90, -77, CABIN_BOTTOM, *DECK_SIZE, CABIN_FLOOR-CABIN_BOTTOM),
        *[cylinder((x, y, CABIN_BOTTOM), Z, 2.75, 4) for x in (-56, 56) for y in (-40, 40)],
        *[cylinder((x, y, CABIN_BOTTOM), Z, 2.25, 4) for x, y in corners],
    )
    add("body", "与外围承力框对齐的储物底板", floor, "#6488a0")
    height = DECK_BOTTOM-CABIN_FLOOR
    for y, side in ((-77, "左"), (74.5, "右")):
        add("body", side+"储物舱外壁", box(-90, y, CABIN_FLOOR, 180, 2.5, height))
    add("body", "储物舱后壁", box(-90, -74.5, CABIN_FLOOR, 2.5, 149, height))
    for x, y in corners:
        holes = [cylinder((x, y, CABIN_FLOOR), Z, 2.25, height)]
        if x > 0:
            # 0.15 mm to the longitudinal rod bore at the blind end of the door hole.
            holes.extend(cylinder((84.4, y, z), X, 1.25, 3.1) for z in (329, 405))
        add("body", "平台支承角柱", cut(
            box(x-4, y-4, CABIN_FLOOR, 9.5 if x > 0 else 8, 8, height), *holes,
        ), "#6589a2")
    grip_level = 391
    grip = fuse(box(87.5, -12, grip_level-4, 2.5, 24, 8),
                *(cylinder((87.5, y, grip_level), X, 4, 2.5) for y in (-12, 12)))
    door = cut(box(87.5, -74.1, CABIN_FLOOR+.5, 2.5, 148.2, height-1), grip,
               *[cylinder((87.5, y, z), X, 1.7, 2.5) for y in (-70, 70) for z in (329, 405)])
    add("door", "齐平可拆储物舱门 · 内嵌指槽", door)

    deck = cut(box(-90, -77, DECK_BOTTOM, *DECK_SIZE, DECK_TOP-DECK_BOTTOM),
               *(cylinder((x, y, DECK_BOTTOM), Z, 2.25, 4) for x, y in corners))
    side_rails = [box(-84, y, DECK_TOP, 168, 2.5, 4) for y in (-77, 74.5)]
    strap_loops = [
        cut(box(x-14, y-2.5, DECK_TOP, 28, 5, 7),
            box(x-11, y-2.5, DECK_TOP, 22, 5, 4))
        for x in (-50, 50) for y in (-68, 68)
    ]
    add("deck", "打印轻载平台 · 齐平侧挡与四个顶面绑带桥",
        fuse(deck, *side_rails, *strap_loops), "#d9e0df")
    for x, y in corners:
        add("deck", "M4×130 贯通螺杆", cylinder((x, y, 301), Z, 2, 130), "#7c8b99")
        for washer_z, nut_z in ((CABIN_BOTTOM-.8, CABIN_BOTTOM-5.8), (DECK_TOP, DECK_TOP+.8)):
            add("deck", "M4 平垫 DIN 125", ring((x, y, washer_z), Z, 4.5, 2.15, .8), "#8393a1")
            hexagon = [(x+4.04*math.cos(i*TAU/6), y+4.04*math.sin(i*TAU/6), nut_z) for i in range(6)]
            add("deck", "M4 锁紧螺母 DIN 985", cut(
                prism(hexagon, (0, 0, 5)), cylinder((x, y, nut_z), Z, 2, 5),
            ), "#8393a1")


def verify_storage_access():
    # Check that a box can pass straight through the front door to the full clear volume.
    x, y, z = CLEAR_STORAGE
    space = box(-x/2, -y/2, CABIN_FLOOR+1, x/2+95, y, z)
    for group, name, shape, _ in PARTS:
        if group in {"body", "deck"} and volume(BRepAlgoAPI_Common(space, shape).Shape()) > .05:
            raise ValueError(f"Storage approach blocked by {name}")


def shape_bounds(shape):
    b = Bnd_Box()
    static(BRepBndLib, "AddOptimal", shape, b)
    return (*b.CornerMin().Coord(), *b.CornerMax().Coord())


def mesh(shape):
    BRepMesh_IncrementalMesh(shape, .22, False, .28, True)
    faces = TopExp_Explorer(shape, TopAbs_FACE)
    vertices, indices = [], []
    while faces.More():
        face = static(TopoDS, "Face", faces.Current())
        loc = TopLoc_Location()
        tri = static(BRep_Tool, "Triangulation", face, loc)
        if tri is not None:
            offset = len(vertices)//3
            t = loc.Transformation()
            for i in range(1, tri.NbNodes()+1):
                vertices.extend(round(c, 3) for c in tri.Node(i).Transformed(t).Coord())
            flip = face.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles()+1):
                a, b, c = tri.Triangle(i).Get()
                indices.extend((offset+a-1, offset+(c if flip else b)-1, offset+(b if flip else c)-1))
        faces.Next()
    if not indices:
        raise ValueError("Failed to produce mesh")
    return {"v": vertices, "i": indices}


def volume(shape):
    props = GProp_GProps()
    static(BRepGProp, "VolumeProperties", shape, props)
    return abs(props.Mass())


def check_assembly(records):
    bbs = [shape_bounds(shape) for _, _, shape, _ in records]
    hits = []
    for i, (g1, n1, a, _) in enumerate(records):
        for j in range(i+1, len(records)):
            g2, n2, b, _ = records[j]
            if any(min(bbs[i][k+3], bbs[j][k+3])-max(bbs[i][k], bbs[j][k]) < .01 for k in range(3)):
                continue
            common = BRepAlgoAPI_Common(a, b)
            if not common.IsDone():
                raise ValueError(f"Intersection check failed: {n1} / {n2}")
            v = volume(common.Shape())
            if v > .15:
                hits.append((n1, n2, round(v, 3)))
    return hits


def write_step(path, name, shape, records=()):
    document = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
    tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())

    def name_label(label, text):
        TDataStd_Name.Set_s(label, TCollection_ExtendedString(text, True))
        return label

    if records:
        root = name_label(tool.NewShape(), name)
        groups = {key: name_label(tool.NewShape(), title)
                  for key, (title, _, _) in GROUPS.items()}
        location = TopLoc_Location()
        for key, title, item, _ in records:
            label = name_label(tool.AddShape(item, False), title)
            name_label(tool.AddComponent(groups[key], label, location), title)
        for key, (title, _, _) in GROUPS.items():
            name_label(tool.AddComponent(root, groups[key], location), title)
        tool.UpdateAssemblies()
    else:
        name_label(tool.AddShape(shape, False), name)

    writer = STEPCAFControl_Writer()
    writer.SetNameMode(True)
    if not writer.Transfer(document, STEPControl_AsIs) or writer.Write(str(path)) != IFSelect_RetDone:
        raise ValueError(f"STEP write failed: {path.name}")
    imported = read_step(path)
    if any(abs(a-b) > .02 for a, b in zip(shape_bounds(imported), shape_bounds(shape))):
        raise ValueError(f"STEP round-trip dimensions changed: {path.name}")
    if abs(volume(imported)-volume(shape)) > .1:
        raise ValueError(f"STEP round-trip volume changed: {path.name}")


def main():
    vendor_model = HERE / "DD9015_supplier.step"
    if not vendor_model.is_file():
        raise FileNotFoundError("Download the DD9015 3D Model.STEP from SteadyWin. See README.md.")
    OUTPUT.mkdir(exist_ok=True)
    make_wheel()
    make_drive()
    make_frame()
    flywheel_parts = stock_flywheel_parts()
    make_flywheels(flywheel_parts)
    make_electronics()
    make_cargo_body()
    verify_storage_access()
    for _, name, shape, _ in PARTS:
        if not BRepCheck_Analyzer(shape).IsValid() or volume(shape) <= 0:
            raise ValueError(f"Invalid CAD solid: {name}")
    assembly = compound(entry[2] for entry in PARTS)
    write_step(OUTPUT/"GyroRoll_stock_inertia_R1B.step", "GyroRoll R1B", assembly, PARTS)
    disc = compound(shape for _, shape, _ in flywheel_parts)
    write_step(OUTPUT/"GyroRoll_R1B_flywheel_D100_bore8_envelope.step",
               "R1B Flywheel D100 Bore 8 - Three Spokes", disc)
    overlap = check_assembly(PARTS)
    print("Solid interferences > 0.15 mm³:", overlap)
    if overlap:
        raise ValueError("Static interference check failed")
    print_dir = OUTPUT/"print_fit_check"
    print_dir.mkdir(exist_ok=True)
    print_manifest = []
    for index, (group, name, shape, _) in enumerate(PARTS, 1):
        if (group in {"cover", "door"} or name.startswith("打印轻载平台")
                or name.endswith(("储物舱外壁", "储物舱后壁"))):
            # Export parts individually; mounting holes belong to the final fit-check print.
            xmin, ymin, zmin, xmax, ymax, zmax = shape_bounds(shape)
            sizes = (xmax-xmin, ymax-ymin, zmax-zmin)
            thickness_axis = 2 if name == "前门拉手" else min(range(len(sizes)), key=sizes.__getitem__)
            if thickness_axis != 2:
                pivot, angle = (Y, -math.pi/2) if thickness_axis == 0 else (X, math.pi/2)
                rotate = gp_Trsf()
                rotate.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(*pivot)), angle)
                shape = BRepBuilderAPI_Transform(shape, rotate, True).Shape()
                xmin, ymin, zmin, *_ = shape_bounds(shape)
            move = gp_Trsf()
            move.SetTranslation(gp_Vec(-xmin, -ymin, -zmin))
            local = BRepBuilderAPI_Transform(shape, move, True).Shape()
            BRepMesh_IncrementalMesh(local, .12)
            stl = StlAPI_Writer()
            stl.ASCIIMode = False
            file = f"{group}_{index:03d}.stl"
            if not stl.Write(local, str(print_dir/file)):
                raise ValueError(f"STL write failed: {name}")
            lo_x, lo_y, lo_z, hi_x, hi_y, hi_z = shape_bounds(local)
            size = [round(hi_x-lo_x, 2), round(hi_y-lo_y, 2), round(hi_z-lo_z, 2)]
            print_manifest.append(f"{file}\t{name}\t{size[0]} × {size[1]} × {size[2]}")
    (print_dir/"parts.tsv").write_text(
        "STL\t零件\t打印方向尺寸 mm\n"+"\n".join(print_manifest)+"\n", encoding="utf-8-sig",
    )
    mass_data = estimate(PARTS)
    baseline = json.loads((HERE/"baseline_mass.json").read_text(encoding="utf-8-sig"))
    (OUTPUT/"mass_properties.json").write_text(json.dumps(
        {"units": {"mass": "g", "position": "mm"}, "before": baseline, "after": mass_data},
        ensure_ascii=False, indent=2), encoding="utf-8")
    data = {
        "groups": {gid: {"title": name, "color": color, "explode": vector}
                   for gid, (name, color, vector) in GROUPS.items()},
        "parts": [dict(group=g, name=n, color=c, **mesh(s)) for g, n, s, c in PARTS],
        "bounds": shape_bounds(assembly),
        "wheelCenterDistance": round(WHEEL_C, 2),
        "wheelBeltLength": WHEEL_BELT_LENGTH,
        "beltWidth": BELT_WIDTH,
        "groundZ": WHEEL_Z-WHEEL_DIAMETER/2,
        "mass": {k: v for k, v in mass_data.items() if k != "rows"},
        "baselineMass": {k: v for k, v in baseline.items() if k != "rows"},
        "battery": next(row for row in mass_data["rows"] if row["part"].startswith("Tattu")),
        "flywheel": {
            "height": FLY_Z, "motorHeight": FLY_MOTOR_Z, "drop": FLY_DROP,
            "horizontalOffset": FLY_OFFSET, "beltLength": FLY_BELT_LENGTH,
            "centerDistance": FLY_DISTANCE,
        },
        "cargo": {
            "clear": CLEAR_STORAGE,
            "deck": DECK_SIZE,
            "floor": CABIN_FLOOR,
            "deckZ": DECK_TOP,
        },
    }
    template = (HERE/"viewer.html.in").read_text(encoding="utf-8")
    lib = HERE/"viewer_lib"
    three = (lib/"three.min.js").read_text(encoding="utf-8")
    license_text = (lib/"three-LICENSE.txt").read_text(encoding="utf-8")
    html = template.replace("/* THREE_BUNDLE */", "/*\n"+license_text+"\n*/\n"+three)
    html = html.replace("/* CAD_DATA */", json.dumps(data, ensure_ascii=False, separators=(",", ":")))
    (OUTPUT/"index.html").write_text(html, encoding="utf-8")
    print(f"{len(PARTS)} parts; assembly bounds {data['bounds']}; wheel center distance {WHEEL_C:.3f} mm")
    print("Mass:", data["mass"])


if __name__ == "__main__":
    main()
