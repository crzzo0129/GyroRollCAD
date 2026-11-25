"""Estimate assembled mass properties in grams and millimetres."""

from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

ALUMINIUM = 0.00270
STEEL = 0.00785
RUBBER = 0.00112
PETG_DENSITY = 0.00127  # g/mm³
SUPPORT_SOLID_FRACTION = 0.85


def print_category(group, name):
    if group in ("cover", "door") or name.endswith(("储物舱外壁", "储物舱后壁")):
        return "shell"
    if (group in ("frame", "body") or name.startswith("打印轻载平台") or
            name in ("前置电池承力底托", "后舱底托", "电池承托条")):
        return "support"
    return None


def part_properties(group, name, solid):
    props = GProp_GProps()
    calc = getattr(BRepGProp, "VolumeProperties_s", None) or BRepGProp.VolumeProperties
    calc(solid, props)
    v = abs(props.Mass())
    c = props.CentreOfMass()
    category = print_category(group, name)
    if category:
        fraction = SUPPORT_SOLID_FRACTION if category == "support" else 1.0
        label = "承力打印件" if category == "support" else "薄壁打印件"
        return {
            "group": group, "part": name, "mass_g": v*PETG_DENSITY*fraction,
            "center_mm": list(c.Coord()), "volume_mm3": v, "print_category": category,
            "basis": f"PETG {PETG_DENSITY*1000:.2f} g/cm³ × {fraction:.0%} CAD 实体体积 · {label}",
        }
    # Purchased components are assigned a mass independently of their CAD envelope.
    if name.startswith("SteadyWin"):
        weight, source = 534, "厂家标称，实体体积中心近似"
    elif name.startswith("MAD M6C12"):
        weight, source = 257, "厂家标称，外形中心近似"
    elif name.startswith("电池安装") or name.startswith("Tattu"):
        weight, source = 225, "Tattu FunFly 6S 1300 mAh 厂家标称"
    elif name.startswith("Raspberry Pi 5"):
        weight, source = 46, "裸板估计"
    elif name.startswith("Pi 接口"):
        weight, source = 24, "主动散热器及安装质量预算"
    elif name.startswith("飞轮驱动与") or name.startswith("ODrive S1"):
        weight, source = 75, "S1 带端子 55 g，散热与安装预算 20 g"
    elif name.startswith("IMU"):
        weight, source = 5, "IMU 与安装件预算"
    elif name.startswith("降压"):
        weight, source = 45, "降压、USB-CAN 及随舱线束预算"
    elif name.startswith("外置编码器"):
        weight, source = 4, "编码器与引线预算"
    elif "成品惯性轮" in name:
        weight, source = v, "单侧 220 g 预算，按重建零件体积分配"
    elif any(s in name for s in ("MSP-8-F", "内圈调整垫")):
        weight, source = v*STEEL, "按钢密度估算"
    elif "相位标记" in name:
        weight, source = v*0.00122, "按塑料实体体积估算"
    elif "圆冠橡胶胎" in name or "同步带" in name or "密封圈" in name:
        weight, source = v*RUBBER, "按均质橡胶体积估算"
    elif any(s in name for s in ("轴承外圈", "轴承内圈", "贯通轮轴", "贯通螺杆", "平垫 DIN", "锁紧螺母", "止动环", "轴用挡圈", "Ø10 飞轮轴", "Ø8 飞轮轴", "飞轮主动中间轴", "传扭键", "轴端磁铁")):
        weight, source = v*STEEL, "按钢密度估算"
    else:
        weight, source = v*ALUMINIUM, "按铝合金体积估算"
    return {"group": group, "part": name, "mass_g": weight, "center_mm": list(c.Coord()), "basis": source}


def total(rows):
    mass = sum(r["mass_g"] for r in rows)
    cog = [sum(r["mass_g"]*r["center_mm"][i] for r in rows)/mass for i in range(3)] if mass else [0, 0, 0]
    return {"mass_g": mass, "center_mm": cog}


def estimate(parts, *, baseline=False):
    rows = [part_properties(g, n, s) for g, n, s, _ in parts]
    flywheel_rows = [row for row in rows if "成品惯性轮" in row["part"]]
    if flywheel_rows:
        scale = 440 / sum(row["mass_g"] for row in flywheel_rows)
        for row in flywheel_rows:
            row["mass_g"] *= scale
    rows.append(dict(group="frame", part="紧固件与零散线束预算", mass_g=65,
                     center_mm=[0, 0, 240], basis="装配预算，两版同口径"))
    if baseline:
        rows.append(dict(group="electronics", part="降压、USB-CAN 及随舱线束预算", mass_g=45,
                         center_mm=[-95, 0, 345], basis="旧电子舱内预算"))
    support = total([row for row in rows if row.get("print_category") == "support"])
    shell = total([row for row in rows if row.get("print_category") == "shell"])
    print_mass = {
        "density_g_cm3": PETG_DENSITY*1000,
        "support_solid_fraction": SUPPORT_SOLID_FRACTION,
        "support": support,
        "shell": shell,
    }
    return {**total(rows), "print_mass": print_mass, "rows": rows}
