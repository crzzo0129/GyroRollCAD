"""Estimate assembled mass properties in grams and millimetres."""

from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

ALUMINIUM = 0.00270
STEEL = 0.00785
RUBBER = 0.00112
PLASTIC = 0.00122


def part_properties(group, name, solid):
    props = GProp_GProps()
    calc = getattr(BRepGProp, "VolumeProperties_s", None) or BRepGProp.VolumeProperties
    calc(solid, props)
    v = abs(props.Mass())
    c = props.CentreOfMass()
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
    elif name.startswith("打印轻载平台"):
        weight, source = v*PLASTIC, "PETG 全实心质量上限"
    elif "相位标记" in name:
        weight, source = v*PLASTIC, "按塑料实体体积估算"
    elif "圆冠橡胶胎" in name or "同步带" in name or "密封圈" in name:
        weight, source = v*RUBBER, "按均质橡胶体积估算"
    elif group in ("cover", "door") or "储物舱外壁" in name or "储物舱后壁" in name:
        weight, source = v*PLASTIC, "按塑料体积估算"
    elif any(s in name for s in ("轴承外圈", "轴承内圈", "贯通轮轴", "贯通螺杆", "平垫 DIN", "锁紧螺母", "止动环", "轴用挡圈", "Ø10 飞轮轴", "Ø8 飞轮轴", "飞轮主动中间轴", "传扭键", "轴端磁铁")):
        weight, source = v*STEEL, "按钢密度估算"
    else:
        weight, source = v*ALUMINIUM, "按铝合金体积估算"
    return {"group": group, "part": name, "mass_g": weight, "center_mm": list(c.Coord()), "basis": source}


def total(rows):
    mass = sum(r["mass_g"] for r in rows)
    cog = [sum(r["mass_g"]*r["center_mm"][i] for r in rows)/mass for i in range(3)]
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
    return {**total(rows), "rows": rows}
