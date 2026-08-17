"""手势判定逻辑自测 v2.2。

v2.2 调整:
- BACKWARD 阈值 0.9 -> 1.2, 不依赖拇指 (握拳时拇指可能稍微伸)
- FORWARD 阈值保持 1.4 (食指伸出, 中/无/小弯曲)
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from src.vision.hand_gesture import _fingers_state, _classify_gesture, Gesture


def mock_21(finger_states: dict[str, str]) -> np.ndarray:
    """构造 21 关键点 mock 数据。"""
    pts = np.zeros((21, 2), dtype=np.float32)
    pts[0] = (320, 400)
    fingers_layout = [
        ("thumb",  1, 2, 3, 4,   (270, 380), (250, 360), (240, 340), (230, 320)),
        ("index",  5, 6, 7, 8,   (305, 320), (305, 260), (305, 220), (305, 180)),
        ("middle", 9, 10, 11, 12, (320, 315), (320, 245), (320, 200), (320, 160)),
        ("ring",   13, 14, 15, 16, (335, 320), (335, 260), (335, 220), (335, 180)),
        ("pinky",  17, 18, 19, 20, (350, 325), (355, 280), (358, 245), (360, 210)),
    ]
    for name, mcp_i, pip_i, dip_i, tip_i, mcp, pip, dip, tip in fingers_layout:
        if finger_states.get(name) == "extended":
            pts[mcp_i] = mcp; pts[pip_i] = pip; pts[dip_i] = dip; pts[tip_i] = tip
        else:
            pts[mcp_i] = mcp; pts[pip_i] = pip
            mid_x = (mcp[0] + pip[0]) / 2; mid_y = (mcp[1] + pip[1]) / 2
            pts[dip_i] = (mid_x + 10, mid_y - 5)
            pts[tip_i] = (mcp[0] + 15, mcp[1] - 10)
    return pts


def mock_21_with_ratio(hand_layout: str) -> np.ndarray:
    """根据 'hand_layout' 字符串构造 21 关键点, 用于模拟更真实的 ratio.

    格式: 5 个字符, 每个字符表示该手指的"伸直程度"
        'E' = 完全伸直 (ratio ~2.0)
        'B' = 完全弯曲 (ratio ~0.5)
        'H' = 半伸 (ratio ~1.0)  # 模拟用户握拳不紧
        'M' = 中等 (ratio ~1.5) # 模拟伸出不到位
        '?' = 任意 (不关心, 测其它手指的判定)
    """
    pts = np.zeros((21, 2), dtype=np.float32)
    pts[0] = (320, 400)
    finger_def = [
        # (name, mcp_i, pip_i, dip_i, tip_i, mcp, pip, dip, tip)
        ("thumb",  1, 2, 3, 4,   (270, 380), (250, 360), (240, 340), (230, 320)),
        ("index",  5, 6, 7, 8,   (305, 320), (305, 260), (305, 220), (305, 180)),
        ("middle", 9, 10, 11, 12, (320, 315), (320, 245), (320, 200), (320, 160)),
        ("ring",   13, 14, 15, 16, (335, 320), (335, 260), (335, 220), (335, 180)),
        ("pinky",  17, 18, 19, 20, (350, 325), (355, 280), (358, 245), (360, 210)),
    ]
    char_to_extent = {
        'E': 1.0,   # 完全伸直
        'M': 0.75,  # 中等
        'H': 0.5,   # 半伸
        'B': 0.2,   # 完全弯
    }
    for c, (name, mcp_i, pip_i, dip_i, tip_i, mcp, pip, dip, tip) in zip(hand_layout, finger_def):
        if c == '?':
            # 任意: 默认伸直 (避免被基础测试的指纹判定干扰)
            pts[mcp_i] = mcp; pts[pip_i] = pip; pts[dip_i] = dip; pts[tip_i] = tip
            continue
        extent = char_to_extent[c]
        if extent >= 0.9:
            pts[mcp_i] = mcp; pts[pip_i] = pip; pts[dip_i] = dip; pts[tip_i] = tip
        else:
            # 把 tip 插值到 mcp 旁边 (extent=0 时 tip 几乎在 mcp)
            pts[mcp_i] = mcp; pts[pip_i] = pip
            tx = mcp[0] + (tip[0] - mcp[0]) * extent
            ty = mcp[1] + (tip[1] - mcp[1]) * extent
            dx = (pip[0] - mcp[0]) * extent
            dy = (pip[1] - mcp[1]) * extent
            pts[dip_i] = (mcp[0] + dx * 1.5, mcp[1] + dy * 1.5)
            pts[tip_i] = (tx, ty)
    return pts


# 形状判定 - 传统 mock (完整伸直/弯曲)
SHAPE_CASES = [
    ("BACKWARD_strict",   {"thumb": "bent",     "index": "bent",     "middle": "bent",     "ring": "bent",     "pinky": "bent"},     Gesture.BACKWARD, 0.0),
    ("FORWARD_strict",    {"thumb": "bent",     "index": "extended", "middle": "bent",     "ring": "bent",     "pinky": "bent"},     Gesture.FORWARD, 0.0),
    ("FORWARD_V",         {"thumb": "bent",     "index": "extended", "middle": "extended", "ring": "bent",     "pinky": "bent"},     Gesture.UNKNOWN, 0.0),
]

# 真实场景: 半握 + 拇指状态变化
# 字符: [thumb, index, middle, ring, pinky]  E=全伸 M=中等 H=半伸 B=全弯 ?=任意
REAL_CASES = [
    # BACKWARD: 4 指弯曲, 拇指任意
    ("back_4bent_thumb_B",       "BBBBB", Gesture.BACKWARD, 0.0),  # 5 指全弯
    ("back_4bent_thumb_M",       "MBBBB", Gesture.BACKWARD, 0.0),  # 拇指中等, 4 指全弯
    ("back_4bent_thumb_H",       "HBBBB", Gesture.BACKWARD, 0.0),  # 拇指半伸, 4 指全弯 (v2.2 新支持)
    # 半握场景: 4 指稍微弯, 不完全握紧
    ("back_loose_4half",         "MHHHH", Gesture.BACKWARD, 0.0),  # 拇指中等, 4 指半伸
    ("back_loose_4half_pinky",   "MHHHB", Gesture.BACKWARD, 0.0),  # 半握 + 小指全弯
    ("fwd_3half",                "BMHHH", Gesture.FORWARD, 0.0),   # 食指伸出 1.75, 其他半弯: FORWARD (宽松判定)
    # FORWARD: 食指伸出, 其他 3 指弯曲, 拇指任意
    ("fwd_index_E_thumb_B",      "BEBBB", Gesture.FORWARD, 0.0),   # 标准前推
    ("fwd_index_E_thumb_E",      "EEBBB", Gesture.FORWARD, 0.0),   # 拇指也伸, 但不影响 FORWARD
    ("fwd_index_M_thumb_B",      "BMBBB", Gesture.FORWARD, 0.0),   # 食指中等 (ratio ~1.5 > 1.4)
    # 激活态: 5 指全伸
    ("active_5ext",              "EEEEE", Gesture.STOP, 0.0),
    ("active_5ext_left",         "EEEEE", Gesture.LEFT, -300.0),
    ("active_5ext_right",        "EEEEE", Gesture.RIGHT, 300.0),
    # 模糊: 半伸 + 不动
    ("unclear_3ext_2bent",       "EEEBB", Gesture.UNKNOWN, 0.0),   # 拇指食指伸, 其他弯 -> 不在 4 种 case 里
]


def main() -> int:
    ok = 0; fail = 0

    print("=== 形状判定 (严格 mock) ===")
    for name, states, expected, vx in SHAPE_CASES:
        pts = mock_21(states)
        f, r = _fingers_state(pts, "Right")
        g, c = _classify_gesture(f, r, vx, 150.0)
        mark = "OK" if g == expected else "FAIL"
        ok += 1 if g == expected else 0
        fail += 1 if g != expected else 0
        print(f"  [{mark}] {name:22s} -> {g.value:10s} (conf={c:.2f})  expected={expected.value}")

    print("\n=== 真实场景 (半握 + 拇指变化) ===")
    for name, layout, expected, vx in REAL_CASES:
        pts = mock_21_with_ratio(layout)
        f, r = _fingers_state(pts, "Right")
        g, c = _classify_gesture(f, r, vx, 150.0)
        mark = "OK" if g == expected else "FAIL"
        ok += 1 if g == expected else 0
        fail += 1 if g != expected else 0
        ratios_str = " ".join(f"{k}={r.get(k, 0):.2f}" for k in ["thumb", "index", "middle", "ring", "pinky"])
        print(f"  [{mark}] {name:26s} {layout} vx={vx:+.0f} -> {g.value:10s}  {ratios_str}")

    print(f"\n=== 通过 {ok}/{ok + fail} ===")
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
