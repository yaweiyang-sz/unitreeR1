"""在画面上叠加 UI: 手势标签、跟随框、telemetry、手指状态条、手势图例。"""
from __future__ import annotations

from typing import Optional

import cv2
import numpy as np

from .hand_gesture import Gesture, GestureResult
from .body_follow import FollowTarget


_GESTURE_COLORS = {
    Gesture.UNKNOWN: (200, 200, 200),
    Gesture.STOP: (0, 0, 255),
    Gesture.FORWARD: (0, 255, 0),
    Gesture.BACKWARD: (0, 165, 255),
    Gesture.LEFT: (255, 100, 0),
    Gesture.RIGHT: (255, 100, 0),
}


# 手势图例: 5 种动作 -> (emoji, 描述, 颜色)
_GESTURE_LEGEND = [
    (Gesture.STOP,     "✋",  "STOP       (停下/张开手掌不动)",       (0, 0, 255)),
    (Gesture.FORWARD,  "☝️ ", "FORWARD    (前进/食指伸出)",          (0, 255, 0)),
    (Gesture.BACKWARD, "✊",  "BACKWARD   (后退/握拳)",               (0, 165, 255)),
    (Gesture.LEFT,     "✋←", "LEFT       (张开+向左挥)",            (255, 100, 0)),
    (Gesture.RIGHT,    "✋→", "RIGHT      (张开+向右挥)",            (255, 100, 0)),
]


def draw_gesture_label(
    frame: np.ndarray,
    result: Optional[GestureResult],
    stable: Optional[Gesture] = None,
) -> np.ndarray:
    """左上角画当前手势名 + 手 bounding box。"""
    if result is None or result.hand_bbox is None:
        return frame
    x, y, w, h = result.hand_bbox
    g = stable or result.gesture
    color = _GESTURE_COLORS.get(g, (255, 255, 255))
    cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
    label = f"{result.handedness} {g.value}  conf={result.confidence:.2f}  {result.inference_ms:.0f}ms"
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    cv2.rectangle(frame, (x, max(0, y - th - 8)), (x + tw + 8, y), color, -1)
    cv2.putText(frame, label, (x + 4, y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return frame


def draw_hand_state(
    frame: np.ndarray,
    result: Optional[GestureResult],
) -> np.ndarray:
    """左下角画 5 根手指状态条 (绿=伸直, 红=弯曲) + ratio 数值 + 手腕速度。"""
    if result is None or result.fingers_state is None or result.fingers_ratio is None:
        return frame

    # 5 根手指, 画成 5 行
    names = ["thumb", "index", "middle", "ring", "pinky"]
    zh = {"thumb": "拇  ", "index": "食  ", "middle": "中  ", "ring": "无  ", "pinky": "小  "}

    line_h = 22
    pad = 6
    bar_w = 100
    box_w = 240
    box_h = line_h * 6 + 2 * pad   # 5 行手指 + 1 行手腕速度
    x0, y0 = 10, frame.shape[0] - box_h - 10

    cv2.rectangle(frame, (x0, y0), (x0 + box_w, y0 + box_h), (0, 0, 0), -1)
    cv2.rectangle(frame, (x0, y0), (x0 + box_w, y0 + box_h), (255, 255, 255), 1)

    for i, name in enumerate(names):
        ext = result.fingers_state.get(name, False)
        ratio = result.fingers_ratio.get(name, 0.0)
        y = y0 + pad + line_h * (i + 1) - 6
        # 名字
        cv2.putText(frame, zh[name], (x0 + pad, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        # 状态色条
        bar_x = x0 + pad + 50
        color = (0, 255, 0) if ext else (0, 0, 255)
        # 把 ratio 映射到条长 (范围 0.5 ~ 2.5)
        norm = max(0, min(1, (ratio - 0.5) / 2.0))
        bw = int(bar_w * norm)
        cv2.rectangle(frame, (bar_x, y - 12), (bar_x + bw, y - 2), color, -1)
        cv2.rectangle(frame, (bar_x, y - 12), (bar_x + bar_w, y - 2), (100, 100, 100), 1)
        # ratio 数值
        cv2.putText(frame, f"{ratio:.2f}", (bar_x + bar_w + 6, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    # 手腕水平速度 (最后一行)
    vx = result.wrist_vx
    y = y0 + pad + line_h * 6 - 6
    cv2.putText(frame, "手速", (x0 + pad, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    bar_x = x0 + pad + 50
    # 把 vx 范围 [-400, +400] 映射到条长
    norm = max(0, min(1, (abs(vx)) / 400.0))
    bw = int(bar_w * norm)
    # 颜色: 左=蓝, 右=橙, 静止=灰
    if vx > 150:
        color = (0, 165, 255)   # orange
        arrow = "→"
    elif vx < -150:
        color = (255, 100, 0)   # blue
        arrow = "←"
    else:
        color = (100, 100, 100)
        arrow = "·"
    if vx < 0:
        # 向左: 从右边向左画
        cv2.rectangle(frame, (bar_x + bar_w - bw, y - 12), (bar_x + bar_w, y - 2), color, -1)
    else:
        cv2.rectangle(frame, (bar_x, y - 12), (bar_x + bw, y - 2), color, -1)
    cv2.rectangle(frame, (bar_x, y - 12), (bar_x + bar_w, y - 2), (100, 100, 100), 1)
    # 中心标记线
    mid = bar_x + bar_w // 2
    cv2.line(frame, (mid, y - 12), (mid, y - 2), (255, 255, 255), 1)
    cv2.putText(frame, f"{arrow}{vx:+.0f}", (bar_x + bar_w + 6, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
    return frame


def draw_gesture_legend(frame: np.ndarray) -> np.ndarray:
    """右下角画 5 种手势的永久图例: emoji + 名称 + 对应动作。"""
    h, w = frame.shape[:2]
    line_h = 22
    pad = 8
    box_w = 270
    box_h = line_h * len(_GESTURE_LEGEND) + 2 * pad
    x0, y0 = w - box_w - 10, 10
    cv2.rectangle(frame, (x0, y0), (x0 + box_w, y0 + box_h), (0, 0, 0), -1)
    cv2.rectangle(frame, (x0, y0), (x0 + box_w, y0 + box_h), (255, 255, 255), 1)
    cv2.putText(frame, "GESTURE  ->  ACTION", (x0 + pad, y0 + pad + 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    for i, (g, emoji, desc, color) in enumerate(_GESTURE_LEGEND):
        y = y0 + pad + 14 + line_h * (i + 1)
        cv2.putText(frame, emoji, (x0 + pad, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        cv2.putText(frame, desc, (x0 + pad + 40, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return frame


def draw_follow_target(
    frame: np.ndarray,
    target: Optional[FollowTarget],
    target_center_x: int,
    target_bbox_area: int,
) -> np.ndarray:
    if target is None:
        return frame
    x, y, w, h = target.bbox
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    # 中心参考线
    cv2.line(frame, (target_center_x, 0), (target_center_x, frame.shape[0]), (0, 200, 0), 1, cv2.LINE_AA)
    cv2.line(frame, (0, frame.shape[0] // 2), (frame.shape[1], frame.shape[0] // 2), (0, 200, 0), 1, cv2.LINE_AA)
    label = f"target area={int(target.area)}  want={target_bbox_area}"
    cv2.putText(frame, label, (x, max(0, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return frame


def draw_telemetry(
    frame: np.ndarray,
    fps: float,
    state_text: str,
    extra: dict[str, str] | None = None,
) -> np.ndarray:
    """底部中央画 FPS / 状态 / 自定义指标。"""
    h, w = frame.shape[:2]
    lines = [f"FPS: {fps:.1f}", f"State: {state_text}"]
    if extra:
        for k, v in extra.items():
            lines.append(f"{k}: {v}")
    line_h = 22
    pad = 6
    box_w = max(cv2.getTextSize(l, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0] for l in lines) + 2 * pad
    box_h = line_h * len(lines) + 2 * pad
    # 居中靠下 (避开左右两侧的 panel)
    x0 = (w - box_w) // 2
    y0 = h - box_h - 10
    cv2.rectangle(frame, (x0, y0), (x0 + box_w, y0 + box_h), (0, 0, 0), -1)
    cv2.rectangle(frame, (x0, y0), (x0 + box_w, y0 + box_h), (255, 255, 255), 1)
    for i, l in enumerate(lines):
        cv2.putText(frame, l, (x0 + pad, y0 + pad + line_h * (i + 1) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return frame
