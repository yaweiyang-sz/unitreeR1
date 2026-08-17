"""统一配置加载器。

用法:
    from src.config import load_config
    cfg = load_config("config.yaml")
    print(cfg["gesture"]["debounce_frames"])
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import yaml


_DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.yaml"


def load_config(path: str | os.PathLike | None = None) -> dict[str, Any]:
    """加载 YAML 配置；找不到时回退到默认 config.yaml。"""
    p = Path(path) if path else _DEFAULT_CONFIG_PATH
    if not p.exists():
        raise FileNotFoundError(f"配置文件不存在: {p}")
    with p.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def get(cfg: dict, dotted_key: str, default: Any = None) -> Any:
    """点号路径取值，例如 get(cfg, "follow.kp_yaw", 0.002)。"""
    cur: Any = cfg
    for k in dotted_key.split("."):
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur
