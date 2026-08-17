"""统一日志。"""
from __future__ import annotations

import logging
import sys


def setup_logger(name: str = "r1ctrl", level: int = logging.INFO) -> logging.Logger:
    logger = logging.getLogger(name)
    if logger.handlers:
        return logger
    h = logging.StreamHandler(sys.stdout)
    h.setFormatter(
        logging.Formatter(
            fmt="[%(asctime)s] %(levelname).1s %(name)s | %(message)s",
            datefmt="%H:%M:%S",
        )
    )
    logger.addHandler(h)
    logger.setLevel(level)
    logger.propagate = False
    return logger
