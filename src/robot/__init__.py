"""宇树 R1 EDU 机器人客户端封装（运控 + 视频）。

设计要点:
1. 真机模式 (dry_run=False) 通过 unitree_sdk2py 调用高层运控 + 视频流
2. 模拟模式 (dry_run=True) 完全不依赖 SDK，可在无机器人环境调试视觉
3. 内部把 R1 的高层运控接口按 G1 通用接口约定封装（两者走同一套 DDS topic）
"""
from .sdk_client import R1Client, R1Mode, VideoFrame

__all__ = ["R1Client", "R1Mode", "VideoFrame"]
