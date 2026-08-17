"""视野显示: 同时支持 OpenCV 窗口和 HTTP MJPEG 流。

用法:
    viewer = Viewer(web_port=8080, opencv_window=True)
    viewer.start()
    while True:
        frame = ...  # BGR np.ndarray
        viewer.update(frame)   # 非阻塞
        if cv2.waitKey(1) == 27: break
    viewer.stop()
"""
from __future__ import annotations

import threading
import time
from typing import Optional

import cv2
import numpy as np

from ..logger import setup_logger

log = setup_logger("r1.viewer")


class _WebStreamer:
    """轻量级 MJPEG 推送: 一个后台 HTTP 服务器。"""

    def __init__(self, port: int = 8080):
        self.port = port
        self._frame: Optional[bytes] = None
        self._lock = threading.Lock()
        self._server_thread: Optional[threading.Thread] = None
        self._httpd = None
        self._running = False

    def update(self, frame_bgr: np.ndarray, quality: int = 70) -> None:
        if frame_bgr is None:
            return
        ok, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            return
        with self._lock:
            self._frame = buf.tobytes()

    def start(self) -> None:
        if self._running:
            return
        try:
            from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
        except ImportError as e:
            log.error(f"无法启动 Web 流（HTTP 服务不可用）: {e}")
            return
        streamer = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):  # noqa: N802
                if self.path == "/":
                    body = (
                        b"<!doctype html><html><head><meta charset='utf-8'>"
                        b"<title>R1 View</title><style>body{margin:0;background:#111;}"
                        b"img{width:100%;display:block;}</style></head>"
                        b"<body><img src='/stream'></body></html>"
                    )
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path == "/stream":
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    while True:
                        with streamer._lock:
                            f = streamer._frame
                        if f is None:
                            time.sleep(0.05)
                            continue
                        try:
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(f"Content-Length: {len(f)}\r\n\r\n".encode())
                            self.wfile.write(f)
                            self.wfile.write(b"\r\n")
                        except (BrokenPipeError, ConnectionResetError):
                            break
                else:
                    self.send_response(404)
                    self.end_headers()

            def log_message(self, *_):  # 静音默认访问日志
                return

        try:
            self._httpd = ThreadingHTTPServer(("0.0.0.0", self.port), Handler)
        except OSError as e:
            log.error(f"Web 流启动失败 (端口 {self.port} 占用?): {e}")
            return
        self._server_thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._server_thread.start()
        self._running = True
        log.info(f"Web 流已启动: http://<robot-ip>:{self.port}/")

    def stop(self) -> None:
        if not self._running:
            return
        try:
            if self._httpd:
                self._httpd.shutdown()
                self._httpd.server_close()
        except Exception:  # noqa: BLE001
            pass
        self._running = False


class Viewer:
    """统一接口: 本地 OpenCV 窗口 + 可选 Web 流。"""

    def __init__(self, opencv_window: bool = True, web_port: int = 8080):
        self.opencv_window = opencv_window
        self.web = _WebStreamer(port=web_port) if web_port else None
        self._last_window_update = 0.0

    def start(self) -> None:
        if self.web:
            self.web.start()

    def update(self, frame_bgr: np.ndarray) -> None:
        if frame_bgr is None:
            return
        if self.opencv_window:
            try:
                cv2.imshow("R1 View", frame_bgr)
            except cv2.error as e:
                log.debug(f"cv2.imshow 失败（无 GUI?）: {e}")
        if self.web:
            self.web.update(frame_bgr)

    def poll_quit(self) -> bool:
        """非阻塞检测用户按 ESC 或 'q'。"""
        if not self.opencv_window:
            return False
        k = cv2.waitKey(1) & 0xFF
        return k in (27, ord("q"))

    def stop(self) -> None:
        if self.opencv_window:
            try:
                cv2.destroyAllWindows()
            except Exception:  # noqa: BLE001
                pass
        if self.web:
            self.web.stop()
