# [ZAV 변경] Ubuntu/WSL 공통 비동기 센서 스트림. 소켓 대기를 physics/render callback에서 분리합니다.
"""Bounded localhost transport for PointCloud2 across Windows/WSL."""
import socket
import os
import struct
import threading
from collections import deque


class PointCloudStream:
    """Bounded CDR queue; capacity=1 keeps only the latest render frame.

    IMU uses a larger queue to preserve physics samples produced within a render
    frame. A slow/disconnected consumer drops the oldest sample on overflow.
    """

    def __init__(self, port, capacity=1):
        self._condition = threading.Condition()
        self._closed = threading.Event()
        self._pending = deque(maxlen=capacity)
        self._listener = socket.socket()
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # [ZAV 변경] 기본 loopback. 별도 NAT 구성에서만 명시적으로 다른 bind 주소를 선택합니다.
        self._listener.bind((os.environ.get('PEGASUS_BIND_HOST', '127.0.0.1'), port))
        self._listener.listen(2)
        self._listener.settimeout(1)
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()
        print(f'[LiDAR] localhost TCP stream: 127.0.0.1:{port}', flush=True)

    def submit(self, payload):
        with self._condition:
            self._pending.append(payload)
            self._condition.notify_all()

    def _run(self):
        while not self._closed.is_set():
            try:
                conn, _ = self._listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with conn:
                conn.settimeout(2)
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                # Reconnection starts with current data, not a backlog from
                # before discovery. Once connected, preserve physics bursts.
                with self._condition:
                    if len(self._pending) > 1:
                        latest = self._pending[-1]
                        self._pending.clear()
                        self._pending.append(latest)
                try:
                    while not self._closed.is_set():
                        with self._condition:
                            self._condition.wait_for(
                                lambda: self._closed.is_set() or bool(self._pending),
                                timeout=1,
                            )
                            if self._closed.is_set():
                                break
                            if not self._pending:
                                continue
                            payload = self._pending.popleft()
                        conn.sendall(struct.pack('!I', len(payload)) + payload)
                except OSError:
                    pass

    def close(self):
        self._closed.set()
        with self._condition:
            self._condition.notify_all()
        self._listener.close()
        self._worker.join(timeout=3)
