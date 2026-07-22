"""Per-channel bandwidth counters for the gateway.

scripts/sample_resources.py runs as a separate process and has no way to
read this one's in-memory counters, so BandwidthStats periodically flushes
a rolling bytes/sec figure per channel to a small JSON file instead of a
socket/HTTP endpoint - the gateway opens no inbound port by design (see
relay-protocol.md), and this shouldn't be the thing that changes that.
"""
import json
import os
import time


class BandwidthStats:

    def __init__(self, path):
        self._path = path
        self._ctrl_out = 0
        self._ctrl_in = 0
        self._video_out = 0
        self._last_flush = time.monotonic()

    def add_control_out(self, num_bytes):
        self._ctrl_out += num_bytes

    def add_control_in(self, num_bytes):
        self._ctrl_in += num_bytes

    def add_video_out(self, num_bytes):
        self._video_out += num_bytes

    def flush(self):
        now = time.monotonic()
        elapsed = max(now - self._last_flush, 1e-6)
        payload = {
            'timestamp': time.time(),
            'ctrl_bytes_out_per_sec': self._ctrl_out / elapsed,
            'ctrl_bytes_in_per_sec': self._ctrl_in / elapsed,
            'video_bytes_out_per_sec': self._video_out / elapsed,
        }
        # Write-tmp-then-replace: a reader (sample_resources.py) polling on
        # its own schedule must never observe a partially-written file.
        tmp_path = f'{self._path}.tmp'
        with open(tmp_path, 'w') as f:
            json.dump(payload, f)
        os.replace(tmp_path, self._path)
        self._ctrl_out = 0
        self._ctrl_in = 0
        self._video_out = 0
        self._last_flush = now
