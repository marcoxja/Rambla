"""Outbound WSS client: the robot-local half of the M4 hosted relay boundary
(CTL-009). Two independent, long-lived connections to the per-robot
Durable Object (control + video - see src/shared/contracts/relay-protocol.md)
so a saturated video channel can never head-of-line-block cmd_vel/lease
traffic; the DO enforces the other half of that guarantee by routing them to
separate sockets on the browser side too.

Runs its own asyncio event loop on the main thread; ControlPanelNode's rclpy
spin runs on a background thread (control_panel_node.spin_in_background).
Sensor/diagnostics/control_authority sinks and the camera-subscribe request
methods are wired to the node here, per-connection, mirroring how server.py
used to wire them to FastAPI's broadcast before M4 - just pointed at a
websocket send instead of a local broadcast set.
"""
import asyncio
import json
import os

import websockets

from .bandwidth_stats import BandwidthStats

PROTOCOL_VERSION = 1

RECONNECT_BACKOFF_INITIAL_S = 1.0
RECONNECT_BACKOFF_MAX_S = 10.0

STATS_WRITE_INTERVAL_S = 1.0


def _relay_config():
    base_url = os.environ.get('RAMBLA_RELAY_URL', 'ws://localhost:8787')
    robot_id = os.environ.get('RAMBLA_ROBOT_ID', 'robot-1')
    token = os.environ.get('RAMBLA_ROBOT_TOKEN', '')
    return base_url.rstrip('/'), robot_id, token


def _stats_file_path():
    return os.environ.get('RAMBLA_GATEWAY_STATS_FILE', '/tmp/rambla_gateway_stats.json')


class GatewayClient:

    def __init__(self, node):
        self._node = node
        self._base_url, self._robot_id, self._token = _relay_config()
        self._stats = BandwidthStats(_stats_file_path())

    async def run(self):
        await asyncio.gather(
            self._run_control_channel(),
            self._run_video_channel(),
            self._run_stats_writer(),
        )

    # Bandwidth measurement (M4_PLAN.md Phase 5): the sampling script
    # (scripts/sample_resources.py) is external to this process and has no
    # way to read in-memory counters, so periodically flush a rolling
    # bytes/sec figure per channel to a local file it can poll. A file, not
    # a socket/HTTP endpoint - the whole point of M4 is that the gateway
    # opens no inbound port (verification item 8).
    async def _run_stats_writer(self):
        while True:
            await asyncio.sleep(STATS_WRITE_INTERVAL_S)
            self._stats.flush()

    def _headers(self):
        return {'Authorization': f'Bearer {self._token}'}

    # --- control channel: bidirectional JSON, see relay-protocol.md's
    #     "Control-channel messages" tables ---

    async def _run_control_channel(self):
        url = f'{self._base_url}/robot/{self._robot_id}/control'
        backoff = RECONNECT_BACKOFF_INITIAL_S
        while True:
            try:
                async with websockets.connect(
                        url, additional_headers=self._headers()) as ws:
                    backoff = RECONNECT_BACKOFF_INITIAL_S
                    await self._control_session(ws)
            except (websockets.exceptions.WebSocketException, OSError) as exc:
                self._node.get_logger().warning(f'control channel disconnected: {exc}')
            # Any drop - clean or not - means we can no longer tell whether a
            # driver is still trying to control the robot. Trip the deadman
            # immediately rather than waiting out its own 0.3s timeout, so
            # /cmd_vel_teleop goes silent as fast as possible (the relay is
            # a coordination layer, not the final backstop - M4_PLAN.md).
            self._node.submit_disconnect()
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, RECONNECT_BACKOFF_MAX_S)

    async def _control_session(self, ws):
        # A fresh control session must not inherit whatever camera/telemetry
        # "wanted" state was left over from a previous connection: the DO
        # only re-sends *_subscribe on this accept if it currently has
        # attached viewers (relay-protocol.md reconnect resync), so if it
        # doesn't, there's otherwise no message that would ever turn a
        # stale "wanted=True" back off, and the gateway would keep
        # streaming to nobody indefinitely. Assume nothing until told.
        self._node.request_camera_unsubscribe()
        self._node.request_telemetry_unsubscribe()

        await ws.send(json.dumps({
            'type': 'hello',
            'robot_id': self._robot_id,
            'protocol_version': PROTOCOL_VERSION,
        }))

        loop = asyncio.get_running_loop()
        send_queue = asyncio.Queue()

        def _enqueue(message):
            loop.call_soon_threadsafe(send_queue.put_nowait, message)

        self._node.on_scan = lambda data: _enqueue({'type': 'sensor_scan', 'data': data})
        self._node.on_imu = lambda data: _enqueue({'type': 'sensor_imu', 'data': data})
        self._node.on_odom = lambda data: _enqueue({'type': 'sensor_odom', 'data': data})
        self._node.on_diagnostics = lambda data: _enqueue({'type': 'diagnostics', 'data': data})
        self._node.on_control_authority = (
            lambda mode: _enqueue({'type': 'control_authority', 'mode': mode}))

        async def _sender():
            while True:
                message = await send_queue.get()
                encoded = json.dumps(message)
                self._stats.add_control_out(len(encoded.encode('utf-8')))
                await ws.send(encoded)

        sender_task = asyncio.create_task(_sender())
        try:
            async for raw in ws:
                self._stats.add_control_in(
                    len(raw) if isinstance(raw, bytes) else len(raw.encode('utf-8')))
                await self._handle_control_message(raw)
        finally:
            sender_task.cancel()
            self._node.on_scan = None
            self._node.on_imu = None
            self._node.on_odom = None
            self._node.on_diagnostics = None
            self._node.on_control_authority = None

    async def _handle_control_message(self, raw):
        try:
            message = json.loads(raw)
        except json.JSONDecodeError:
            return
        message_type = message.get('type')
        if message_type == 'cmd_vel':
            self._node.submit_drive_command(
                float(message.get('linear', 0.0)), float(message.get('angular', 0.0)))
        elif message_type == 'video_subscribe':
            self._node.request_camera_subscribe()
        elif message_type == 'video_unsubscribe':
            self._node.request_camera_unsubscribe()
        elif message_type == 'telemetry_subscribe':
            self._node.request_telemetry_subscribe()
        elif message_type == 'telemetry_unsubscribe':
            self._node.request_telemetry_unsubscribe()

    # --- video channel: gateway -> DO only, pure binary frames, no
    #     envelope - see relay-protocol.md's "Video-channel framing" ---

    async def _run_video_channel(self):
        url = f'{self._base_url}/robot/{self._robot_id}/video'
        backoff = RECONNECT_BACKOFF_INITIAL_S
        while True:
            try:
                async with websockets.connect(
                        url, additional_headers=self._headers()) as ws:
                    backoff = RECONNECT_BACKOFF_INITIAL_S
                    await self._video_session(ws)
            except (websockets.exceptions.WebSocketException, OSError) as exc:
                self._node.get_logger().warning(f'video channel disconnected: {exc}')
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, RECONNECT_BACKOFF_MAX_S)

    async def _video_session(self, ws):
        loop = asyncio.get_running_loop()
        # maxsize=1: only the newest frame matters if the socket is
        # temporarily slower than the camera's 15Hz - drop the stale one
        # rather than let unsent frames pile up, matching the DO's own
        # "byte-fanout, never buffer" behavior on the other end.
        frame_queue = asyncio.Queue(maxsize=1)

        def _enqueue_frame(jpeg_bytes):
            def _put():
                try:
                    frame_queue.put_nowait(jpeg_bytes)
                except asyncio.QueueFull:
                    try:
                        frame_queue.get_nowait()
                    except asyncio.QueueEmpty:
                        pass
                    frame_queue.put_nowait(jpeg_bytes)
            loop.call_soon_threadsafe(_put)

        self._node.on_camera_frame = _enqueue_frame
        try:
            while True:
                jpeg_bytes = await frame_queue.get()
                self._stats.add_video_out(len(jpeg_bytes))
                await ws.send(jpeg_bytes)
        finally:
            self._node.on_camera_frame = None


def run_gateway(node):
    asyncio.run(GatewayClient(node).run())
