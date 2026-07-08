"""FastAPI app wiring the browser (joystick input, sensor/diagnostic
readouts, camera feed) to a ControlPanelNode running rclpy on a background
thread. One shared /ws connection multiplexes cmd_vel (browser->server)
and sensor/diagnostics (server->browser) messages by a "type" field, to
keep this a single process/single port dev tool - see the plan's rationale
for not using rosbridge_suite or per-tab connections.
"""
import asyncio
import json
import os
import time

import uvicorn
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
from fastapi.staticfiles import StaticFiles

HOST = '0.0.0.0'  # noqa: S104 - intentional: CTL-008 requires reachability
# from the dev's home network, not just the VM's local console. No
# TLS/auth here by design - deferred to CTL-009, out of scope for this
# home-network-only dev tool slice.
PORT = 8000

MJPEG_BOUNDARY = 'frame'
CAMERA_STREAM_FPS = 15


def create_app(node):
    app = FastAPI()
    connections = set()

    async def _broadcast(message):
        dead = set()
        for ws in connections:
            try:
                await ws.send_text(json.dumps(message))
            except Exception:
                dead.add(ws)
        connections.difference_update(dead)

    def _make_sink(loop, msg_type):
        def sink(data):
            loop.call_soon_threadsafe(
                asyncio.create_task, _broadcast({'type': msg_type, 'data': data}))
        return sink

    @app.on_event('startup')
    async def _wire_node_sinks():
        loop = asyncio.get_running_loop()
        node.on_scan = _make_sink(loop, 'sensor_scan')
        node.on_imu = _make_sink(loop, 'sensor_imu')
        node.on_odom = _make_sink(loop, 'sensor_odom')
        node.on_diagnostics = _make_sink(loop, 'diagnostics')

    @app.websocket('/ws')
    async def ws_endpoint(websocket: WebSocket):
        await websocket.accept()
        connections.add(websocket)
        try:
            while True:
                raw = await websocket.receive_text()
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                if msg.get('type') == 'cmd_vel':
                    node.submit_drive_command(
                        float(msg.get('linear', 0.0)), float(msg.get('angular', 0.0)))
        except WebSocketDisconnect:
            pass
        finally:
            connections.discard(websocket)
            node.submit_disconnect()

    @app.get('/camera/stream')
    async def camera_stream():
        async def frame_generator():
            period = 1.0 / CAMERA_STREAM_FPS
            while True:
                jpeg = node.latest_camera_jpeg()
                if jpeg is not None:
                    yield (
                        b'--' + MJPEG_BOUNDARY.encode() + b'\r\n'
                        b'Content-Type: image/jpeg\r\n'
                        b'Content-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n'
                        + jpeg + b'\r\n'
                    )
                await asyncio.sleep(period)
        return StreamingResponse(
            frame_generator(),
            media_type=f'multipart/x-mixed-replace; boundary={MJPEG_BOUNDARY}',
        )

    static_dir = os.path.join(
        get_package_share_directory('rambla_control_panel'), 'static')
    app.mount('/', StaticFiles(directory=static_dir, html=True), name='static')

    return app


def run(node):
    app = create_app(node)
    uvicorn.run(app, host=HOST, port=PORT, log_level='info')
