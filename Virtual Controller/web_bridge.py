#!/usr/bin/env python3
"""
web_bridge.py  —  WebSocket-to-UDP bridge for the ESP32-S3 drone
================================================================
Runs two services on your PC/laptop:

  HTTP  :8080  — serves controller.html (open on phone)
  WS    :8765  — receives JSON from browser, forwards as 12-byte UDP to drone


Safety chain:
  Browser sends at 25 Hz → bridge forwards UDP at 50 Hz only while a browser
  setpoint arrived within the last 300 ms. If the phone screen locks, the tab
  backgrounds, or the bridge dies, UDP stops. The ESP32 detects staleness at
  500 ms (SETPOINT_TIMEOUT_MS) and disarms. No firmware changes needed.

Requirements:
    pip install websockets

Usage:
    python3 web_bridge.py --drone 192.168.1.105
    python3 web_bridge.py --drone 192.168.1.105 --blynk YOUR_AUTH_TOKEN
"""

import argparse
import asyncio
import json
import logging
import os
import socket
import struct
import sys
import threading
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from typing import Optional
from urllib.error import URLError
from urllib.request import urlopen

# ── Dependency check ──────────────────────────────────────────────────────────
try:
    import websockets
    import websockets.exceptions
except ImportError:
    sys.exit(
        "ERROR: websockets not installed.\n"
        "Run:   pip install websockets\n"
    )

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-5s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

# ── UDP packet builder (matches setpoint.h exactly) ───────────────────────────
#
# Wire format (12 bytes, big-endian):
#   [0]    0xAB  magic
#   [1]    0xCD  magic
#   [2-3]  throttle   uint16  1000–2000 µs
#   [4-5]  roll       int16   ×10  →  ±450 = ±45.0°
#   [6-7]  pitch      int16   ×10  →  ±450 = ±45.0°
#   [8-9]  yaw_rate   int16   ×10  →  ±3000 = ±300.0°/s
#   [10]   flags      uint8   0
#   [11]   checksum   uint8   XOR of [0..10]

_MAGIC = (0xAB, 0xCD)


def build_packet(throttle: int, roll: float,
                 pitch: float, yaw_rate: float) -> bytes:
    t = max(1000, min(2000, int(round(throttle))))
    r = max(-450,  min(450,  int(round(roll      * 10))))
    p = max(-450,  min(450,  int(round(pitch     * 10))))
    y = max(-3000, min(3000, int(round(yaw_rate  * 10))))
    body = struct.pack(">BBHhhhB", _MAGIC[0], _MAGIC[1], t, r, p, y, 0)
    chk = 0
    for b in body:
        chk ^= b
    return body + bytes([chk])


# ── Shared state ───────────────────────────────────────────────────────────────
#
# Written by the asyncio WS handler, read by the asyncio UDP loop.
# A plain asyncio.Lock is sufficient — no threading issues here.

_lock = asyncio.Lock()
_sp = {
    "throttle":  1000,
    "roll":      0.0,
    "pitch":     0.0,
    "yaw_rate":  0.0,
    "fresh_ts":  0.0,   # monotonic time of last valid setpoint; 0 = never
    "clients":   0,
}

_blynk_token: Optional[str] = None
_FRESHNESS_S = 0.30    # bridge stops UDP if no setpoint in 300 ms
_DISARM_S    = 0.50    # ESP32 disarms if no UDP in 500 ms (SETPOINT_TIMEOUT_MS)


# ── Blynk REST (called from a daemon thread to avoid blocking asyncio) ────────
def _blynk_set_v0(value: str) -> None:
    if not _blynk_token:
        log.warning("Blynk token not set — ARM/DISARM ignored. Pass --blynk TOKEN")
        return
    url = (f"https://blynk.cloud/external/api/update"
           f"?token={_blynk_token}&V0={value}")
    try:
        urlopen(url, timeout=5)
        log.info("Blynk V0 ← %s (%s)", value, "ARM" if value == "1" else "DISARM")
    except URLError as exc:
        log.warning("Blynk API failed: %s", exc.reason)


# ── WebSocket handler ─────────────────────────────────────────────────────────
async def _ws_handler(websocket) -> None:
    peer = websocket.remote_address
    async with _lock:
        _sp["clients"] += 1
        count = _sp["clients"]
    log.info("WS  ↑  %s:%s  (total: %d)", peer[0], peer[1], count)

    try:
        async for raw in websocket:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue

            cmd = msg.get("cmd")

            if cmd == "setpoint":
                thr = max(1000,   min(2000,   int(msg["throttle"])))
                rol = max(-45.0,  min(45.0,   float(msg["roll"])))
                pit = max(-45.0,  min(45.0,   float(msg["pitch"])))
                yaw = max(-300.0, min(300.0,  float(msg["yaw_rate"])))
                async with _lock:
                    _sp["throttle"]  = thr
                    _sp["roll"]      = rol
                    _sp["pitch"]     = pit
                    _sp["yaw_rate"]  = yaw
                    _sp["fresh_ts"]  = time.monotonic()

            elif cmd == "arm":
                log.info("ARM  received from %s", peer[0])
                threading.Thread(target=_blynk_set_v0, args=("1",),
                                 daemon=True).start()

            elif cmd == "disarm":
                log.info("DISARM  received from %s", peer[0])
                async with _lock:
                    _sp["throttle"]  = 1000
                    _sp["roll"]      = 0.0
                    _sp["pitch"]     = 0.0
                    _sp["yaw_rate"]  = 0.0
                    _sp["fresh_ts"]  = 0.0   # immediately stops UDP
                threading.Thread(target=_blynk_set_v0, args=("0",),
                                 daemon=True).start()

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        async with _lock:
            _sp["clients"] = max(0, _sp["clients"] - 1)
            _sp["fresh_ts"] = 0.0    # phone disconnect → stop UDP immediately
        log.info("WS  ↓  %s  —  UDP halted, drone disarms in ≤%.0f ms if armed",
                 peer[0], _DISARM_S * 1000)


# ── 50 Hz UDP sender ──────────────────────────────────────────────────────────
async def _udp_loop(drone_ip: str, drone_port: int) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    log.info("UDP  sender → %s:%d  @ 50 Hz  (active only while browser connected)",
             drone_ip, drone_port)

    sent = errs = 0
    report_t    = time.monotonic()
    was_active  = False

    while True:
        async with _lock:
            age      = time.monotonic() - _sp["fresh_ts"]
            clients  = _sp["clients"]
            sp       = (_sp["throttle"], _sp["roll"], _sp["pitch"], _sp["yaw_rate"])

        active = clients > 0 and age < _FRESHNESS_S

        if active:
            if not was_active:
                log.info("UDP  ▶  transmitting to %s:%d", drone_ip, drone_port)
            was_active = True
            try:
                sock.sendto(build_packet(*sp), (drone_ip, drone_port))
                sent += 1
            except OSError as exc:
                errs += 1
                if errs <= 3 or errs % 50 == 0:
                    log.error("UDP  send error: %s", exc)
        else:
            if was_active:
                log.info("UDP  ■  paused  (age=%.0fms clients=%d)",
                         age * 1000, clients)
            was_active = False

        now = time.monotonic()
        if now - report_t >= 10.0:
            if sent or errs:
                log.info("UDP  stats/10s: %d sent %d errors | "
                         "THR=%d R=%.1f P=%.1f Y=%.1f",
                         sent, errs, *sp)
                sent = errs = 0
            report_t = now

        await asyncio.sleep(0.02)   # 50 Hz


# ── HTTP server — serves controller.html in its own thread ───────────────────
class _QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, *_):
        pass


def _http_thread(port: int, directory: str) -> None:
    os.chdir(directory)
    httpd = HTTPServer(("0.0.0.0", port), _QuietHandler)
    log.info("HTTP  serving '%s'  on port %d", directory, port)
    httpd.serve_forever()


# ── Main ──────────────────────────────────────────────────────────────────────
async def _amain(args: argparse.Namespace) -> None:
    global _blynk_token
    _blynk_token = args.blynk

    script_dir = str(Path(__file__).resolve().parent)

    if not (Path(script_dir) / "controller.html").exists():
        log.error("controller.html not found next to web_bridge.py — aborting.")
        sys.exit(1)

    # Resolve LAN IP for the startup banner
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        lan_ip = s.getsockname()[0]
        s.close()
    except Exception:
        lan_ip = "<YOUR_PC_IP>"

    threading.Thread(target=_http_thread, args=(args.http_port, script_dir),
                     daemon=True).start()

    log.info("=" * 66)
    log.info("  Phone URL   →  http://%s:%d/controller.html", lan_ip, args.http_port)
    log.info("  WebSocket   →  ws://%s:%d", lan_ip, args.ws_port)
    log.info("  Drone UDP   →  %s:%d", args.drone, args.drone_port)
    if args.blynk:
        log.info("  Blynk token →  %s…  (ARM/DISARM enabled)", args.blynk[:10])
    else:
        log.info("  Blynk token →  not set  (pass --blynk TOKEN to enable ARM button)")
    log.info("=" * 66)

    udp_task = asyncio.create_task(_udp_loop(args.drone, args.drone_port))

    async with websockets.serve(
        _ws_handler,
        "0.0.0.0",
        args.ws_port,
        max_size=4096,
        ping_interval=15,
        ping_timeout=8,
    ):
        log.info("WS   listening on 0.0.0.0:%d  —  Ctrl+C to stop", args.ws_port)
        try:
            await asyncio.Future()
        except asyncio.CancelledError:
            pass

    udp_task.cancel()
    try:
        await udp_task
    except asyncio.CancelledError:
        pass


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Drone web controller bridge — WebSocket → UDP",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--drone",       required=True,
                    help="IP address of the ESP32-S3 drone")
    ap.add_argument("--drone-port",  type=int, default=4210,   dest="drone_port")
    ap.add_argument("--http-port",   type=int, default=8080,   dest="http_port")
    ap.add_argument("--ws-port",     type=int, default=8765,   dest="ws_port")
    ap.add_argument("--blynk",       default=None, metavar="TOKEN",
                    help="Blynk auth token — enables ARM/DISARM from the web app")
    args = ap.parse_args()

    try:
        asyncio.run(_amain(args))
    except KeyboardInterrupt:
        log.info("Bridge stopped.")


if __name__ == "__main__":
    main()