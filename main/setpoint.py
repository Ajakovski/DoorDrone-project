#!/usr/bin/env python3
"""
test_setpoint.py — Phase 1 UDP setpoint verification tool
----------------------------------------------------------
Sends 12-byte UDP packets to the drone's ESP32-S3 on port 4210.
No external libraries required — uses Python stdlib only.

Usage:
    python3 test_setpoint.py <DRONE_IP>

Examples:
    python3 test_setpoint.py 192.168.1.105          # idle heartbeat
    python3 test_setpoint.py 192.168.1.105 --roll 15
    python3 test_setpoint.py 192.168.1.105 --thr 1200 --pitch -10 --yaw 30
    python3 test_setpoint.py 192.168.1.105 --hz 100  # stress test at 100 Hz

Packet format (12 bytes, big-endian):
    [0]    0xAB  magic
    [1]    0xCD  magic
    [2-3]  throttle  uint16  1000-2000 µs
    [4-5]  roll      int16   value*10  (±450 = ±45.0°)
    [6-7]  pitch     int16   value*10  (±450 = ±45.0°)
    [8-9]  yaw_rate  int16   value*10  (±3000 = ±300.0°/s)
    [10]   flags     uint8   reserved
    [11]   checksum  uint8   XOR of [0..10]
"""

import socket
import struct
import time
import sys
import argparse

DRONE_PORT = 4210
MAGIC_0    = 0xAB
MAGIC_1    = 0xCD
PACKET_SIZE = 12


def build_packet(throttle: int, roll_deg: float, pitch_deg: float,
                 yaw_rate_dps: float, flags: int = 0) -> bytes:
    """
    Build a validated 12-byte setpoint packet.
    Raises ValueError if any value is out of the accepted range.
    """
    throttle    = int(round(throttle))
    roll_raw    = int(round(roll_deg    * 10))
    pitch_raw   = int(round(pitch_deg   * 10))
    yaw_raw     = int(round(yaw_rate_dps * 10))

    if not (1000 <= throttle <= 2000):
        raise ValueError(f"Throttle {throttle} out of range [1000-2000]")
    if not (-450 <= roll_raw <= 450):
        raise ValueError(f"Roll {roll_deg}° out of range [±45°]")
    if not (-450 <= pitch_raw <= 450):
        raise ValueError(f"Pitch {pitch_deg}° out of range [±45°]")
    if not (-3000 <= yaw_raw <= 3000):
        raise ValueError(f"Yaw rate {yaw_rate_dps}°/s out of range [±300°/s]")

    # Pack header + payload (big-endian)
    # Format: BB H h h h B  →  magic0 magic1 throttle roll pitch yaw flags
    body = struct.pack('>BBHhhhB',
                       MAGIC_0, MAGIC_1,
                       throttle, roll_raw, pitch_raw, yaw_raw,
                       flags)
    assert len(body) == PACKET_SIZE - 1

    # XOR checksum over all 11 payload bytes
    checksum = 0
    for b in body:
        checksum ^= b

    return body + bytes([checksum])


def run(drone_ip: str, throttle: int, roll: float, pitch: float,
        yaw_rate: float, hz: float, duration_s: float):

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / hz
    target = (drone_ip, DRONE_PORT)

    print(f"\n{'─'*54}")
    print(f"  Phase 1 setpoint test")
    print(f"  Target  : {drone_ip}:{DRONE_PORT}")
    print(f"  Rate    : {hz:.0f} Hz  (period {period*1000:.1f} ms)")
    print(f"  Throttle: {throttle} µs")
    print(f"  Roll    : {roll:.1f}°")
    print(f"  Pitch   : {pitch:.1f}°")
    print(f"  Yaw rate: {yaw_rate:.1f} °/s")
    if duration_s > 0:
        print(f"  Duration: {duration_s:.1f} s")
    else:
        print(f"  Duration: until Ctrl+C")
    print(f"{'─'*54}\n")

    pkt = build_packet(throttle, roll, pitch, yaw_rate)
    assert len(pkt) == PACKET_SIZE, f"Packet wrong size: {len(pkt)}"

    print(f"  Packet bytes: {pkt.hex().upper()}")
    print(f"  Checksum: 0x{pkt[-1]:02X}\n")
    print("  Sending... (Ctrl+C to stop)\n")

    sent = 0
    t_start = time.monotonic()
    next_send = t_start

    try:
        while True:
            now = time.monotonic()
            elapsed = now - t_start

            if duration_s > 0 and elapsed >= duration_s:
                break

            if now >= next_send:
                sock.sendto(pkt, target)
                sent += 1
                next_send += period

                # Progress update every 2 seconds
                if sent % max(1, int(hz * 2)) == 0:
                    rate = sent / elapsed if elapsed > 0.001 else 0
                    print(f"\r  Sent {sent:6d} packets  ({rate:.1f} Hz actual)    ",
                          end='', flush=True)

            # Sleep for ~half a period to avoid busy-wait, keep timing tight
            sleep_time = next_send - time.monotonic()
            if sleep_time > 0.001:
                time.sleep(sleep_time * 0.9)

    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.monotonic() - t_start
        sock.close()
        print(f"\n\n  ── Summary ──────────────────────────")
        print(f"  Sent   : {sent} packets")
        print(f"  Elapsed: {elapsed:.2f} s")
        print(f"  Rate   : {sent/elapsed:.1f} Hz actual")
        print(f"\n  Now check the ESP32 serial monitor for:")
        print(f"    [SETPOINT] First valid packet from {drone_ip}:...")
        print(f"    [SETPOINT] PKT #100 | THR={throttle} ...")
        print(f"\n  Stop sending and wait 0.5s to see:")
        print(f"    [SETPOINT] Setpoint STALE — ... ms since last packet")
        print(f"{'─'*38}\n")


def main():
    parser = argparse.ArgumentParser(
        description="Phase 1 UDP setpoint test — sends packets to ESP32 drone"
    )
    parser.add_argument("drone_ip", help="IP address of the drone's ESP32-S3")
    parser.add_argument("--thr",   type=int,   default=1000,
                        help="Throttle µs (1000-2000, default 1000)")
    parser.add_argument("--roll",  type=float, default=0.0,
                        help="Roll target degrees (±45, default 0)")
    parser.add_argument("--pitch", type=float, default=0.0,
                        help="Pitch target degrees (±45, default 0)")
    parser.add_argument("--yaw",   type=float, default=0.0,
                        help="Yaw rate °/s (±300, default 0)")
    parser.add_argument("--hz",    type=float, default=50.0,
                        help="Send rate Hz (default 50)")
    parser.add_argument("--time",  type=float, default=0.0,
                        help="Duration in seconds, 0 = run until Ctrl+C (default)")

    args = parser.parse_args()

    try:
        run(
            drone_ip   = args.drone_ip,
            throttle   = args.thr,
            roll       = args.roll,
            pitch      = args.pitch,
            yaw_rate   = args.yaw,
            hz         = args.hz,
            duration_s = args.time,
        )
    except ValueError as e:
        print(f"\n  ERROR: {e}\n")
        sys.exit(1)


if __name__ == "__main__":
    main()