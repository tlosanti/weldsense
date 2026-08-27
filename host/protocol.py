"""
WeldSense binary serial protocol — decoder + CRC.

This module knows how to turn the raw USB byte stream from the XIAO into
structured packets. It is intentionally standalone (no serial, no numpy) so it
can be unit-tested and reused.

Frame layout (little-endian):
    0xAA 0x55  <type:u8>  <seq:u16>  <len:u16>  <payload:len bytes>  <crc16:u16>
CRC16-CCITT (poly 0x1021, init 0xFFFF) is computed over type+seq+len+payload.

The parser is a resynchronizing byte-oriented state machine: it tolerates
dropped/corrupted bytes by scanning for the sync word and validating CRC before
ever emitting a packet. A bad CRC never produces a packet — it just resyncs.
"""

from __future__ import annotations
import struct
from dataclasses import dataclass, field
from typing import Optional

# ---- Packet types -----------------------------------------------------------
PKT_IMU    = 0x01
PKT_AUDIO  = 0x02
PKT_STATUS = 0x03
PKT_CONFIG = 0x10

SYNC0 = 0xAA
SYNC1 = 0x55

MAX_PAYLOAD = 4096  # sanity bound; real max is the audio block (~518 bytes)


# ---- CRC16-CCITT ------------------------------------------------------------
def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


# ---- Decoded payloads -------------------------------------------------------
@dataclass
class ImuPacket:
    t_us: int
    ax: int; ay: int; az: int      # raw int16 counts
    gx: int; gy: int; gz: int


@dataclass
class AudioPacket:
    t_us: int                      # timestamp of END of block (see firmware)
    n: int                         # sample count
    samples: bytes                 # raw little-endian int16 PCM (n*2 bytes)


@dataclass
class StatusPacket:
    t_us: int
    imu_count: int
    audio_count: int
    audio_overflows: int
    imu_ok: bool


@dataclass
class ConfigPacket:
    fw_major: int
    fw_minor: int
    imu_rate_hz: int
    audio_fs_hz: int
    audio_block: int
    accel_range_g: int
    gyro_range_dps: int
    accel_scale_g_per_lsb: float
    gyro_scale_dps_per_lsb: float


@dataclass
class ParserStats:
    good: int = 0
    crc_errors: int = 0
    resyncs: int = 0
    bytes_in: int = 0
    # per-type sequence tracking -> dropped-packet counts
    last_seq: dict = field(default_factory=dict)
    dropped: dict = field(default_factory=dict)


def _decode_payload(ptype: int, seq: int, payload: bytes):
    if ptype == PKT_IMU and len(payload) == 16:
        t_us, ax, ay, az, gx, gy, gz = struct.unpack("<Ihhhhhh", payload)
        return ImuPacket(t_us, ax, ay, az, gx, gy, gz)
    if ptype == PKT_AUDIO and len(payload) >= 6:
        t_us, n = struct.unpack_from("<IH", payload, 0)
        return AudioPacket(t_us, n, payload[6:6 + n * 2])
    if ptype == PKT_STATUS and len(payload) == 20:
        t_us, imu_c, aud_c, ovf, flags = struct.unpack("<IIIIB", payload[:17])
        return StatusPacket(t_us, imu_c, aud_c, ovf, bool(flags & 0x01))
    if ptype == PKT_CONFIG and len(payload) >= 20:
        (maj, minr, imu_hz, afs, ablk, arng, grng, asc, gsc) = struct.unpack(
            "<BBHHHHHff", payload[:20])
        return ConfigPacket(maj, minr, imu_hz, afs, ablk, arng, grng, asc, gsc)
    return None


class StreamParser:
    """Feed bytes with feed(); yields decoded packet objects."""

    def __init__(self):
        self.buf = bytearray()
        self.stats = ParserStats()

    def _track_seq(self, ptype: int, seq: int):
        last = self.stats.last_seq.get(ptype)
        if last is not None:
            gap = (seq - last - 1) & 0xFFFF
            if 0 < gap < 1000:  # ignore huge gaps (likely a wrap after long idle)
                self.stats.dropped[ptype] = self.stats.dropped.get(ptype, 0) + gap
        self.stats.last_seq[ptype] = seq

    def feed(self, data: bytes):
        self.stats.bytes_in += len(data)
        self.buf.extend(data)
        out = []
        buf = self.buf
        i = 0
        n = len(buf)
        while True:
            # find sync
            if n - i < 7:
                break
            if not (buf[i] == SYNC0 and buf[i + 1] == SYNC1):
                i += 1
                self.stats.resyncs += 1
                continue
            ptype = buf[i + 2]
            seq = buf[i + 3] | (buf[i + 4] << 8)
            length = buf[i + 5] | (buf[i + 6] << 8)
            if length > MAX_PAYLOAD:
                i += 1  # bogus length -> treat sync as false positive
                continue
            frame_len = 7 + length + 2
            if n - i < frame_len:
                break  # wait for more bytes
            payload = bytes(buf[i + 7:i + 7 + length])
            rx_crc = buf[i + 7 + length] | (buf[i + 7 + length + 1] << 8)
            calc = crc16_ccitt(bytes(buf[i + 2:i + 7]))
            calc = crc16_ccitt(payload, calc)
            if calc != rx_crc:
                self.stats.crc_errors += 1
                i += 1  # bad frame; slide forward and resync
                continue
            pkt = _decode_payload(ptype, seq, payload)
            if pkt is not None:
                self._track_seq(ptype, seq)
                self.stats.good += 1
                out.append(pkt)
            i += frame_len
        # keep the unconsumed tail
        del buf[:i]
        return out
