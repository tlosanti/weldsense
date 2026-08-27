#!/usr/bin/env python3
"""Self-test for the WeldSense protocol: encode -> corrupt -> decode.

Run:  python3 test_protocol.py
Exercises CRC, resync after garbage/dropped bytes, and drop detection.
"""
import struct
from protocol import (StreamParser, crc16_ccitt, ImuPacket, AudioPacket,
                      ConfigPacket, StatusPacket,
                      PKT_IMU, PKT_AUDIO, PKT_STATUS, PKT_CONFIG,
                      SYNC0, SYNC1)


def frame(ptype, seq, payload):
    hdr = bytes([SYNC0, SYNC1, ptype, seq & 0xFF, seq >> 8,
                 len(payload) & 0xFF, len(payload) >> 8])
    crc = crc16_ccitt(hdr[2:] + payload)
    return hdr + payload + struct.pack("<H", crc)


def imu_frame(seq, t):
    return frame(PKT_IMU, seq, struct.pack("<Ihhhhhh", t, 1, 2, 3, 4, 5, 6))


def audio_frame(seq, t, n):
    pcm = struct.pack("<%dh" % n, *range(n))
    return frame(PKT_AUDIO, seq, struct.pack("<IH", t, n) + pcm)


def main():
    p = StreamParser()
    ok = True

    # 1) Clean round-trip
    pkts = p.feed(imu_frame(0, 1000))
    assert len(pkts) == 1 and isinstance(pkts[0], ImuPacket)
    assert pkts[0].ax == 1 and pkts[0].gz == 6, "IMU decode"
    print("PASS clean IMU round-trip")

    # 2) Audio round-trip
    pkts = p.feed(audio_frame(0, 2000, 256))
    assert len(pkts) == 1 and isinstance(pkts[0], AudioPacket)
    assert pkts[0].n == 256 and len(pkts[0].samples) == 512
    print("PASS audio round-trip")

    # 3) Garbage before a frame -> must resync and still decode
    pkts = p.feed(b"\x00\xff\xaa\x13garbage" + imu_frame(1, 1005))
    assert any(isinstance(x, ImuPacket) for x in pkts), "resync after garbage"
    print("PASS resync after garbage")

    # 4) Corrupted CRC -> dropped, no packet, crc_errors increments
    bad = bytearray(imu_frame(2, 1010)); bad[-1] ^= 0xFF
    before = p.stats.crc_errors
    pkts = p.feed(bytes(bad))
    assert p.stats.crc_errors > before, "CRC error counted"
    assert not any(isinstance(x, ImuPacket) for x in pkts), "bad CRC not emitted"
    print("PASS CRC rejection")

    # 5) Split frame across two feeds
    fr = imu_frame(3, 1015)
    pkts = p.feed(fr[:5]); assert pkts == []
    pkts = p.feed(fr[5:]); assert len(pkts) == 1
    print("PASS split-frame reassembly")

    # 6) Drop detection via sequence gap (seq 4 skipped -> jump to 6)
    p.feed(imu_frame(4, 1020))
    dropped_before = p.stats.dropped.get(PKT_IMU, 0)
    p.feed(imu_frame(6, 1030))   # seq 5 missing
    dropped_after = p.stats.dropped.get(PKT_IMU, 0)
    assert dropped_after == dropped_before + 1, "one dropped IMU detected"
    print("PASS drop detection")

    # 7) Config + status
    cfg = struct.pack("<BBHHHHHff", 1, 0, 200, 16000, 256, 4, 1000,
                      0.122e-3, 35.0e-3)
    pkts = p.feed(frame(PKT_CONFIG, 0, cfg))
    assert isinstance(pkts[0], ConfigPacket) and pkts[0].audio_fs_hz == 16000
    st = struct.pack("<IIIIB", 3000, 100, 20, 0, 1) + b"\x00\x00\x00"
    pkts = p.feed(frame(PKT_STATUS, 0, st))
    assert isinstance(pkts[0], StatusPacket) and pkts[0].imu_ok
    print("PASS config + status decode")

    print("\nALL TESTS PASSED" if ok else "FAILURES")


if __name__ == "__main__":
    main()
