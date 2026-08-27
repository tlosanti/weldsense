#!/usr/bin/env python3
"""
WeldSense host — acquisition, recording, analysis, and dashboard server.

RESPONSIBILITIES (all the "smart" stuff lives here, NOT on the XIAO):
  * Open the USB serial port and read the framed binary stream.
  * Decode packets, detect dropped/corrupted frames, resync.
  * Calibrate gyro bias on the host.
  * Compute pitch/roll (gravity + complementary filter) and relative yaw.
  * Compute audio features (dominant freq, RMS, spectral centroid) for LIVE view.
  * Record ALL RAW DATA to disk (imu.csv, audio.wav, audio_blocks.csv,
    metadata.json) so analysis can be redone later without re-welding.
  * Serve a local browser dashboard that only VISUALIZES host state.

Acquisition + recording run in their own thread and are INDEPENDENT of the
dashboard. Closing the browser does not stop recording.

Usage:
    python3 weldsense_host.py                 # auto-detect port, serve dashboard
    python3 weldsense_host.py --port /dev/cu.usbmodemXXXX
    python3 weldsense_host.py --list          # list serial ports and exit
"""

from __future__ import annotations
import argparse
import json
import math
import os
import sys
import threading
import time
import wave
import webbrowser
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import serial
import serial.tools.list_ports

from protocol import (StreamParser, ImuPacket, AudioPacket, StatusPacket,
                      ConfigPacket)

SOFTWARE_VERSION = "weldsense-host 1.0.0"
HTTP_HOST = "127.0.0.1"
HTTP_PORT = 8765
HERE = os.path.dirname(os.path.abspath(__file__))
RECORDINGS_DIR = os.path.abspath(os.path.join(HERE, "..", "recordings"))

# ---- Live audio analysis settings (host-side; does not affect recording) ----
FFT_WINDOW = 2048          # samples used for the live spectrum
FFT_HOP_MIN_SAMPLES = 256  # recompute after roughly each audio block

# ---- Fusion settings --------------------------------------------------------
COMP_ALPHA = 0.98          # complementary filter weight on the gyro
GYRO_CAL_SAMPLES = 400     # ~2 s at 200 Hz for startup bias estimate


# =============================================================================
# Orientation fusion (host side)
# =============================================================================
class Fusion:
    """Gravity-based pitch/roll with a complementary filter; integrated yaw.

    Sign/axis conventions (XIAO flat on table, USB pointing away):
        roll  = rotation about X  (accel: atan2(ay, az))
        pitch = rotation about Y  (accel: atan2(-ax, hypot(ay, az)))
        yaw   = rotation about Z  (gyro integration only -> DRIFTS, no mag)
    These match the user's verified flat readings (pitch ~+0.5, roll ~-2.1).
    """

    def __init__(self):
        self.gyro_bias = np.zeros(3)      # dps, [gx, gy, gz]
        self._cal_buf = []
        self.calibrated = False
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self._last_t_us = None

    def reset_calibration(self):
        self._cal_buf = []
        self.calibrated = False

    @staticmethod
    def _dt_us(prev, now):
        # uint32 micros() wraps at 2^32; handle safely.
        return ((now - prev) & 0xFFFFFFFF) / 1e6

    def accel_angles(self, ax, ay, az):
        roll = math.degrees(math.atan2(ay, az))
        pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
        return pitch, roll

    def update(self, t_us, ax, ay, az, gx, gy, gz):
        """Inputs already in physical units: accel in g, gyro in dps."""
        # ---- Startup bias calibration (assumes device is still at boot) ----
        if not self.calibrated:
            self._cal_buf.append((gx, gy, gz))
            if len(self._cal_buf) >= GYRO_CAL_SAMPLES:
                arr = np.array(self._cal_buf)
                self.gyro_bias = arr.mean(axis=0)
                self.calibrated = True
                # seed orientation from gravity
                self.pitch, self.roll = self.accel_angles(ax, ay, az)
                self.yaw = 0.0
            self._last_t_us = t_us
            return

        dt = self._dt_us(self._last_t_us, t_us) if self._last_t_us is not None else 0.005
        self._last_t_us = t_us
        if dt <= 0 or dt > 0.5:
            dt = 0.005  # guard against stalls/wrap glitches

        gxb = gx - self.gyro_bias[0]
        gyb = gy - self.gyro_bias[1]
        gzb = gz - self.gyro_bias[2]

        pitch_acc, roll_acc = self.accel_angles(ax, ay, az)

        # Complementary filter: gyro for fast motion, accel for long-term truth.
        self.roll = COMP_ALPHA * (self.roll + gxb * dt) + (1 - COMP_ALPHA) * roll_acc
        self.pitch = COMP_ALPHA * (self.pitch + gyb * dt) + (1 - COMP_ALPHA) * pitch_acc

        # Yaw: no absolute reference -> integrate and let the UI flag it.
        self.yaw_rate = gzb
        self.yaw += gzb * dt


# =============================================================================
# Recorder — writes raw data to disk, independent of the dashboard
# =============================================================================
class Recorder:
    def __init__(self, session_dir, audio_fs, meta):
        self.dir = session_dir
        os.makedirs(session_dir, exist_ok=True)
        self.audio_fs = audio_fs
        self._lock = threading.Lock()
        self.imu_rows = 0
        self.audio_samples = 0
        self.audio_blocks = 0

        self._imu = open(os.path.join(session_dir, "imu.csv"), "w", buffering=1)
        self._imu.write("t_us,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,"
                        "pitch_deg,roll_deg,yaw_deg,yaw_rate_dps\n")

        self._blocks = open(os.path.join(session_dir, "audio_blocks.csv"), "w",
                            buffering=1)
        self._blocks.write("block_index,t_us,n_samples,sample_offset\n")

        self._wav = wave.open(os.path.join(session_dir, "audio.wav"), "wb")
        self._wav.setnchannels(1)
        self._wav.setsampwidth(2)      # 16-bit
        self._wav.setframerate(audio_fs)

        self._meta_path = os.path.join(session_dir, "metadata.json")
        self._meta = meta
        self._write_meta()

    def _write_meta(self):
        with open(self._meta_path, "w") as f:
            json.dump(self._meta, f, indent=2)

    def write_imu(self, t_us, ax, ay, az, gx, gy, gz, pitch, roll, yaw, yaw_rate):
        with self._lock:
            self._imu.write(
                f"{t_us},{ax:.6f},{ay:.6f},{az:.6f},"
                f"{gx:.5f},{gy:.5f},{gz:.5f},"
                f"{pitch:.4f},{roll:.4f},{yaw:.4f},{yaw_rate:.5f}\n")
            self.imu_rows += 1

    def write_audio(self, t_us, n, pcm_bytes):
        with self._lock:
            self._blocks.write(
                f"{self.audio_blocks},{t_us},{n},{self.audio_samples}\n")
            self._wav.writeframes(pcm_bytes)
            self.audio_samples += n
            self.audio_blocks += 1

    def close(self, extra_meta=None):
        with self._lock:
            try:
                self._imu.close()
            except Exception:
                pass
            try:
                self._blocks.close()
            except Exception:
                pass
            try:
                self._wav.close()
            except Exception:
                pass
            if extra_meta:
                self._meta.update(extra_meta)
            self._meta["imu_rows"] = self.imu_rows
            self._meta["audio_samples"] = self.audio_samples
            self._meta["audio_blocks"] = self.audio_blocks
            self._write_meta()


# =============================================================================
# Live audio feature extraction (host side)
# =============================================================================
class AudioAnalyzer:
    def __init__(self, fs):
        self.fs = fs
        self.buf = np.zeros(FFT_WINDOW, dtype=np.float32)
        self._window = np.hanning(FFT_WINDOW).astype(np.float32)
        self._freqs = np.fft.rfftfreq(FFT_WINDOW, 1.0 / fs)
        self.dominant_hz = 0.0
        self.rms = 0.0
        self.centroid_hz = 0.0
        self._since = 0

    def push(self, samples_int16: np.ndarray):
        x = samples_int16.astype(np.float32) / 32768.0
        n = len(x)
        if n >= FFT_WINDOW:
            self.buf = x[-FFT_WINDOW:].copy()
        else:
            self.buf = np.roll(self.buf, -n)
            self.buf[-n:] = x
        self._since += n
        # RMS over what just arrived (responsive level meter)
        if n:
            self.rms = float(np.sqrt(np.mean(x * x)))
        if self._since >= FFT_HOP_MIN_SAMPLES:
            self._since = 0
            self._spectrum()

    def _spectrum(self):
        spec = np.abs(np.fft.rfft(self.buf * self._window))
        if spec.sum() <= 1e-9:
            self.dominant_hz = 0.0
            self.centroid_hz = 0.0
            return
        # Ignore DC bin for dominant frequency.
        k = int(np.argmax(spec[1:])) + 1
        self.dominant_hz = float(self._freqs[k])
        self.centroid_hz = float(np.sum(self._freqs * spec) / np.sum(spec))


# =============================================================================
# Main application
# =============================================================================
class WeldSenseApp:
    def __init__(self, port=None):
        self.requested_port = port
        self.ser = None
        self.parser = StreamParser()
        self.fusion = Fusion()
        self.config: ConfigPacket | None = None
        self.audio: AudioAnalyzer | None = None

        self._reader_thread = None
        self._stop = threading.Event()
        self._state_lock = threading.Lock()
        self._recorder: Recorder | None = None
        self._rec_lock = threading.Lock()
        self.session_name = ""

        # default scales until CONFIG arrives (±4g / ±1000dps)
        self.accel_scale = 0.122e-3
        self.gyro_scale = 35.0e-3
        self.audio_fs = 16000

        self._state = {
            "connected": False,
            "port": "",
            "recording": False,
            "session_name": "",
            "calibrated": False,
            "pitch": 0.0, "roll": 0.0, "yaw": 0.0, "yaw_rate": 0.0,
            "ax": 0.0, "ay": 0.0, "az": 0.0,
            "gx": 0.0, "gy": 0.0, "gz": 0.0,
            "dominant_hz": 0.0, "rms": 0.0, "centroid_hz": 0.0,
            "imu_packets": 0, "audio_packets": 0,
            "imu_dropped": 0, "audio_dropped": 0,
            "crc_errors": 0, "resyncs": 0,
            "device_audio_overflows": 0, "device_imu_ok": False,
            "fw_version": "", "status_msg": "idle",
            "imu_rate_est": 0.0, "audio_rate_est": 0.0,
        }
        self._imu_ts_hist = []
        self._audio_ts_hist = []

    # ---- state helpers ----
    def get_state(self):
        with self._state_lock:
            s = dict(self._state)
        s["imu_dropped"] = self.parser.stats.dropped.get(0x01, 0)
        s["audio_dropped"] = self.parser.stats.dropped.get(0x02, 0)
        s["crc_errors"] = self.parser.stats.crc_errors
        s["resyncs"] = self.parser.stats.resyncs
        return s

    def _set(self, **kw):
        with self._state_lock:
            self._state.update(kw)

    # ---- serial connection ----
    @staticmethod
    def list_ports():
        return [p.device for p in serial.tools.list_ports.comports()]

    @staticmethod
    def auto_port():
        """Find the XIAO across macOS / Linux / Windows.

        Preference order:
          1) Seeed USB vendor id 0x2886 (most reliable, OS-independent)
          2) description/manufacturer mentions xiao / seeed / nrf52
          3) an OS-typical serial device name (cu.usbmodem*, ttyACM*, COMx)
        """
        ports = list(serial.tools.list_ports.comports())
        for p in ports:                      # 1) Seeed vendor id
            if getattr(p, "vid", None) == 0x2886:
                return p.device
        for p in ports:                      # 2) name keywords
            blob = " ".join(str(x) for x in
                            (p.description, p.manufacturer,
                             getattr(p, "product", None)) if x).lower()
            if any(k in blob for k in ("xiao", "seeed", "nrf52")):
                return p.device
        for p in ports:                      # 3) OS-typical names
            dev = p.device
            if ("usbmodem" in dev or "usbserial" in dev or "ACM" in dev
                    or dev.upper().startswith("COM")):
                return dev
        return None

    def connect(self, port=None):
        if self.ser is not None:
            return True, "already connected"
        port = port or self.requested_port or self.auto_port()
        if not port:
            return False, "no serial port found (is the XIAO plugged in?)"
        try:
            # baudrate is nominal for USB CDC; any value works.
            self.ser = serial.Serial(port, baudrate=1000000, timeout=0.05)
        except Exception as e:
            return False, f"open failed: {e}"
        self._stop.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop,
                                               name="serial-reader", daemon=True)
        self._reader_thread.start()
        # ask the device to (re)send its CONFIG
        try:
            self.ser.write(b"C")
        except Exception:
            pass
        self._set(connected=True, port=port, status_msg=f"connected {port}")
        return True, f"connected {port}"

    def disconnect(self):
        self._stop.set()
        if self._reader_thread:
            self._reader_thread.join(timeout=1.0)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self._set(connected=False, status_msg="disconnected")
        return True, "disconnected"

    def _reader_loop(self):
        while not self._stop.is_set():
            try:
                data = self.ser.read(4096)
            except Exception as e:
                self._set(status_msg=f"serial error: {e}", connected=False)
                break
            if not data:
                continue
            for pkt in self.parser.feed(data):
                self._handle(pkt)

    # ---- packet handling ----
    def _handle(self, pkt):
        if isinstance(pkt, ConfigPacket):
            self.config = pkt
            self.accel_scale = pkt.accel_scale_g_per_lsb
            self.gyro_scale = pkt.gyro_scale_dps_per_lsb
            self.audio_fs = pkt.audio_fs_hz
            self.audio = AudioAnalyzer(pkt.audio_fs_hz)
            self._set(fw_version=f"{pkt.fw_major}.{pkt.fw_minor}",
                      status_msg="config received")
            return

        if isinstance(pkt, ImuPacket):
            ax = pkt.ax * self.accel_scale
            ay = pkt.ay * self.accel_scale
            az = pkt.az * self.accel_scale
            gx = pkt.gx * self.gyro_scale
            gy = pkt.gy * self.gyro_scale
            gz = pkt.gz * self.gyro_scale
            self.fusion.update(pkt.t_us, ax, ay, az, gx, gy, gz)
            self._rate_est(self._imu_ts_hist, pkt.t_us, "imu_rate_est")
            with self._state_lock:
                self._state.update(
                    ax=ax, ay=ay, az=az, gx=gx, gy=gy, gz=gz,
                    pitch=self.fusion.pitch, roll=self.fusion.roll,
                    yaw=self.fusion.yaw, yaw_rate=self.fusion.yaw_rate,
                    calibrated=self.fusion.calibrated,
                    imu_packets=self.parser.stats.last_seq.get(0x01, 0))
            with self._rec_lock:
                if self._recorder:
                    self._recorder.write_imu(
                        pkt.t_us, ax, ay, az, gx, gy, gz,
                        self.fusion.pitch, self.fusion.roll,
                        self.fusion.yaw, self.fusion.yaw_rate)
            return

        if isinstance(pkt, AudioPacket):
            pcm = np.frombuffer(pkt.samples, dtype="<i2")
            if self.audio:
                self.audio.push(pcm)
                self._set(dominant_hz=self.audio.dominant_hz,
                          rms=self.audio.rms,
                          centroid_hz=self.audio.centroid_hz)
            self._rate_est(self._audio_ts_hist, pkt.t_us, "audio_rate_est",
                           per_block=pkt.n)
            with self._state_lock:
                self._state["audio_packets"] = self.parser.stats.last_seq.get(0x02, 0)
            with self._rec_lock:
                if self._recorder:
                    self._recorder.write_audio(pkt.t_us, pkt.n, pkt.samples)
            return

        if isinstance(pkt, StatusPacket):
            self._set(device_audio_overflows=pkt.audio_overflows,
                      device_imu_ok=pkt.imu_ok)
            return

    def _rate_est(self, hist, t_us, key, per_block=1):
        hist.append(t_us)
        if len(hist) > 64:
            hist.pop(0)
        if len(hist) >= 2:
            span = ((hist[-1] - hist[0]) & 0xFFFFFFFF) / 1e6
            if span > 0:
                rate = (len(hist) - 1) * per_block / span
                self._set(**{key: rate})

    # ---- recording control ----
    def start_recording(self, name=None):
        with self._rec_lock:
            if self._recorder:
                return False, "already recording"
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            base = (name or "weld_sample").strip().replace(" ", "_") or "weld_sample"
            self.session_name = f"{base}_{ts}"
            session_dir = os.path.join(RECORDINGS_DIR, self.session_name)
            meta = self._build_metadata(self.session_name)
            self._recorder = Recorder(session_dir, self.audio_fs, meta)
        self._set(recording=True, session_name=self.session_name,
                  status_msg=f"recording -> {self.session_name}")
        return True, self.session_name

    def stop_recording(self):
        with self._rec_lock:
            if not self._recorder:
                return False, "not recording"
            rec = self._recorder
            self._recorder = None
        rec.close(extra_meta={
            "stop_time": datetime.now().isoformat(),
            # Refresh with the FINAL calibrated bias — calibration may have
            # finished after recording began (bias at start could be zeros).
            "gyro_bias_dps": {
                "gx": float(self.fusion.gyro_bias[0]),
                "gy": float(self.fusion.gyro_bias[1]),
                "gz": float(self.fusion.gyro_bias[2]),
                "calibrated": bool(self.fusion.calibrated),
                "cal_samples": GYRO_CAL_SAMPLES,
            },
        })
        self._set(recording=False, status_msg=f"saved {self.session_name}")
        return True, self.session_name

    def recalibrate(self):
        self.fusion.reset_calibration()
        self._set(calibrated=False, status_msg="recalibrating gyro bias...")
        return True, "recalibrating"

    def _build_metadata(self, name):
        cfg = self.config
        return {
            "sample_name": name,
            "start_time": datetime.now().isoformat(),
            "software_version": SOFTWARE_VERSION,
            "firmware_version": (f"{cfg.fw_major}.{cfg.fw_minor}" if cfg else "unknown"),
            "hardware": {
                "mcu": "Seeed XIAO nRF52840 Sense",
                "imu": "LSM6DS3 (I2C 0x6A)",
                "microphone": "on-board PDM MEMS mic",
            },
            "sample_rates": {
                "imu_hz": cfg.imu_rate_hz if cfg else 200,
                "audio_fs_hz": cfg.audio_fs_hz if cfg else self.audio_fs,
                "audio_block_samples": cfg.audio_block if cfg else 256,
            },
            "imu_ranges": {
                "accel_range_g": cfg.accel_range_g if cfg else None,
                "gyro_range_dps": cfg.gyro_range_dps if cfg else None,
                "accel_scale_g_per_lsb": self.accel_scale,
                "gyro_scale_dps_per_lsb": self.gyro_scale,
            },
            "gyro_bias_dps": {
                "gx": float(self.fusion.gyro_bias[0]),
                "gy": float(self.fusion.gyro_bias[1]),
                "gz": float(self.fusion.gyro_bias[2]),
                "calibrated": bool(self.fusion.calibrated),
                "cal_samples": GYRO_CAL_SAMPLES,
            },
            "fusion": {
                "type": "complementary",
                "alpha": COMP_ALPHA,
                "notes": "pitch/roll from gravity + gyro; yaw = gyro integration "
                         "only and DRIFTS (no magnetometer).",
            },
            "audio_analysis": {
                "fft_window": FFT_WINDOW,
                "window": "hann",
                "note": "Live features only; audio.wav holds the full raw PCM.",
            },
            "data_files": {
                "imu.csv": "raw scaled IMU + host-computed orientation, per sample",
                "audio.wav": "raw 16 kHz / 16-bit mono PCM, all samples",
                "audio_blocks.csv": "per-block device timestamp + cumulative "
                                    "sample offset for IMU<->audio alignment",
            },
            "timebase": "device micros() (uint32, wraps ~71.6 min); all packet "
                        "t_us share this one clock.",
        }


# =============================================================================
# HTTP dashboard server
# =============================================================================
def make_handler(app: WeldSenseApp):
    dashboard_path = os.path.join(HERE, "dashboard.html")

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass  # quiet

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/" or self.path.startswith("/index"):
                try:
                    with open(dashboard_path, "rb") as f:
                        body = f.read()
                except FileNotFoundError:
                    self.send_error(500, "dashboard.html missing")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/api/state":
                self._json(app.get_state())
            elif self.path == "/api/ports":
                self._json({"ports": app.list_ports()})
            else:
                self.send_error(404)

        def do_POST(self):
            length = int(self.headers.get("Content-Length", 0) or 0)
            raw = self.rfile.read(length) if length else b""
            try:
                data = json.loads(raw) if raw else {}
            except Exception:
                data = {}
            if self.path == "/api/connect":
                ok, msg = app.connect(data.get("port"))
                self._json({"ok": ok, "msg": msg})
            elif self.path == "/api/disconnect":
                ok, msg = app.disconnect()
                self._json({"ok": ok, "msg": msg})
            elif self.path == "/api/start":
                ok, msg = app.start_recording(data.get("name"))
                self._json({"ok": ok, "msg": msg})
            elif self.path == "/api/stop":
                ok, msg = app.stop_recording()
                self._json({"ok": ok, "msg": msg})
            elif self.path == "/api/recalibrate":
                ok, msg = app.recalibrate()
                self._json({"ok": ok, "msg": msg})
            else:
                self.send_error(404)

    return Handler


def main():
    ap = argparse.ArgumentParser(description="WeldSense host + dashboard")
    ap.add_argument("--port", help="serial device, e.g. /dev/cu.usbmodemXXXX")
    ap.add_argument("--list", action="store_true", help="list serial ports")
    ap.add_argument("--no-connect", action="store_true",
                    help="start server without auto-connecting")
    ap.add_argument("--open", action="store_true",
                    help="open the dashboard in the default browser")
    ap.add_argument("--http-port", type=int, default=HTTP_PORT)
    args = ap.parse_args()

    if args.list:
        for p in serial.tools.list_ports.comports():
            print(f"{p.device}\t{p.description}")
        return

    os.makedirs(RECORDINGS_DIR, exist_ok=True)
    app = WeldSenseApp(port=args.port)

    if not args.no_connect:
        ok, msg = app.connect()
        print(f"[connect] {msg}")

    handler = make_handler(app)
    ThreadingHTTPServer.allow_reuse_address = True
    try:
        httpd = ThreadingHTTPServer((HTTP_HOST, args.http_port), handler)
    except OSError as e:
        print(f"[error] could not start on port {args.http_port}: {e}")
        print(f"[hint] WeldSense may already be running. Open "
              f"http://{HTTP_HOST}:{args.http_port} , or quit the other copy "
              f"(pkill -f weldsense_host.py), or use --http-port <N>.")
        app.disconnect()
        sys.exit(1)
    url = f"http://{HTTP_HOST}:{args.http_port}"
    print(f"[dashboard] {url}")
    print("[info] Press Ctrl+C to quit.")
    if args.open:
        # Open after the server is bound; delay avoids a "connection refused"
        # flash. webbrowser picks the right opener on macOS/Windows/Linux.
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[shutdown] stopping...")
    finally:
        if app.get_state()["recording"]:
            app.stop_recording()
            print("[shutdown] recording saved.")
        app.disconnect()
        httpd.shutdown()


if __name__ == "__main__":
    main()
