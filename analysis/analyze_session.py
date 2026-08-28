#!/usr/bin/env python3
"""
WeldSense — offline audio (+ orientation) analysis for one recorded session.

This is where you actually LOOK at a weld. It reads a session folder, computes
the standard acoustic views, aligns them to torch orientation on the shared
device clock, writes a per-frame feature CSV, and saves report figures.

Usage:
    python3 analyze_session.py                         # newest session
    python3 analyze_session.py ../recordings/weld_sample_01_YYYYMMDD_HHMMSS
    python3 analyze_session.py <dir> --no-show         # save PNGs, don't pop up
    python3 analyze_session.py <dir> --tmin 5 --tmax 20  # zoom to a time window

Outputs (next to this script, in analysis/out/):
    <session>_report.png     spectrogram + feature time-series + torch angle
    <session>_psd.png        average spectrum (which frequencies are present)
    <session>_features.csv   per-frame features for your own stats / ML

WHAT THE FEATURES MEAN (see the README "Reading weld audio" section):
    level_db        loudness. Rises with power / spatter / instability.
    centroid_hz     "brightness" — where the spectral energy sits on average.
    flatness        0 = tonal (whistle/hum), 1 = white noise. Tonal content
                    often means a resonating keyhole / periodic process.
    rolloff85_hz    frequency below which 85% of the energy lives.
    flux            spectral change between frames — spikes = transients
                    (spatter ejection, keyhole pops, mode switches).
    band_*          fraction of energy in 0-0.5 / 0.5-2 / 2-4 / 4-8 kHz bands.
    dom_hz          single loudest frequency in the frame.
    pitch/roll_deg  torch orientation at that instant (interpolated from IMU).
"""

from __future__ import annotations
import argparse
import glob
import json
import os
import sys
import wave

import numpy as np
from scipy.signal import stft, welch, find_peaks

import matplotlib
matplotlib.use("Agg") if "--no-show" in sys.argv else None
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_REC = os.path.abspath(os.path.join(HERE, "..", "recordings"))
OUTDIR = os.path.join(HERE, "out")

# Analysis bands (Hz). Edit freely — these are a sensible starting split for a
# 16 kHz airborne capture (Nyquist = 8 kHz).
BANDS = [(0, 500), (500, 2000), (2000, 4000), (4000, 8000)]
NPERSEG = 1024          # STFT window (~64 ms @16 kHz, ~15.6 Hz bins)
NOVERLAP = 512          # 50% overlap -> ~31 frames/s


# ---------------------------------------------------------------------------
def newest_session(recdir):
    dirs = [d for d in glob.glob(os.path.join(recdir, "*")) if os.path.isdir(d)]
    if not dirs:
        sys.exit(f"No sessions found in {recdir}")
    return max(dirs, key=os.path.getmtime)


def load_session(session_dir):
    meta = {}
    mp = os.path.join(session_dir, "metadata.json")
    if os.path.exists(mp):
        meta = json.load(open(mp))

    w = wave.open(os.path.join(session_dir, "audio.wav"))
    fs = w.getframerate()
    audio = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float64)

    # IMU (may be absent/empty)
    imu = None
    ip = os.path.join(session_dir, "imu.csv")
    if os.path.exists(ip):
        try:
            imu = np.genfromtxt(ip, delimiter=",", names=True)
            if imu.size == 0:
                imu = None
        except Exception:
            imu = None

    # audio_blocks -> map audio sample index to device clock (t_us)
    blocks = None
    bp = os.path.join(session_dir, "audio_blocks.csv")
    if os.path.exists(bp):
        try:
            blocks = np.genfromtxt(bp, delimiter=",", names=True)
            if blocks.size == 0:
                blocks = None
        except Exception:
            blocks = None
    return meta, fs, audio, imu, blocks


def sample_to_t_us(blocks):
    """Linear map: audio sample index -> device t_us, from block timestamps.
    Block t_us marks the END of a block (samples [offset, offset+n))."""
    if blocks is None:
        return None
    end_sample = np.atleast_1d(blocks["sample_offset"]) + np.atleast_1d(blocks["n_samples"])
    t_us = np.atleast_1d(blocks["t_us"]).astype(np.float64)
    if len(end_sample) < 2:
        return None
    # robust linear fit sample -> t_us
    a, b = np.polyfit(end_sample.astype(np.float64), t_us, 1)
    return lambda s: a * s + b


def spectral_features(f, Z):
    """Z: complex STFT (freq x time). Returns dict of per-frame features."""
    mag = np.abs(Z) + 1e-12
    power = mag ** 2
    total = power.sum(axis=0)

    level_db = 10 * np.log10(total + 1e-12)
    centroid = (f[:, None] * mag).sum(0) / mag.sum(0)
    # spectral flatness = geometric mean / arithmetic mean of power
    gmean = np.exp(np.mean(np.log(power), axis=0))
    amean = np.mean(power, axis=0)
    flatness = gmean / amean
    # rolloff 85%
    cume = np.cumsum(power, axis=0)
    thresh = 0.85 * cume[-1, :]
    idx = (cume >= thresh[None, :]).argmax(axis=0)
    rolloff = f[idx]
    # spectral flux (positive changes only)
    dmag = np.diff(mag, axis=1, prepend=mag[:, :1])
    flux = np.sqrt((np.maximum(dmag, 0) ** 2).sum(0))
    # dominant frequency (ignore DC bin)
    dom = f[1 + np.argmax(mag[1:, :], axis=0)]
    # band energy ratios
    bands = {}
    for lo, hi in BANDS:
        m = (f >= lo) & (f < hi)
        bands[f"band_{lo}_{hi}"] = power[m, :].sum(0) / (total + 1e-12)
    return dict(level_db=level_db, centroid=centroid, flatness=flatness,
                rolloff=rolloff, flux=flux, dom=dom, bands=bands)


def main():
    ap = argparse.ArgumentParser(description="Analyze one WeldSense session.")
    ap.add_argument("session", nargs="?", help="session dir (default: newest)")
    ap.add_argument("--recdir", default=DEFAULT_REC)
    ap.add_argument("--no-show", action="store_true", help="save PNGs only")
    ap.add_argument("--tmin", type=float, default=None, help="start time (s)")
    ap.add_argument("--tmax", type=float, default=None, help="end time (s)")
    args = ap.parse_args()

    session_dir = args.session or newest_session(args.recdir)
    session_dir = os.path.abspath(session_dir)
    name = os.path.basename(session_dir.rstrip("/"))
    os.makedirs(OUTDIR, exist_ok=True)

    meta, fs, audio, imu, blocks = load_session(session_dir)
    dur = len(audio) / fs
    print(f"\nSession: {name}")
    print(f"  audio: {dur:.1f} s @ {fs} Hz  ({len(audio)} samples)")

    # ---- health checks ----
    peak = np.max(np.abs(audio)) if len(audio) else 0
    clip = 100 * np.mean(np.abs(audio) > 32000) if len(audio) else 0
    headroom_db = 20 * np.log10(32767 / max(peak, 1))
    print(f"  peak: {int(peak)}/32767  headroom: {headroom_db:.1f} dB  "
          f"clipping: {clip:.2f}%")
    if clip > 0.1:
        print("  ** WARNING: clipping — lower PDM.setGain() in the firmware.")
    elif headroom_db > 25:
        print("  note: very quiet capture (lots of headroom). If this was a real "
              "weld, consider raising PDM.setGain() for more resolution.")

    # ---- average spectrum (what frequencies are present) ----
    f_w, P = welch(audio, fs=fs, nperseg=min(8192, len(audio)))
    pk, _ = find_peaks(10 * np.log10(P + 1e-12), prominence=3, distance=5)
    pk = pk[np.argsort(P[pk])[::-1]][:8]
    tones = sorted(int(f_w[i]) for i in pk if f_w[i] > 25)
    print(f"  dominant spectral peaks (Hz): {tones}")

    # ---- STFT + features ----
    f, t, Z = stft(audio, fs=fs, nperseg=NPERSEG, noverlap=NOVERLAP)
    feats = spectral_features(f, Z)

    print(f"  median loudness: {np.median(feats['level_db']):.1f} dB(rel)")
    print(f"  median centroid: {np.median(feats['centroid']):.0f} Hz")
    fl = np.median(feats['flatness'])
    print(f"  median flatness: {fl:.3f}  "
          f"({'tonal/periodic' if fl < 0.2 else 'broadband/noisy' if fl > 0.5 else 'mixed'})")
    # transient (spatter/pop) count via flux peaks
    fx = feats['flux']
    tp, _ = find_peaks(fx, height=np.median(fx) + 4 * np.std(fx), distance=3)
    print(f"  transient events (flux spikes): {len(tp)}  "
          f"(~{len(tp)/dur:.1f}/s) — candidate spatter/keyhole pops")

    # ---- orientation aligned to audio frames (shared device clock) ----
    pitch_f = roll_f = None
    s2t = sample_to_t_us(blocks)
    if imu is not None and s2t is not None and "pitch_deg" in imu.dtype.names:
        frame_samples = t * fs
        frame_t_us = s2t(frame_samples)
        imu_t = imu["t_us"].astype(np.float64)
        order = np.argsort(imu_t)
        pitch_f = np.interp(frame_t_us, imu_t[order], imu["pitch_deg"][order])
        roll_f = np.interp(frame_t_us, imu_t[order], imu["roll_deg"][order])

    # ---- write per-frame feature CSV ----
    csv_path = os.path.join(OUTDIR, f"{name}_features.csv")
    cols = {"t_s": t, "level_db": feats["level_db"], "centroid_hz": feats["centroid"],
            "flatness": feats["flatness"], "rolloff85_hz": feats["rolloff"],
            "flux": feats["flux"], "dom_hz": feats["dom"]}
    cols.update(feats["bands"])
    if pitch_f is not None:
        cols["pitch_deg"] = pitch_f
        cols["roll_deg"] = roll_f
    hdr = ",".join(cols.keys())
    arr = np.column_stack(list(cols.values()))
    np.savetxt(csv_path, arr, delimiter=",", header=hdr, comments="", fmt="%.5f")
    print(f"\n  wrote {csv_path}  ({arr.shape[0]} frames x {arr.shape[1]} cols)")

    # ---- time-window mask for plotting ----
    tmin = args.tmin if args.tmin is not None else 0
    tmax = args.tmax if args.tmax is not None else dur
    m = (t >= tmin) & (t <= tmax)

    # ---- figure 1: spectrogram + features + orientation ----
    nrows = 5 if pitch_f is not None else 4
    fig, ax = plt.subplots(nrows, 1, figsize=(12, 2.1 * nrows), sharex=True)
    Sdb = 20 * np.log10(np.abs(Z) + 1e-9)
    vmax = np.percentile(Sdb, 99.5)
    ax[0].pcolormesh(t[m], f / 1000, Sdb[:, m], shading="auto",
                     vmin=vmax - 70, vmax=vmax, cmap="magma")
    ax[0].set_ylabel("kHz"); ax[0].set_title(f"{name} — spectrogram (dB)")
    ax[1].plot(t[m], feats["level_db"][m], lw=0.8, color="#f0883e")
    ax[1].set_ylabel("level dB")
    ax[2].plot(t[m], feats["centroid"][m], lw=0.8, label="centroid", color="#58a6ff")
    ax[2].plot(t[m], feats["rolloff"][m], lw=0.6, label="rolloff85", color="#8b949e")
    ax[2].set_ylabel("Hz"); ax[2].legend(loc="upper right", fontsize=8)
    ax[3].plot(t[m], feats["flatness"][m], lw=0.8, label="flatness", color="#3fb950")
    ax3b = ax[3].twinx()
    ax3b.plot(t[m], feats["flux"][m], lw=0.6, label="flux", color="#d29922", alpha=.7)
    ax[3].set_ylabel("flatness"); ax3b.set_ylabel("flux")
    ax[3].legend(loc="upper left", fontsize=8); ax3b.legend(loc="upper right", fontsize=8)
    if pitch_f is not None:
        ax[4].plot(t[m], pitch_f[m], lw=0.9, label="pitch", color="#f0883e")
        ax[4].plot(t[m], roll_f[m], lw=0.9, label="roll", color="#58a6ff")
        ax[4].set_ylabel("deg"); ax[4].legend(loc="upper right", fontsize=8)
    ax[-1].set_xlabel("time (s)")
    fig.tight_layout()
    p1 = os.path.join(OUTDIR, f"{name}_report.png")
    fig.savefig(p1, dpi=120)
    print(f"  wrote {p1}")

    # ---- figure 2: average spectrum ----
    fig2, a2 = plt.subplots(figsize=(10, 4))
    a2.semilogy(f_w, P, color="#58a6ff", lw=1)
    for i in pk:
        a2.axvline(f_w[i], color="#f85149", ls="--", lw=.6)
        a2.text(f_w[i], P[i], f" {int(f_w[i])}", fontsize=8, color="#f85149")
    a2.set_xlabel("Hz"); a2.set_ylabel("PSD"); a2.set_xlim(0, fs / 2)
    a2.set_title(f"{name} — average spectrum (Welch)")
    fig2.tight_layout()
    p2 = os.path.join(OUTDIR, f"{name}_psd.png")
    fig2.savefig(p2, dpi=120)
    print(f"  wrote {p2}\n")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
