# WeldSense

A sensor / data-acquisition system for a handheld laser-welding torch, built on
a **Seeed XIAO nRF52840 Sense**. The design goal is a *real experimental
instrument*, not a demo: capture raw, correlate torch **orientation** with weld
**acoustics**, and keep every raw sample so the analysis can be redone later
without re-running the weld.

## Architecture — the one rule

> **The XIAO is a raw data-acquisition device. It does NOT compute anything.**

```
  ┌────────────────────────┐        framed binary over USB CDC        ┌──────────────────────────┐
  │  XIAO nRF52840 Sense    │  ───────────────────────────────────▶   │  Host (Python 3, macOS)   │
  │  • LSM6DS3 @ 200 Hz     │   0xAA55 | type | seq | len | ... | CRC  │  • decode + integrity     │
  │  • PDM mic @ 16 kHz     │                                          │  • gyro cal, pitch/roll   │
  │  • raw int16 only       │   single micros() clock timestamps all   │  • yaw, FFT, features     │
  │  • sample + transmit     │                                         │  • record raw to disk     │
  └────────────────────────┘                                          │  • serve dashboard        │
                                                                       └──────────────────────────┘
                                                                                    │
                                                                       http://127.0.0.1:8765 (browser view only)
```

The firmware only **samples and transmits**. Calibration, orientation, FFT,
feature extraction, visualization, and logging all happen on the host. The
browser dashboard is a *view* of host state — it is **not** the acquisition
layer; recording continues even with no browser open.

## Get started (for a teammate with their own XIAO)

You need: a XIAO nRF52840 Sense wired to the LSM6DS3, a **data-capable** USB-C
cable, Python 3, and Arduino IDE 1.8.19. Works on **macOS, Windows, and Linux**.

```bash
git clone https://github.com/tlosanti/weldsense.git
cd weldsense
```

**macOS / Linux:**
```bash
./run.sh
```
**Windows** (double-click `run.bat`, or in Command Prompt / PowerShell):
```bat
run.bat
```

The launcher sets up a Python virtual-env, installs dependencies, launches the
host, and opens the dashboard at **http://127.0.0.1:8765**. First run takes a
minute (installing deps); after that it's instant. The XIAO is auto-detected on
all three OSes (by Seeed's USB vendor id), whether it shows up as
`/dev/cu.usbmodem*` (macOS), `/dev/ttyACM*` (Linux), or `COM5` (Windows). Flash
the firmware once (see [Flash the firmware](#1-flash-the-firmware)) and you're
live.

> **Windows note:** install Python 3 from
> [python.org](https://www.python.org/downloads/windows/) and tick **"Add
> python.exe to PATH"** during setup. No serial driver is needed — the XIAO
> enumerates as a standard USB serial (CDC) device on Windows 10/11.

> **Note:** WeldSense runs **locally** on the machine the torch is plugged into —
> the dashboard is served on `127.0.0.1`, not a public URL. GitHub distributes
> the code; the USB data stays on your own computer.

## Repository layout

```
WeldSense V2/
├── firmware/
│   └── weldsense_firmware/
│       └── weldsense_firmware.ino # XIAO raw acquisition (upload with Arduino IDE)
├── host/
│   ├── weldsense_host.py          # acquisition + recording + analysis + web server
│   ├── protocol.py                # binary frame decoder + CRC (standalone, tested)
│   ├── dashboard.html             # live browser dashboard
│   ├── test_protocol.py           # protocol unit tests (no hardware needed)
│   └── requirements.txt
└── recordings/                    # one folder per session (created at runtime)
```

---

## Quick start

### 1. Flash the firmware
1. Arduino IDE 1.8.19, board **"XIAO nRF52840 Sense (No Updates)"**, Seeed mbed
   package **2.9.3**.
2. Install libraries: **Seeed Arduino LSM6DS3** (Library Manager). `PDM` ships
   with the mbed core.
3. Open `firmware/weldsense_firmware/weldsense_firmware.ino`, **Upload**.

### 2. Run the host
Easiest: use the launcher — `./run.sh` (macOS/Linux) or `run.bat` (Windows).

To run it by hand instead (or to pass flags):
```bash
# macOS / Linux
python3 -m venv .venv && source .venv/bin/activate
pip install -r host/requirements.txt         # pyserial, numpy, scipy
python3 host/weldsense_host.py --open
```
```bat
REM Windows (Command Prompt / PowerShell)
python -m venv .venv
.venv\Scripts\activate
pip install -r host\requirements.txt
python host\weldsense_host.py --open
```
Then open **http://127.0.0.1:8765** (or pass `--open` and it opens itself).

Useful flags (same on every OS):
```bash
python3 host/weldsense_host.py --list        # list serial ports
python3 host/weldsense_host.py --port COM5   # or /dev/cu.usbmodemXXXX (macOS)
python3 host/weldsense_host.py --no-connect  # serve UI, connect from browser
python3 host/weldsense_host.py --http-port 9000
```

### 3. Record
- Click **Connect XIAO** (if not auto-connected).
- Hold the torch **still ~2 s** so the host estimates gyro bias (the dashboard
  says "Calibrating…"). Re-do anytime with **Recalibrate Gyro**.
- Type a sample name, click **Start Recording** → **Stop Recording**.
- Files land in `recordings/<name>_YYYYMMDD_HHMMSS/`.

Run the protocol tests anytime (no hardware): `python3 host/test_protocol.py`.

---

## Data storage

Every session creates a self-describing folder:

```
recordings/weld_sample_01_20260827_143000/
├── imu.csv            # raw scaled IMU + host-computed orientation, per sample
├── audio.wav          # raw 16 kHz / 16-bit / mono PCM — ALL samples
├── audio_blocks.csv   # per-block device timestamp + cumulative sample offset
└── metadata.json      # everything needed to re-interpret the raw data
```

**`imu.csv`** columns:
`t_us, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, pitch_deg, roll_deg, yaw_deg, yaw_rate_dps`
The first seven are raw measurements (`t_us` = device clock, accel/gyro already
scaled to physical units from the raw int16 counts). The last four are
host-computed and can be *recomputed from the raw columns* later — they're saved
for convenience, not as the source of truth.

**`audio.wav`** is the untouched microphone PCM. No timestamp per sample (see
Sync below) — timing comes from `audio_blocks.csv` + the 16 kHz rate.

**`audio_blocks.csv`** (`block_index, t_us, n_samples, sample_offset`) is the key
to alignment: each row ties an audio block's cumulative sample offset in the WAV
to a device timestamp on the *same clock as the IMU*. If a block is ever dropped,
`sample_offset` jumps — so gaps are explicit and recoverable, never silently
smeared.

**`metadata.json`** records sample name, start/stop time, sample rates, IMU
ranges + scale factors, the estimated **gyro bias**, fusion settings, FFT
settings, firmware/software versions, and hardware info.

### Recompute orientation from raw later
Because raw is preserved, you can change the analysis without re-welding:
```python
import numpy as np, pandas as pd
d = pd.read_csv("recordings/.../imu.csv")
pitch = np.degrees(np.arctan2(-d.ax_g, np.hypot(d.ay_g, d.az_g)))  # new algorithm
```

---

## Binary protocol

Little-endian, one frame per packet:

| bytes | field | notes |
|------:|-------|-------|
| 2 | sync | `0xAA 0x55` |
| 1 | type | `0x01`=IMU, `0x02`=AUDIO, `0x03`=STATUS, `0x10`=CONFIG |
| 2 | seq | uint16 **per-type** sequence → per-stream drop detection |
| 2 | len | uint16 payload length |
| len | payload | see below |
| 2 | crc16 | CRC16-CCITT (poly `0x1021`, init `0xFFFF`) over `type+seq+len+payload` |

**IMU payload (16 B):** `u32 t_us | int16 ax ay az gx gy gz` — **raw register
counts**, no scaling on-device. The host multiplies by the scale factors from the
CONFIG packet (`±4 g` → 0.122 mg/LSB, `±1000 dps` → 35 mdps/LSB). Sending raw
counts is smaller *and* loses nothing.

**AUDIO payload (6 + 2·n B):** `u32 t_us | u16 n_samples | int16[n] pcm`. `n` is
256 (the verified PDM block). One block timestamp, not one per sample.

**STATUS (20 B, ~1 Hz):** `u32 t_us | u32 imu_count | u32 audio_count |
u32 audio_overflows | u8 flags` — device-side health / overflow counter.

**CONFIG (20 B, at boot + on request):** firmware version, IMU/audio rates,
block size, ranges, and the **scale factors**. The host asks for it by sending a
single `'C'` byte, so decoding is self-describing — change a range in firmware
and the host adapts with no code edits.

### Improvements over the first-draft idea
- **Per-type sequence numbers** (not just one global) → you can count exactly how
  many IMU vs audio packets were lost.
- **CRC16-CCITT** on every frame; the host parser is a **resynchronizing state
  machine** — bad CRC or stray bytes never emit a packet, it just scans forward
  to the next valid `0xAA55`. Split frames across USB reads reassemble correctly.
- **CONFIG packet** carries scale factors so the format is self-describing and
  future-proof (change ranges, add sensors, without breaking the host).

---

## Timing / synchronization

**One clock rules everything.** The XIAO timestamps every packet from a single
monotonic `micros()`. Because IMU and audio share that clock, they are
inherently on the same timeline — no host-side guesswork about which sample
happened when.

- **IMU (200 Hz):** timestamp taken at the moment of the register read.
- **Audio (16 kHz):** one timestamp per 256-sample block, captured in the PDM
  callback. Convention: `t_us` marks the **end** of the block (when it was handed
  to us). Per-sample time is reconstructed as
  `t(sample k) = t_us − (n−1−k)·(1e6/16000)`.
  You don't want a timestamp on every audio sample — it triples the audio
  bandwidth for information you already have from the constant sample rate. The
  block timestamp anchors the block; the known rate fills in the rest.

**Cross-sensor alignment on the host:** to find the torch orientation at a given
audio sample, take that sample's reconstructed `t_us` (via `audio_blocks.csv`)
and interpolate the IMU columns in `imu.csv` at that time. Both are the same
device clock, so this is a direct lookup.

**Two caveats, both handled honestly:**
1. `micros()` is a uint32 and **wraps every ~71.6 min**. All time deltas on the
   host use modulo-2³² subtraction, so a wrap is a non-event.
2. The XIAO oscillator and your Mac's clock drift slightly relative to each
   other. For within-session correlation this doesn't matter — everything is on
   the *device* clock. If you later need absolute wall-clock alignment (e.g. to a
   separate instrument), log host arrival time alongside `t_us` and fit a linear
   `device_us → host_time` map; the format already leaves room for that.

---

## Bandwidth — and why 115200 is a red herring

**Payload rates:**
- IMU: 200 Hz × 25 B/frame (16 payload + 9 framing) ≈ **5.0 KB/s**
- Audio: 62.5 blocks/s × 527 B/block (512 PCM + 6 + 9) ≈ **32.9 KB/s**
- **Total ≈ 38 KB/s ≈ 304 kbit/s.**

**The key point the baud-rate question misses:** the XIAO enumerates as a **USB
CDC-ACM** device. `Serial.begin(115200)` and pyserial's `baudrate=` are a
*virtual* setting for CDC — there is no real UART, so the number does **not**
throttle throughput. Data moves at **USB Full-Speed bulk** rates: 12 Mbit/s on
the wire, and ~**700 KB/s–1 MB/s** of practical application throughput.

So the honest comparison is 38 KB/s of payload against ~700+ KB/s of USB-CDC
capacity — under **~6 %** utilization, with 10–20× headroom for the future
sensors below. If this were a *real* 115200-baud UART (≈11.5 KB/s usable), 38
KB/s would be ~3× over budget and impossible — but it isn't, so no change to the
approach is needed. The firmware sets a nominal 1 Mbaud purely so any tool that
displays a baud rate shows something sensible; it changes nothing.

This is exactly why the protocol is **binary**: the same audio as ASCII
(`"-32768 "` ≈ 7 B/sample) would be ~112 KB/s and wasteful. Raw int16 is 2
B/sample and the natural format for a future external ADC.

---

## Why host-side computation is the right call for experimental data

1. **Raw is irreplaceable; a weld is not repeatable on demand.** Compute on the
   MCU and you throw away information at capture time. Stream raw and every
   analysis choice (filter cutoff, FFT window, fusion algorithm, ML features)
   stays changeable *after* the experiment — forever.
2. **It fixes your sluggish-rotation problem.** On the old firmware, IMU output
   was gated by the ~32 ms audio-FFT window, so a fast 180° flip looked laggy.
   Decoupling acquisition from analysis lets the IMU stream at a true 200 Hz; the
   host's complementary filter tracks fast motion while gravity keeps it honest
   at rest.
3. **The MCU stays cheap and deterministic.** No FFTs or giant ASCII prints
   competing with sampling — it just samples and ships, which keeps timing tight.
4. **The host has the horsepower and the tools** — NumPy/SciPy, easy plotting,
   version-controlled analysis you can re-run on archived raw data.
5. **Clean upgrade path.** Add a magnetometer, a better IMU, a higher-rate MCU,
   an external ADC, current/voltage or optical sensing, or ML — mostly by adding
   a packet **type** and a host handler. The CONFIG packet keeps the stream
   self-describing so the data format survives the upgrade.

---

## Orientation notes (read this)

- **Pitch & roll are trustworthy** — derived from the gravity vector, corrected
  for gyro bias, smoothed with a complementary filter. This is the primary,
  static-angle measurement.
- **Yaw is relative and drifts.** A 6-axis IMU has **no absolute heading
  reference** — yaw here is pure gyro integration and will wander over time. The
  dashboard flags it with an asterisk. Don't treat it as absolute. Add a
  magnetometer for true heading (the format is ready for it).
- Gyro bias (your ~+0.4 / −2.1 / +0.6 dps offsets) is estimated on the host at
  startup from ~2 s of stillness and stored in `metadata.json`. **Raw gyro is
  never altered** — bias is only subtracted for the derived orientation.

---

## Analyzing a recording

Offline analysis of any saved session — spectrogram, acoustic features, and
torch orientation on the shared clock:

```bash
pip install -r analysis/requirements.txt        # once (adds matplotlib)
python3 analysis/analyze_session.py             # newest session
python3 analysis/analyze_session.py recordings/weld_sample_01_YYYYMMDD_HHMMSS
python3 analysis/analyze_session.py <dir> --tmin 5 --tmax 20   # zoom a window
python3 analysis/analyze_session.py <dir> --no-show            # save PNGs only
```
Writes to `analysis/out/`: `<session>_report.png` (spectrogram + features +
angle), `<session>_psd.png` (average spectrum), and `<session>_features.csv`
(per-frame features for your own stats/ML).

### Reading weld audio

This is a **16 kHz airborne MEMS mic**, so it captures the *audible* band up to
**8 kHz** (Nyquist) — not the ultrasonic 100 kHz–1 MHz range that contact
acoustic-emission sensors use. It's uncalibrated, so **relative change matters,
absolute level does not**. Within that band there's real, usable information:

| Feature | Physical meaning | What to watch for |
|---|---|---|
| **level_db** | loudness | rises with power, spatter, instability |
| **centroid_hz** | spectral "brightness" | shifts with process mode / material |
| **flatness** | tonal (→0) vs noisy (→1) | a strong tone can mean a resonating keyhole; broadband hiss = plume/gas |
| **flux** | spectral change | spikes = transients: **spatter**, keyhole pops, mode switches |
| **band ratios** | energy per 0–0.5 / 0.5–2 / 2–4 / 4–8 kHz | band shifts track process-state changes |

**Physical processes that make sound in laser welding:**
- **Melt-pool oscillation** — low frequency (tens–few hundred Hz).
- **Keyhole dynamics** — in penetration (keyhole) mode a vapor cavity forms and
  oscillates; its stability is tied to penetration depth and to porosity. Often
  the most information-rich band (~1–5 kHz here, capped at 8 kHz).
- **Vapor plume / shielding gas** — broadband hiss (raises flatness).
- **Spatter ejection** — impulsive broadband transients (flux spikes).
- **Conduction vs keyhole mode** — shallow conduction welds sound different
  (quieter, less tonal) from deeper keyhole welds. This mode change is the most
  realistic thing to detect first.
- **Beam wobble / pulse modulation** — if your handheld unit wobbles or pulses
  the beam, expect strong tones at those rates **and their harmonics**. Identify
  and account for these first — they can dominate the spectrum.

**Honest limit:** "frequency X = penetration depth Y" is **not** a lookup you can
read off one weld. It's a *correlation you establish by experiment*:

1. Weld coupons at **controlled, varied settings** (power, speed, focus, gas),
   deliberately spanning good → bad (shallow, full-penetration, burn-through,
   porous).
2. **Label ground truth**: cross-section and measure penetration/defects, or at
   minimum photograph top+back and rate each weld.
3. Run `analyze_session.py` on each; the `features.csv` gives you per-frame
   numbers.
4. **Compare feature distributions across labels** (e.g. does median centroid or
   a band ratio separate full-penetration from lack-of-fusion?). Start with
   simple scatter/box plots before any ML.
5. Once a feature separates classes, *that's* your candidate acoustic indicator —
   validated on your machine, your material, this mic.

Because WeldSense keeps **all raw audio**, you can re-run every analysis choice
over the same welds without re-welding — which is exactly what steps 3–4 need.

### What the literature says (and the one caveat that matters most)

Acoustic monitoring of laser welding is a real, active research field. The
consensus points:

- **Airborne sound does carry penetration information.** Molten-pool pulsation,
  vapor/plasma plume, thermal stress, and **keyhole oscillation** all radiate
  sound, and the keyhole-oscillation bands are reported as *distinctive when the
  weld is fully penetrated*. (Reviews: *Acoustic process monitoring in laser
  beam welding*; *Interpreting acoustic emissions to determine the weld depth
  during laser beam welding*, J. Laser Appl. 34(4), 2022.)
- **Melt-pool / keyhole oscillations sit partly in the audible band.**
  Frequency-based studies of weld-pool dynamics report oscillations from ~100 Hz
  up into the low kHz (*Frequency-based analysis of weld pool dynamics and
  keyhole oscillations at laser beam welding of galvanized steel sheets*).
- **Spectrogram + deep learning is the modern approach** — exactly the
  time-frequency view this tool produces. (*Laser Welding Penetration Monitoring
  Based on Time-Frequency Characterization of Acoustic Emission and CNN-LSTM*,
  Sensors 2023, PMC9967101; *Laser Beam Welding State Classification: A Deep
  Learning Framework for Acoustic Signal Intelligence*, Machines 2025.)
- **Ground truth is established by an independent sensor, not by the audio
  itself.** The gold-standard method (Empa / Shevchik & Wasmer, *Supervised deep
  learning for real-time quality monitoring of laser welding with X-ray
  radiographic guidance*, Sci. Rep. 2020) labels each moment with **X-ray
  radiography**, then learns the acoustic signature. Your accessible equivalent:
  **cross-section the weld and measure penetration under a microscope**, plus
  top/back photos. That is what "verifies" an acoustic finding.

> **The caveat that matters most for THIS hardware:** most laser-welding
> penetration studies use **ultrasonic** acoustic emission — sensors and sampling
> in the **~40 kHz – 1 MHz** range. The strongest "keyhole resonance ↔ full
> penetration" bands are often reported around **40–110 kHz**, with unstable
> melt-pool activity ~8–26 kHz. Your XIAO mic samples at **16 kHz → 8 kHz
> Nyquist**, so it **cannot see any of those bands.** What it *can* see: the
> low-frequency melt-pool/process tones (hundreds of Hz to a few kHz), overall
> loudness and stability, mode changes, and spatter transients — all genuinely
> useful for *process-state* and *event* detection, but **not** the ultrasonic
> penetration-depth measurement those papers describe. To do that measurement you
> would upgrade the front end to a high-sample-rate ultrasonic AE sensor
> (contact piezo) or a wideband optical microphone — which is exactly the kind of
> sensor swap the raw-data architecture is built to accept later.

**Bottom line:** in the audible band you can realistically build a *classifier of
process state* (e.g. conduction vs keyhole, stable vs spatter/burn-through) on
**your** machine and material, validated against cross-sections. Reading absolute
penetration depth from this mic alone is not supported by the physics — be
honest about that boundary in any writeup.

## Troubleshooting — serial

**Find the port** (`--list` works on every OS)
```bash
python3 host/weldsense_host.py --list
```
- **macOS:** use the `cu.usbmodem*` device, not `tty.*` (`tty.*` blocks waiting
  for carrier detect). `ls /dev/cu.usbmodem*`
- **Linux:** it's `/dev/ttyACM0`. If "permission denied", add yourself to the
  group: `sudo usermod -a -G dialout $USER` then log out/in.
- **Windows:** it's `COMx` (check Device Manager → Ports). No driver needed on
  Windows 10/11 — the XIAO is a standard USB CDC serial device.

**"Resource busy" / "Access is denied" / can't open the port**
- Only one program can hold the port. Close the **Arduino Serial Monitor /
  Plotter**, and close any other running copy of the host:
  - macOS/Linux: `pkill -f weldsense_host.py`
  - Windows: close the launcher window (or end `python.exe` in Task Manager).

**Port doesn't appear**
- Try a different **data-capable** USB-C cable (charge-only cables won't
  enumerate). Reseat, try another port/hub.
- Confirm the board is running (the on-board LED). If it was mid-upload,
  double-tap **RESET** to enter the bootloader and re-flash.

**Port name changes between plug-ins** — `usbmodemXXXX` / `COMx` can vary. The
host auto-detects by Seeed's USB vendor id, or pass `--port`.

**Windows: `python` not recognized** — reinstall Python from python.org with
**"Add python.exe to PATH"** checked, or use `py` instead of `python`.

**Permissions** — usually none needed on macOS/Windows. If macOS shows a
security prompt for a USB/serial device, allow it.

**No data / all zeros**
- Watch the dashboard **Link/Integrity** panel: rising `CRC errors` or `resyncs`
  = a flaky cable or wrong firmware. `IMU rate` should read ~200, `audio rate`
  ~16000.
- `device audio overflow > 0` means the host isn't draining fast enough (rare on
  USB) — the ring buffer drops a block and logs it rather than corrupting data.
- Re-request config: the host sends `'C'` on connect; reconnect if CONFIG never
  arrives (firmware version stays blank).

**Robustness guarantees**
- Corrupted/dropped serial bytes never produce bad samples — CRC + resync drop
  them and the counters make losses visible.
- Recording is independent of the dashboard; closing the browser doesn't stop it.
- Raw stays recoverable even if you rewrite all the analysis later: `imu.csv`
  raw columns, `audio.wav`, and `audio_blocks.csv` fully reconstruct the session.
```
