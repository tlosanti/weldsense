/* =============================================================================
 * WeldSense — XIAO nRF52840 Sense RAW ACQUISITION FIRMWARE
 * =============================================================================
 *
 * ROLE OF THIS DEVICE:
 *   This firmware is a *dumb, fast* data-acquisition front end. It does NOT
 *   compute orientation, FFTs, RMS, spectral centroid, or anything else.
 *   It samples the sensors and streams RAW data to the host over USB using a
 *   framed binary protocol. ALL analysis happens on the computer.
 *
 * WHAT IT STREAMS:
 *   - IMU  (LSM6DS3): raw int16 register counts, 6 axes, at 200 Hz.
 *   - AUDIO (PDM mic): raw int16 PCM, 16 kHz, in 256-sample blocks
 *                      (using the KNOWN-WORKING PDM.onReceive callback).
 *
 * CLOCK:
 *   A single monotonic microsecond clock (micros()) timestamps everything.
 *   That one shared clock is what lets the host line up 200 Hz IMU with
 *   16 kHz audio. See README "Timing / Synchronization".
 *
 * PROTOCOL (little-endian):
 *   Byte 0 : 0xAA           sync 0
 *   Byte 1 : 0x55           sync 1
 *   Byte 2 : type           0x01 IMU, 0x02 AUDIO, 0x03 STATUS, 0x10 CONFIG
 *   Byte 3 : seq low        uint16 per-type sequence number (LE)
 *   Byte 4 : seq high
 *   Byte 5 : len low        uint16 payload length in bytes (LE)
 *   Byte 6 : len high
 *   Byte 7.. : payload
 *   last 2 : CRC16-CCITT (LE) computed over bytes [2 .. 6+len-1]
 *            (i.e. type + seq + len + payload)
 *
 * WHY BINARY / WHY NOT WORRY ABOUT 115200:
 *   The XIAO enumerates as USB CDC-ACM. The "baud rate" is a virtual setting
 *   and does NOT throttle USB throughput — data moves at USB Full-Speed bulk
 *   rates (hundreds of KB/s). Our full raw stream is ~38 KB/s, which is
 *   trivial for USB. See README "Bandwidth".
 *
 * Board:  XIAO nRF52840 Sense (No Updates), Seeed mbed package 2.9.3
 * IDE:    Arduino 1.8.19
 * Libs:   Seeed Arduino LSM6DS3, PDM (bundled with the mbed core)
 * ========================================================================== */

#include <PDM.h>
#include "LSM6DS3.h"
#include "Wire.h"

// ----------------------------------------------------------------------------
// Firmware identity / configuration (also reported to host in CONFIG packet)
// ----------------------------------------------------------------------------
#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 0

// IMU output ranges. These decide the scale factors the HOST uses to turn the
// raw int16 counts back into g / dps. If you change these, the host reads the
// new values from the CONFIG packet automatically — nothing to hand-edit.
#define ACCEL_RANGE_G     4        // ±4 g
#define GYRO_RANGE_DPS    1000     // ±1000 dps
#define IMU_ODR_HZ        208      // sensor ODR (we sample it at IMU_RATE_HZ)

#define IMU_RATE_HZ       200      // host-facing IMU stream rate
#define IMU_PERIOD_US     (1000000UL / IMU_RATE_HZ)   // 5000 us

#define AUDIO_FS_HZ       16000    // PDM sample rate
#define AUDIO_BLOCK       256      // int16 samples per PDM callback (verified)

// Nominal baud. Ignored by USB CDC, but set high anyway for tools that read it.
#define USB_BAUD          1000000

// ----------------------------------------------------------------------------
// Packet types
// ----------------------------------------------------------------------------
#define PKT_IMU     0x01
#define PKT_AUDIO   0x02
#define PKT_STATUS  0x03
#define PKT_CONFIG  0x10

#define SYNC0 0xAA
#define SYNC1 0x55

// ----------------------------------------------------------------------------
// IMU
// ----------------------------------------------------------------------------
LSM6DS3 myIMU(I2C_MODE, 0x6A);
bool imuOK = false;

// ----------------------------------------------------------------------------
// PDM audio ring buffer.
//   The PDM callback fires ~62.5x/sec and must NOT block. It copies its block
//   into a ring so the main loop can transmit it without the callback
//   overwriting data mid-send (prevents torn audio blocks).
// ----------------------------------------------------------------------------
#define AUDIO_RING_SLOTS  8
static int16_t  audioRing[AUDIO_RING_SLOTS][AUDIO_BLOCK];
static uint32_t audioRingT[AUDIO_RING_SLOTS];   // block timestamp (us)
static uint16_t audioRingN[AUDIO_RING_SLOTS];   // sample count in slot
static volatile uint8_t  ringHead = 0;          // written by callback
static volatile uint8_t  ringTail = 0;          // read by loop()
static volatile uint32_t audioOverflows = 0;    // callback had nowhere to write

// Scratch buffer the callback reads PDM into before copying to the ring.
static int16_t pdmScratch[AUDIO_BLOCK * 2];

// ----------------------------------------------------------------------------
// Sequence counters (per type) — let the host detect dropped packets per stream
// ----------------------------------------------------------------------------
static uint16_t seqIMU = 0;
static uint16_t seqAudio = 0;
static uint16_t seqStatus = 0;

static uint32_t imuCount = 0;
static uint32_t audioCount = 0;

// ----------------------------------------------------------------------------
// CRC16-CCITT (poly 0x1021, init 0xFFFF). Matches host implementation.
// ----------------------------------------------------------------------------
static inline uint16_t crc16_update(uint16_t crc, uint8_t b) {
  crc ^= (uint16_t)b << 8;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
    else              crc <<= 1;
  }
  return crc;
}

// ----------------------------------------------------------------------------
// sendFrame: write one framed packet. CRC covers type+seq+len+payload.
// ----------------------------------------------------------------------------
static void sendFrame(uint8_t type, uint16_t seq, const uint8_t *payload, uint16_t len) {
  uint8_t header[7];
  header[0] = SYNC0;
  header[1] = SYNC1;
  header[2] = type;
  header[3] = (uint8_t)(seq & 0xFF);
  header[4] = (uint8_t)(seq >> 8);
  header[5] = (uint8_t)(len & 0xFF);
  header[6] = (uint8_t)(len >> 8);

  // CRC over header[2..6] then payload
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 2; i < 7; i++) crc = crc16_update(crc, header[i]);
  for (uint16_t i = 0; i < len; i++) crc = crc16_update(crc, payload[i]);

  uint8_t crcbuf[2] = { (uint8_t)(crc & 0xFF), (uint8_t)(crc >> 8) };

  Serial.write(header, 7);
  if (len) Serial.write(payload, len);
  Serial.write(crcbuf, 2);
}

// ----------------------------------------------------------------------------
// CONFIG packet: tells the host everything it needs to decode + scale the data.
//   Sent at boot, and again on request (host sends a 'C' byte).
// Layout (LE):
//   u8  fw_major
//   u8  fw_minor
//   u16 imu_rate_hz
//   u16 audio_fs_hz
//   u16 audio_block
//   u16 accel_range_g
//   u16 gyro_range_dps
//   float accel_scale_g_per_lsb
//   float gyro_scale_dps_per_lsb
// ----------------------------------------------------------------------------
static float accelScale;   // g per LSB
static float gyroScale;    // dps per LSB

static void computeScales() {
  // LSM6DS3 datasheet sensitivities
  switch (ACCEL_RANGE_G) {
    case 2:  accelScale = 0.061e-3f; break;
    case 4:  accelScale = 0.122e-3f; break;
    case 8:  accelScale = 0.244e-3f; break;
    case 16: accelScale = 0.488e-3f; break;
    default: accelScale = 0.122e-3f; break;
  }
  switch (GYRO_RANGE_DPS) {
    case 125:  gyroScale = 4.375e-3f; break;
    case 245:  gyroScale = 8.75e-3f;  break;
    case 500:  gyroScale = 17.50e-3f; break;
    case 1000: gyroScale = 35.0e-3f;  break;
    case 2000: gyroScale = 70.0e-3f;  break;
    default:   gyroScale = 35.0e-3f;  break;
  }
}

static void sendConfig() {
  uint8_t p[24];
  uint16_t i = 0;
  p[i++] = FW_VERSION_MAJOR;
  p[i++] = FW_VERSION_MINOR;
  uint16_t v;
  v = IMU_RATE_HZ;      p[i++] = v & 0xFF; p[i++] = v >> 8;
  v = AUDIO_FS_HZ;      p[i++] = v & 0xFF; p[i++] = v >> 8;
  v = AUDIO_BLOCK;      p[i++] = v & 0xFF; p[i++] = v >> 8;
  v = ACCEL_RANGE_G;    p[i++] = v & 0xFF; p[i++] = v >> 8;
  v = GYRO_RANGE_DPS;   p[i++] = v & 0xFF; p[i++] = v >> 8;
  memcpy(&p[i], &accelScale, 4); i += 4;
  memcpy(&p[i], &gyroScale, 4);  i += 4;
  sendFrame(PKT_CONFIG, 0, p, i);
}

// ----------------------------------------------------------------------------
// PDM callback — KNOWN-WORKING callback architecture, preserved.
//   Reads the available samples and drops them into the ring buffer with a
//   timestamp. Kept minimal so it returns quickly.
// ----------------------------------------------------------------------------
void onPDMdata() {
  int bytesAvailable = PDM.available();
  if (bytesAvailable <= 0) return;

  int nsamples = bytesAvailable / 2;
  if (nsamples > AUDIO_BLOCK) nsamples = AUDIO_BLOCK;   // clamp to slot size

  // Timestamp the block. Convention: t_us = time the block was handed to us
  // (END of the block). The host reconstructs per-sample times by counting
  // backward: sample k time = t_us - (n-1-k)*(1e6/fs).
  uint32_t t = micros();

  uint8_t head = ringHead;
  uint8_t next = (head + 1) % AUDIO_RING_SLOTS;
  if (next == ringTail) {
    // Ring full: host/USB not draining fast enough. Record overflow, drop.
    audioOverflows++;
    // Still must read to clear the PDM FIFO, otherwise it stalls.
    PDM.read(pdmScratch, bytesAvailable);
    return;
  }

  PDM.read(audioRing[head], nsamples * 2);
  audioRingT[head] = t;
  audioRingN[head] = (uint16_t)nsamples;
  ringHead = next;
}

// ----------------------------------------------------------------------------
// setup
// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(USB_BAUD);
  // Give USB CDC time to enumerate; do NOT hard-block forever if host absent.
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 2000)) { }

  computeScales();

  // ---- IMU ----
  myIMU.settings.accelRange       = ACCEL_RANGE_G;
  myIMU.settings.gyroRange        = GYRO_RANGE_DPS;
  myIMU.settings.accelSampleRate  = IMU_ODR_HZ;
  myIMU.settings.gyroSampleRate   = IMU_ODR_HZ;
  myIMU.settings.accelBandWidth   = 100;
  imuOK = (myIMU.begin() == 0);

  // ---- PDM mic (callback architecture, verified working) ----
  PDM.onReceive(onPDMdata);
  PDM.begin(1, AUDIO_FS_HZ);   // 1 channel, 16 kHz
  PDM.setGain(30);

  // Announce configuration to the host.
  sendConfig();
}

// ----------------------------------------------------------------------------
// loop — sample IMU on schedule, drain audio ring, emit heartbeat, handle cmds
// ----------------------------------------------------------------------------
static uint32_t nextImuUs = 0;
static uint32_t lastStatusMs = 0;

void loop() {
  uint32_t now = micros();

  // ---- Host commands (single-byte) ----
  //   'C' -> resend CONFIG   (host uses this to (re)sync decoding)
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c == 'C') sendConfig();
  }

  // ---- IMU at fixed 200 Hz ----
  if ((int32_t)(now - nextImuUs) >= 0) {
    nextImuUs = (nextImuUs == 0) ? now + IMU_PERIOD_US : nextImuUs + IMU_PERIOD_US;
    // Guard against fall-behind (e.g. after USB stall): resync schedule.
    if ((int32_t)(now - nextImuUs) > (int32_t)(4 * IMU_PERIOD_US)) nextImuUs = now + IMU_PERIOD_US;

    // RAW int16 register counts — no scaling on device.
    int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    if (imuOK) {
      ax = myIMU.readRawAccelX(); ay = myIMU.readRawAccelY(); az = myIMU.readRawAccelZ();
      gx = myIMU.readRawGyroX();  gy = myIMU.readRawGyroY();  gz = myIMU.readRawGyroZ();
    }

    uint8_t p[16];
    uint32_t ts = micros();
    memcpy(&p[0],  &ts, 4);
    memcpy(&p[4],  &ax, 2); memcpy(&p[6],  &ay, 2); memcpy(&p[8],  &az, 2);
    memcpy(&p[10], &gx, 2); memcpy(&p[12], &gy, 2); memcpy(&p[14], &gz, 2);
    sendFrame(PKT_IMU, seqIMU++, p, 16);
    imuCount++;
  }

  // ---- Drain audio ring (one block per loop pass is plenty at 62.5 blk/s) ----
  while (ringTail != ringHead) {
    uint8_t tail = ringTail;
    uint16_t n = audioRingN[tail];
    uint32_t t = audioRingT[tail];

    // payload: u32 t_us | u16 n_samples | int16[n]
    static uint8_t abuf[6 + AUDIO_BLOCK * 2];
    memcpy(&abuf[0], &t, 4);
    memcpy(&abuf[4], &n, 2);
    memcpy(&abuf[6], audioRing[tail], n * 2);
    sendFrame(PKT_AUDIO, seqAudio++, abuf, 6 + n * 2);
    audioCount++;

    ringTail = (tail + 1) % AUDIO_RING_SLOTS;
  }

  // ---- Heartbeat / status at ~1 Hz ----
  uint32_t nowMs = millis();
  if (nowMs - lastStatusMs >= 1000) {
    lastStatusMs = nowMs;
    uint8_t p[20];
    uint32_t ts = micros();
    uint32_t ovf = audioOverflows;
    memcpy(&p[0],  &ts, 4);
    memcpy(&p[4],  &imuCount, 4);
    memcpy(&p[8],  &audioCount, 4);
    memcpy(&p[12], &ovf, 4);
    uint8_t flags = (imuOK ? 0x01 : 0x00);
    memcpy(&p[16], &flags, 1);
    uint8_t pad[3] = {0,0,0}; memcpy(&p[17], pad, 3);
    sendFrame(PKT_STATUS, seqStatus++, p, 20);
  }
}
