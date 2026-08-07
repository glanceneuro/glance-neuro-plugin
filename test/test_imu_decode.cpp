// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Rice University
//
// Cross-check for the plugin's IMU (stream_type = 3) decoder.
//
// The plugin is the THIRD implementation of the packet layout, after the firmware
// and remote/net.py in glance-neuro, and nothing enforces its side
// automatically (CLAUDE.md). This test closes that gap for the IMU stream: it
// reads datagrams produced by the FIRMWARE's own host-test binary
// (glance-neuro/firmware/test-host, run with IMU_TEST_DUMP) and decodes them
// with the same field offsets and scale factors IntanInterface.cpp uses. If
// the firmware's wire format ever moves, this fails instead of silently
// producing wrong physical units in Open Ephys.
//
// Build + run:  test/run_imu_decode_test.sh [path/to/glance-neuro]
// No JUCE, no Open Ephys, no board.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>

namespace {

// --- mirrored verbatim from Source/IntanInterface.cpp -----------------------
constexpr uint32_t UNIFIED_MAGIC       = 0xCAFEBABE;
constexpr size_t   COMMON_HEADER_WORDS = 8;
constexpr uint8_t  STREAM_TYPE_IMU     = 3;
constexpr size_t   IMU_PACKET_BYTES    = 52;
constexpr float    BNO055_QUAT_SCALE   = 1.0f / 16384.0f;
constexpr float    BNO055_ACCEL_SCALE  = 1.0f / 100.0f;
constexpr float    BNO055_GYRO_SCALE   = 1.0f / 16.0f;

uint32_t unpackU32LE(const uint8_t* b) {
    return b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24);
}
int16_t unpackI16LE(const uint8_t* b) {
    return (int16_t)(uint16_t)(b[0] | (b[1] << 8));
}

struct ImuSample {
    uint64_t timestamp; uint32_t sequence; int port;
    float quat[4], accel[3], gyro[3];
    uint8_t calibStatus, operatingMode; int8_t temperatureC;
    uint16_t periodMs; uint8_t iicErrors, sendDrops;
};

// The decode body from IntanInterface::Impl::processImuDatagram, minus the
// stats/callback plumbing (which needs the whole class).
bool decodeImu(const uint8_t* data, size_t len, ImuSample& s) {
    if (len < IMU_PACKET_BYTES) return false;
    if (unpackU32LE(data) != UNIFIED_MAGIC) return false;
    uint32_t typeVer = unpackU32LE(data + 4);
    if ((typeVer & 0xFF) != STREAM_TYPE_IMU) return false;
    constexpr size_t HDR = COMMON_HEADER_WORDS * 4;

    s.port     = (int)((typeVer >> 16) & 1);
    s.timestamp = (uint64_t)unpackU32LE(data + 8) |
                  ((uint64_t)unpackU32LE(data + 12) << 32);
    s.sequence = unpackU32LE(data + 16);
    uint32_t aux0 = unpackU32LE(data + 20), aux1 = unpackU32LE(data + 24);
    s.periodMs      = (uint16_t)(aux0 & 0xFFFF);
    s.iicErrors     = (uint8_t)((aux0 >> 16) & 0xFF);
    s.sendDrops     = (uint8_t)((aux0 >> 24) & 0xFF);
    s.calibStatus   = (uint8_t)(aux1 & 0xFF);
    s.operatingMode = (uint8_t)((aux1 >> 8) & 0xFF);
    s.temperatureC  = (int8_t)((aux1 >> 16) & 0xFF);
    for (int i = 0; i < 4; ++i)
        s.quat[i]  = (float)unpackI16LE(data + HDR + 2 * i) * BNO055_QUAT_SCALE;
    for (int i = 0; i < 3; ++i)
        s.accel[i] = (float)unpackI16LE(data + HDR + 8 + 2 * i) * BNO055_ACCEL_SCALE;
    for (int i = 0; i < 3; ++i)
        s.gyro[i]  = (float)unpackI16LE(data + HDR + 14 + 2 * i) * BNO055_GYRO_SCALE;
    return true;
}

// --- expectations: mirror mock_reset() in the firmware's imu_host_mock.c ----
int16_t mockQuat(int p, int i) { return (int16_t)(0x4000 - i * 1000 - p * 7); }
int16_t mockAcc (int p, int i) { return (int16_t)(-981 + i * 100 + p * 3); }
int16_t mockGyr (int p, int i) { return (int16_t)(160 * (i - 1) + p); }

int failures = 0;
void check(bool cond, const std::string& what) {
    if (!cond) { ++failures; std::printf("  FAIL %s\n", what.c_str()); }
}
bool close(float a, float b) { return std::fabs(a - b) < 1e-6f; }

} // namespace

int main(int argc, char** argv) {
    const char* dump = (argc > 1) ? argv[1] : "imu_pkts.bin";
    std::FILE* f = std::fopen(dump, "rb");
    if (!f) {
        std::printf("TB_FAIL  cannot open %s (run via run_imu_decode_test.sh)\n", dump);
        return 1;
    }
    std::vector<uint8_t> raw;
    uint8_t buf[4096];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) raw.insert(raw.end(), buf, buf + n);
    std::fclose(f);

    size_t off = 0, count = 0, perPort[2] = {0, 0};
    bool haveSeq[2] = {false, false};
    uint32_t lastSeq[2] = {0, 0};

    while (off + 2 <= raw.size()) {
        uint16_t len = (uint16_t)(raw[off] | (raw[off + 1] << 8));
        if (off + 2 + len > raw.size()) break;
        const uint8_t* p = raw.data() + off + 2;
        off += 2 + len;

        check(len == IMU_PACKET_BYTES, "packet length is 52 bytes");
        ImuSample s;
        if (!decodeImu(p, len, s)) { check(false, "decode failed"); continue; }
        ++count; ++perPort[s.port];

        // Per-port SEQ continuity -- the plugin's loss check depends on this
        // being per port, not global.
        if (haveSeq[s.port])
            check(s.sequence == lastSeq[s.port] + 1, "per-port SEQ continuity");
        haveSeq[s.port] = true;
        lastSeq[s.port] = s.sequence;

        // Engineering units: the whole point of the plugin-side scaling.
        for (int i = 0; i < 4; ++i)
            check(close(s.quat[i], mockQuat(s.port, i) / 16384.0f), "quaternion scale");
        for (int i = 0; i < 3; ++i) {
            check(close(s.accel[i], mockAcc(s.port, i) / 100.0f), "accel scale (m/s^2)");
            check(close(s.gyro[i],  mockGyr(s.port, i) / 16.0f),  "gyro scale (deg/s)");
        }
        // A negative accel component must survive as negative (the sign-extension
        // path that a naive shift would get wrong).
        check(s.accel[0] < 0.0f, "negative accel stays negative");
        check(s.timestamp == (((uint64_t)0x2 << 32) | 0xDEAD0001), "PL timestamp");
        check(s.periodMs == 10, "period_ms");
        if (s.sequence >= 1) {
            check(s.operatingMode == 0x0C, "OPR_MODE reports NDOF");
            check(s.calibStatus == 0xC3, "calib_stat");
            check(s.temperatureC == (int8_t)(23 + s.port), "die temperature");
        }
    }

    check(count >= 18, "enough packets decoded");
    check(perPort[0] >= 9 && perPort[1] >= 9, "both ports represented");
    std::printf("  decoded %zu firmware datagrams (A=%zu B=%zu)\n",
                count, perPort[0], perPort[1]);

    if (failures) { std::printf("TB_FAIL  Errors: %d\n", failures); return 1; }
    std::printf("TB_PASS  Errors: 0\n");
    return 0;
}
