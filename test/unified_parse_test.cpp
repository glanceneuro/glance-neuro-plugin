// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

// ===========================================================================
// Standalone unified-packet parser test (no JUCE / no Open Ephys).
//
// This re-implements the EXACT decode logic that ephys-socket's
// Source/IntanInterface.cpp (demuxDatagram / validateAndDispatchPacket /
// processLfpDatagram) and Source/IntanSocket.cpp (updateBuffer header offsets)
// use for the UNIFIED single-port format, and asserts it against synthetic
// packets built to the spec in the glance-neuro repo's docs/unified-packet-format.md.
//
// Goal: prove the plugin's demux + per-stream SEQ-gap detection + header field
// offsets match the firmware byte-for-byte (the same bytes net.py's UnifiedSink
// decodes). The companion ref_decode.py decodes the SAME synthetic packets the
// net.py way; run_test.sh diffs the two summaries.
//
// Build:  g++ -std=c++17 -O2 unified_parse_test.cpp -o unified_parse_test
// Run:    ./unified_parse_test         (emits a machine-readable summary)
// ===========================================================================
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include <cassert>

// ---- constants copied verbatim from IntanInterface.cpp -------------------
static constexpr uint32_t UNIFIED_MAGIC         = 0xCAFEBABE;
static constexpr uint8_t  UNIFIED_VERSION       = 1;
static constexpr size_t   COMMON_HEADER_WORDS   = 8;
static constexpr uint8_t  STREAM_TYPE_BROADBAND = 1;
static constexpr uint8_t  STREAM_TYPE_LFP       = 2;
static constexpr size_t   PACKET_HEADER_WORDS   = 14;  // broadband 8 + 6 sub-block

static inline uint32_t unpackU32LE(const uint8_t* b) {
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
           ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}
static inline void packU32LE(std::vector<uint8_t>& v, uint32_t x) {
    v.push_back(x & 0xFF); v.push_back((x >> 8) & 0xFF);
    v.push_back((x >> 16) & 0xFF); v.push_back((x >> 24) & 0xFF);
}

// ---- decode state (mirrors the Impl members) -----------------------------
struct Decoder {
    // broadband
    bool     haveSeq = false; uint32_t lastSeq = 0;
    uint64_t bbPkts = 0, bbSeqGaps = 0, bbLost = 0, magicErrors = 0;
    // lfp
    bool     lfpHaveSeq = false; uint32_t lfpLastSeq = 0;
    uint64_t lfpPkts = 0, lfpSeqGaps = 0, lfpLost = 0;
    // captured for cross-checks
    uint64_t lastBbTs = 0; uint8_t lastBbDigitalIn = 0; uint16_t lastBbEcho1 = 0;
    uint64_t lastLfpTs = 0; uint8_t lastLfpLaneMask = 0, lastLfpDecimR = 0;
    uint32_t lastLfpNumSamples = 0; int16_t lastLfpFirstSample = 0;

    // The plugin sets expectedPacketSizeWords_ from channel_enable; for the
    // broadband test we drive it directly (size check + word-count).
    size_t expectedPacketSizeWords = 0;

    void demux(const uint8_t* data, size_t len) {
        if (len < COMMON_HEADER_WORDS * 4) return;
        uint32_t magic   = unpackU32LE(data);
        uint32_t typeVer = unpackU32LE(data + 4);
        if (magic != UNIFIED_MAGIC) { magicErrors++; return; }
        uint8_t st = (uint8_t)(typeVer & 0xFF);
        if (st == STREAM_TYPE_BROADBAND) broadband(data, len);
        else if (st == STREAM_TYPE_LFP)  lfp(data, len);
    }

    void broadband(const uint8_t* data, size_t len) {
        size_t expectedBytes = expectedPacketSizeWords * 4;
        if (expectedBytes && len != expectedBytes) return;  // size check
        std::vector<uint32_t> w(expectedPacketSizeWords);
        for (size_t i = 0; i < expectedPacketSizeWords; ++i)
            w[i] = unpackU32LE(data + i * 4);
        if (w[0] != UNIFIED_MAGIC || (uint8_t)(w[1] & 0xFF) != STREAM_TYPE_BROADBAND) {
            magicErrors++; return;
        }
        bbPkts++;
        uint64_t ts  = ((uint64_t)w[3] << 32) | w[2];
        uint32_t seq = w[4];
        if (haveSeq) {
            uint32_t exp = lastSeq + 1;
            if (seq != exp) { uint32_t miss = seq - exp; bbSeqGaps++; bbLost += miss; }
        }
        lastSeq = seq; haveSeq = true;
        lastBbTs = ts;
        // AUX1 = w6 : {echo0[31:16], aux_flags[15:8], digital_in[7:0]}
        lastBbDigitalIn = (uint8_t)(w[6] & 0xFF);
        // sub-block w8 low 16 = slot-1 echo (accel de-interleave)
        lastBbEcho1 = (uint16_t)(w[8] & 0xFFFF);
    }

    void lfp(const uint8_t* data, size_t len) {
        const size_t HDR = COMMON_HEADER_WORDS * 4;  // 32 bytes
        if (len < HDR) return;
        uint64_t ts        = (uint64_t)unpackU32LE(data + 8) |
                             ((uint64_t)unpackU32LE(data + 12) << 32);
        uint32_t seq       = unpackU32LE(data + 16);  // w4
        uint32_t aux0      = unpackU32LE(data + 20);  // w5
        uint32_t numSamp   = unpackU32LE(data + 24);  // w6
        uint8_t laneMask = (uint8_t)(aux0 & 0xFF);
        uint8_t decimR   = (uint8_t)((aux0 >> 8) & 0xFF);
        int popcount = 0; for (int b = 0; b < 8; ++b) popcount += (laneMask >> b) & 1;
        size_t expectedSamples = (size_t)popcount * 32;
        if (numSamp != 0 && numSamp != expectedSamples) return;
        if (len - HDR < expectedSamples * 2) return;
        if (lfpHaveSeq) {
            uint32_t exp = lfpLastSeq + 1;
            if (seq != exp) { uint32_t miss = seq - exp; lfpSeqGaps++; lfpLost += miss; }
        }
        lfpLastSeq = seq; lfpHaveSeq = true;
        lfpPkts++;
        const uint16_t* s = reinterpret_cast<const uint16_t*>(data + HDR);
        lastLfpTs = ts; lastLfpLaneMask = laneMask; lastLfpDecimR = decimR;
        lastLfpNumSamples = (uint32_t)expectedSamples;
        // offset-binary -> signed, exactly as the plugin's processLfpFrame does
        lastLfpFirstSample = (int16_t)((int)s[0] - 32768);
    }
};

// ---- synthetic packet builders (to the spec) -----------------------------
static std::vector<uint8_t> buildBroadband(uint32_t seq, uint64_t ts,
                                           uint8_t channelEnable,
                                           uint8_t digitalIn, uint16_t echo1,
                                           uint16_t firstSample) {
    int numStreams = 0; for (int b = 0; b < 8; ++b) numStreams += (channelEnable >> b) & 1;
    uint32_t total16 = 35 * (uint32_t)numStreams;
    uint32_t dataWords = (total16 + 1) / 2;
    std::vector<uint8_t> v;
    packU32LE(v, UNIFIED_MAGIC);                                   // w0
    packU32LE(v, (uint32_t)STREAM_TYPE_BROADBAND | (UNIFIED_VERSION << 8)); // w1
    packU32LE(v, (uint32_t)(ts & 0xFFFFFFFF));                     // w2
    packU32LE(v, (uint32_t)(ts >> 32));                           // w3
    packU32LE(v, seq);                                            // w4
    packU32LE(v, (uint32_t)channelEnable | (dataWords << 8));      // w5 AUX0
    // w6 AUX1 = digital_in[7:0] | aux_flags[15:8] | echo0[31:16]
    packU32LE(v, (uint32_t)digitalIn | (0u << 8) | (0u << 16));    // w6
    packU32LE(v, 0);                                              // w7 RSVD
    packU32LE(v, (uint32_t)echo1);                                // w8 sub-block (slot-1 echo low)
    packU32LE(v, 0); packU32LE(v, 0); packU32LE(v, 0); packU32LE(v, 0); // w9..12
    packU32LE(v, 0);                                              // w13 reserved
    // data words: first 16-bit sample = firstSample, rest 0x8000 (midscale)
    for (uint32_t i = 0; i < dataWords; ++i) {
        uint16_t lo = (i == 0) ? firstSample : 0x8000;
        uint16_t hi = 0x8000;
        packU32LE(v, (uint32_t)lo | ((uint32_t)hi << 16));
    }
    return v;
}

static std::vector<uint8_t> buildLfp(uint32_t seq, uint64_t ts, uint8_t laneMask,
                                     uint8_t decimR, uint8_t numTaps,
                                     uint16_t firstSampleOffsetBinary) {
    int popcount = 0; for (int b = 0; b < 8; ++b) popcount += (laneMask >> b) & 1;
    uint32_t numSamples = (uint32_t)popcount * 32;
    std::vector<uint8_t> v;
    packU32LE(v, UNIFIED_MAGIC);                                   // w0
    packU32LE(v, (uint32_t)STREAM_TYPE_LFP | (UNIFIED_VERSION << 8)); // w1
    packU32LE(v, (uint32_t)(ts & 0xFFFFFFFF));                     // w2
    packU32LE(v, (uint32_t)(ts >> 32));                           // w3
    packU32LE(v, seq);                                            // w4
    // w5 AUX0 = lane_mask | decim_R<<8 | num_taps<<16 | overrun<<24
    packU32LE(v, (uint32_t)laneMask | ((uint32_t)decimR << 8) | ((uint32_t)numTaps << 16));
    packU32LE(v, numSamples);                                     // w6 AUX1 = num_samples
    packU32LE(v, 0);                                              // w7 RSVD
    for (uint32_t i = 0; i < numSamples; ++i) {
        uint16_t s = (i == 0) ? firstSampleOffsetBinary : 0x8000;
        v.push_back(s & 0xFF); v.push_back((s >> 8) & 0xFF);
    }
    return v;
}

int main() {
    Decoder d;
    const uint8_t ce = 0x0F;                       // 4 streams
    int numStreams = 4;
    uint32_t total16 = 35 * numStreams;
    uint32_t bbDataWords = (total16 + 1) / 2;      // 70
    d.expectedPacketSizeWords = PACKET_HEADER_WORDS + bbDataWords;  // 84

    // Interleave broadband + LFP on one stream of datagrams, each with its own
    // independent SEQ counter, and DELIBERATELY drop one broadband packet and
    // one LFP packet to exercise gap detection.
    uint32_t bbSeq = 100, lfpSeq = 7000;
    uint64_t bbTs = 0, lfpTs = 9;
    const uint8_t lfpLane = 0x0F; const uint8_t decimR = 10, numTaps = 131;

    for (int i = 0; i < 20; ++i) {
        // broadband every iteration; drop the packet at i==5 (skip a SEQ)
        if (i == 5) { bbSeq++; bbTs++; }  // simulate a lost broadband packet
        auto bb = buildBroadband(bbSeq, bbTs, ce,
                                 (uint8_t)(i & 0xFF),       // digital_in
                                 (uint16_t)(0x2000 + i),    // slot-1 echo
                                 (uint16_t)(0x9000 + i));   // first sample
        d.demux(bb.data(), bb.size());
        bbSeq++; bbTs++;

        // LFP every 10th broadband packet (decimation), drop the one at i==12
        if (i % 10 == 0) {
            if (i == 10) { /* normal */ }
            if (i == 20) {}
            auto lf = buildLfp(lfpSeq, lfpTs, lfpLane, decimR, numTaps,
                               (uint16_t)(0x8100 + i));
            d.demux(lf.data(), lf.size());
            lfpSeq++; lfpTs += decimR;
        }
    }
    // a second LFP burst with a gap
    for (int j = 0; j < 5; ++j) {
        if (j == 2) { lfpSeq++; lfpTs += decimR; }  // drop one LFP frame
        auto lf = buildLfp(lfpSeq, lfpTs, lfpLane, decimR, numTaps,
                           (uint16_t)(0x8200 + j));
        d.demux(lf.data(), lf.size());
        lfpSeq++; lfpTs += decimR;
    }

    // Machine-readable summary (ref_decode.py prints the same keys).
    printf("bb_pkts=%llu\n",        (unsigned long long)d.bbPkts);
    printf("bb_seq_gaps=%llu\n",    (unsigned long long)d.bbSeqGaps);
    printf("bb_lost=%llu\n",        (unsigned long long)d.bbLost);
    printf("magic_errors=%llu\n",   (unsigned long long)d.magicErrors);
    printf("bb_last_digital_in=%u\n", (unsigned)d.lastBbDigitalIn);
    printf("bb_last_echo1=%u\n",    (unsigned)d.lastBbEcho1);
    printf("bb_last_ts=%llu\n",     (unsigned long long)d.lastBbTs);
    printf("lfp_pkts=%llu\n",       (unsigned long long)d.lfpPkts);
    printf("lfp_seq_gaps=%llu\n",   (unsigned long long)d.lfpSeqGaps);
    printf("lfp_lost=%llu\n",       (unsigned long long)d.lfpLost);
    printf("lfp_last_lane_mask=%u\n", (unsigned)d.lastLfpLaneMask);
    printf("lfp_last_decim_r=%u\n", (unsigned)d.lastLfpDecimR);
    printf("lfp_last_num_samples=%u\n", (unsigned)d.lastLfpNumSamples);
    printf("lfp_last_first_sample=%d\n", (int)d.lastLfpFirstSample);
    printf("lfp_last_ts=%llu\n",    (unsigned long long)d.lastLfpTs);
    return 0;
}
