#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

# Reference decoder for the unified-packet parser test. Builds the SAME
# synthetic broadband + LFP packets as unified_parse_test.cpp and decodes them
# exactly the way mz-unified-ports/remote/net.py does (UnifiedSink demux,
# DataValidator broadband SEQ check, receive_lfp LFP header + offset-binary
# samples). Emits the same machine-readable summary keys so run_test.sh can diff
# the C++ plugin logic against the firmware's host reference byte-for-byte.
import struct

UNIFIED_MAGIC = 0xCAFEBABE
UNIFIED_VERSION = 1
COMMON_HEADER_WORDS = 8
STREAM_TYPE_BROADBAND = 1
STREAM_TYPE_LFP = 2
PACKET_HEADER_WORDS = 14  # net.py BB_HEADER_WORDS


def build_broadband(seq, ts, ce, digital_in, echo1, first_sample):
    num_streams = bin(ce & 0xFF).count("1")
    data_words = (35 * num_streams + 1) // 2
    w = [0] * PACKET_HEADER_WORDS
    w[0] = UNIFIED_MAGIC
    w[1] = STREAM_TYPE_BROADBAND | (UNIFIED_VERSION << 8)
    w[2] = ts & 0xFFFFFFFF
    w[3] = (ts >> 32) & 0xFFFFFFFF
    w[4] = seq
    w[5] = (ce & 0xFF) | (data_words << 8)            # AUX0
    w[6] = digital_in & 0xFF                          # AUX1: digital_in | flags<<8 | echo0<<16
    w[7] = 0
    w[8] = echo1 & 0xFFFF                             # sub-block slot-1 echo
    # w9..w13 already 0
    data = []
    for i in range(data_words):
        lo = first_sample if i == 0 else 0x8000
        hi = 0x8000
        data.append((lo & 0xFFFF) | ((hi & 0xFFFF) << 16))
    return struct.pack(f"<{PACKET_HEADER_WORDS + data_words}I", *(w + data))


def build_lfp(seq, ts, lane_mask, decim_r, num_taps, first_sample_ob):
    popcount = bin(lane_mask & 0xFF).count("1")
    num_samples = popcount * 32
    hdr = [0] * COMMON_HEADER_WORDS
    hdr[0] = UNIFIED_MAGIC
    hdr[1] = STREAM_TYPE_LFP | (UNIFIED_VERSION << 8)
    hdr[2] = ts & 0xFFFFFFFF
    hdr[3] = (ts >> 32) & 0xFFFFFFFF
    hdr[4] = seq
    hdr[5] = (lane_mask & 0xFF) | (decim_r << 8) | (num_taps << 16)  # AUX0
    hdr[6] = num_samples                                             # AUX1
    hdr[7] = 0
    payload = bytearray()
    for i in range(num_samples):
        s = first_sample_ob if i == 0 else 0x8000
        payload += struct.pack("<H", s & 0xFFFF)
    return struct.pack("<8I", *hdr) + bytes(payload)


class Decoder:
    def __init__(self, expected_words):
        self.expected_words = expected_words
        self.have_seq = False
        self.last_seq = 0
        self.bb_pkts = self.bb_gaps = self.bb_lost = self.magic_errors = 0
        self.lfp_have = False
        self.lfp_last_seq = 0
        self.lfp_pkts = self.lfp_gaps = self.lfp_lost = 0
        self.last_bb_di = self.last_bb_echo1 = self.last_bb_ts = 0
        self.last_lfp_lane = self.last_lfp_decim = 0
        self.last_lfp_ns = self.last_lfp_ts = 0
        self.last_lfp_first = 0

    def demux(self, data):
        if len(data) < COMMON_HEADER_WORDS * 4:
            return
        magic, type_ver = struct.unpack("<II", data[:8])
        if magic != UNIFIED_MAGIC:
            self.magic_errors += 1
            return
        st = type_ver & 0xFF
        if st == STREAM_TYPE_BROADBAND:
            self.broadband(data)
        elif st == STREAM_TYPE_LFP:
            self.lfp(data)

    def broadband(self, data):
        if len(data) != self.expected_words * 4:
            return
        w = struct.unpack(f"<{self.expected_words}I", data)
        if w[0] != UNIFIED_MAGIC or (w[1] & 0xFF) != STREAM_TYPE_BROADBAND:
            self.magic_errors += 1
            return
        self.bb_pkts += 1
        ts = (w[3] << 32) | w[2]
        seq = w[4]
        if self.have_seq:
            exp = (self.last_seq + 1) & 0xFFFFFFFF
            if seq != exp:
                miss = (seq - exp) & 0xFFFFFFFF
                self.bb_gaps += 1
                self.bb_lost += miss
        self.last_seq = seq
        self.have_seq = True
        self.last_bb_ts = ts
        self.last_bb_di = w[6] & 0xFF
        self.last_bb_echo1 = w[8] & 0xFFFF

    def lfp(self, data):
        HDR = COMMON_HEADER_WORDS * 4
        magic, type_ver, ts_lo, ts_hi, seq, cfg, num_samples, rsvd = \
            struct.unpack("<8I", data[:HDR])
        lane_mask = cfg & 0xFF
        decim_r = (cfg >> 8) & 0xFF
        popcount = bin(lane_mask).count("1")
        expected = popcount * 32
        if num_samples != 0 and num_samples != expected:
            return
        if len(data) - HDR < expected * 2:
            return
        if self.lfp_have:
            exp = (self.lfp_last_seq + 1) & 0xFFFFFFFF
            if seq != exp:
                miss = (seq - exp) & 0xFFFFFFFF
                self.lfp_gaps += 1
                self.lfp_lost += miss
        self.lfp_last_seq = seq
        self.lfp_have = True
        self.lfp_pkts += 1
        ts = ts_lo | (ts_hi << 32)
        s0 = struct.unpack_from("<H", data, HDR)[0]
        self.last_lfp_ts = ts
        self.last_lfp_lane = lane_mask
        self.last_lfp_decim = decim_r
        self.last_lfp_ns = expected
        self.last_lfp_first = s0 - 0x8000   # offset binary -> signed


def main():
    ce = 0x0F
    num_streams = 4
    bb_data_words = (35 * num_streams + 1) // 2
    d = Decoder(PACKET_HEADER_WORDS + bb_data_words)

    bb_seq, lfp_seq = 100, 7000
    bb_ts, lfp_ts = 0, 9
    lfp_lane, decim_r, num_taps = 0x0F, 10, 131

    for i in range(20):
        if i == 5:
            bb_seq += 1
            bb_ts += 1
        d.demux(build_broadband(bb_seq, bb_ts, ce, i & 0xFF,
                                0x2000 + i, 0x9000 + i))
        bb_seq += 1
        bb_ts += 1
        if i % 10 == 0:
            d.demux(build_lfp(lfp_seq, lfp_ts, lfp_lane, decim_r, num_taps,
                              0x8100 + i))
            lfp_seq += 1
            lfp_ts += decim_r

    for j in range(5):
        if j == 2:
            lfp_seq += 1
            lfp_ts += decim_r
        d.demux(build_lfp(lfp_seq, lfp_ts, lfp_lane, decim_r, num_taps,
                          0x8200 + j))
        lfp_seq += 1
        lfp_ts += decim_r

    print(f"bb_pkts={d.bb_pkts}")
    print(f"bb_seq_gaps={d.bb_gaps}")
    print(f"bb_lost={d.bb_lost}")
    print(f"magic_errors={d.magic_errors}")
    print(f"bb_last_digital_in={d.last_bb_di}")
    print(f"bb_last_echo1={d.last_bb_echo1}")
    print(f"bb_last_ts={d.last_bb_ts}")
    print(f"lfp_pkts={d.lfp_pkts}")
    print(f"lfp_seq_gaps={d.lfp_gaps}")
    print(f"lfp_lost={d.lfp_lost}")
    print(f"lfp_last_lane_mask={d.last_lfp_lane}")
    print(f"lfp_last_decim_r={d.last_lfp_decim}")
    print(f"lfp_last_num_samples={d.last_lfp_ns}")
    print(f"lfp_last_first_sample={d.last_lfp_first}")
    print(f"lfp_last_ts={d.last_lfp_ts}")


if __name__ == "__main__":
    main()
