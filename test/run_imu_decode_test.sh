#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Rice University
#
# Cross-repo check: generate IMU datagrams with the FIRMWARE's host
# test (glance-neuro/firmware/test-host) and decode them with the plugin's
# decoder. Fails if the two repos ever disagree on the wire format.
#
#   test/run_imu_decode_test.sh [path/to/glance-neuro]
#
# Defaults to ../glance-neuro next to this repo. Needs only a C/C++ compiler.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
NEURO="${1:-../glance-neuro}"

if [ ! -x "$NEURO/firmware/test-host/run_imu_stream_test.sh" ]; then
    echo "SKIP: glance-neuro not found at '$NEURO'" >&2
    echo "      pass its path: test/run_imu_decode_test.sh /path/to/glance-neuro" >&2
    exit 2
fi

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

echo "generating datagrams from the firmware host test ..."
IMU_TEST_DUMP="$WORK/imu_pkts.bin" "$NEURO/firmware/test-host/run_imu_stream_test.sh" \
    | tail -2

echo "decoding them with the plugin decoder ..."
c++ -std=c++17 -O1 -Wall -Wextra -Werror test/test_imu_decode.cpp -o "$WORK/test_imu_decode"
"$WORK/test_imu_decode" "$WORK/imu_pkts.bin"
