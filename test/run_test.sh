#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

# Build + run the standalone unified-packet parser test and diff the plugin's
# C++ decode logic against the net.py-style Python reference decode of the SAME
# synthetic packets. Exit non-zero on any mismatch.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
cd "$here"

echo "== building C++ harness =="
g++ -std=c++17 -O2 -Wall -Wextra unified_parse_test.cpp -o unified_parse_test

echo "== C++ (plugin decode logic) =="
./unified_parse_test | tee cpp_out.txt

echo "== Python (net.py reference decode) =="
python3 ref_decode.py | tee py_out.txt

echo "== diff =="
if diff -u py_out.txt cpp_out.txt; then
    echo "PASS: plugin decode matches net.py reference byte-for-byte"
else
    echo "FAIL: mismatch between plugin decode and net.py reference"
    exit 1
fi
