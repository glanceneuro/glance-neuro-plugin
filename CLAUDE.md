# CLAUDE.md

Working notes for this repo. `README.md` says what the plugin is and
[`docs/building.md`](docs/building.md) says how to build it — this file is the part
that is easy to get wrong.

## What this is

An Open Ephys `DataThread` source plugin for the GLANCE MicroZed/Zynq-7020 acquisition
board. It publishes two streams: **broadband** at 30 kHz and a decimated **LFP** band at
3 kHz, both arriving on one UDP port and demultiplexed by `stream_type` in the common
header.

`IntanInterface.{h,cpp}` is a standalone C++ client for the board protocol with no JUCE
dependency. It is the **third consumer of the register/packet contract**, after the
firmware and `remote/net.py` in
[glance-neuro](https://github.com/glanceneuro/glance-neuro). Changing the contract means
changing all three, plus that repo's `docs/protocol.md` and `docs/register-map.md`.
Nothing enforces the plugin's side automatically — the firmware and `net.py` are kept
honest by a `_Static_assert` on the status struct, and this repo is checked by hand.

## Architecture

Two threads, deliberately:

- **receive thread** — pulls datagrams off the socket into a ring and does nothing else
- **demux thread** — pops the ring, reads `stream_type`, routes broadband and LFP

The split exists so downstream work can never hold up the socket. `net.py` has no such
buffer — it demuxes inline on one thread — so **the plugin staying clean while `net.py`
shows loss proves nothing about the board.** That asymmetry has misled diagnosis before.

`sourceBuffers` is owned by this plugin, not auto-managed by Open Ephys. The constructor
creates `[0]` for broadband only; `[1]` is added or resized for LFP when a stream is
configured.

## Hard rules

**1. Every path that can change the board's lane mask must refresh the LFP geometry.**
The board derives the LFP lane mask from the broadband channel-enable, so anything that
rewrites the channel mask — connect, LFP enable, **auto-detection** — changes the size of
the LFP frames arriving. The frame consumer drops any frame whose `sampleCount` disagrees
with `lfp_num_channels`, so a stale count **silently discards the entire band**: zeros,
not an error, and nothing in the log.

All such paths go through `applyLfpStatus()`, and `lfp_num_channels` is assigned in
exactly one function. Adding a fourth path means calling it. This is not hypothetical —
rescanning with LFP enabled produced a dead band until `applyDetectionConfig()` was
taught to refresh.

**2. Filter configuration does not belong here.** The cascade's coefficients are loaded
from the bitstream at boot and its decimation is structural. There is nothing for a
client to set up, and a client that uploads taps can only push a filter the hardware
cannot run. An older build did exactly that and corrupted the halfband stage.

**3. Don't add per-packet work to the receive path.** It runs 30,000 times a second. A
copy, an allocation, or a lock there is paid at that rate.

**4. Keep the GPL-3 SPDX headers.** Every source file carries them. The upstream dev repo
(`ckemere/ephys-socket`) does **not**, so copying a file verbatim from there silently
strips the header — re-add it.

## Gotchas

- **Open Ephys appends its own `-A`/`-B` suffix** per stream. The base names here are
  `BroadbandStream` and `LFPStream`, so `BroadbandStream-A` is correct and expected.
  Useful as a build check: `IntanStream-A` means an old build is loaded.
- Stream **channel order follows the mask's bit order** (low→high, port A then B), the
  same packing the firmware uses. LFP channels mirror the broadband layout with an `LFP_`
  prefix.
- Samples are **offset binary** — subtract `0x8000` for signed.
- Because of the 2-command SPI readback pipeline, amplifier channel `ch` arrives at
  **cycle `ch + 2`**; cycles 0–1 and the aux cycles are not amplifier data.
- The board keeps streaming when the TCP control connection drops — that is deliberate,
  so a transient disconnect does not abort acquisition and reconnection is meaningful.
  Don't "fix" it by stopping the stream on disconnect.

## Testing

```bash
bash test/run_test.sh        # no JUCE, no Open Ephys, no board needed
```

`test/` holds a **differential** test of the one thing worth guarding here: it builds
synthetic unified packets, decodes them with a standalone copy of the plugin's decode
logic (`unified_parse_test.cpp`) and with a `net.py`-style reference decoder
(`ref_decode.py`), and diffs the two. Any disagreement about header fields, demux, or
per-stream SEQ-gap detection fails the run. It guards the wire-format contract rather
than an implementation, which is why it earns its keep — keep it in step when the decode
changes.

There is deliberately nothing beyond that. The rest of this plugin is I/O against a live
board or the GUI, and a mock would assert only that the mock matches itself.

Beyond the parser test, validate against real data:

- `remote/net.py` in the glance-neuro repo is the **reference decoder**. When the plugin
  and `net.py` disagree about the same stream, `net.py` is almost certainly right — it
  derives frame sizes from the packet header rather than from cached state, which is
  exactly the class of bug this plugin is prone to.
- A clean run shows **0 SEQ gaps** on both streams.
- Before believing any packet-loss report, run the host gate in that repo
  (`python3 remote/netperf_loopback.py 30 20 154 0`). A degraded host has repeatedly
  looked like a plugin or firmware bug.

When the plugin shows a stream as **zeros rather than noise**, suspect a silent drop
before suspecting the board: a geometry mismatch discards frames without logging
anything.

## Conventions

- `Build/` is generated and gitignored. Never commit it.
- Plain forward commits; don't rebase shared branches.
- Commit or push only when asked. No AI attribution trailers.
- Upstream dev work happens in `ckemere/ephys-socket`; this repo is the public GPL-3
  release. Keep the two in step when publishing.
