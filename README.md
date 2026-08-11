<div align="center">

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="Resources/logo-darkmode.png">
  <img src="Resources/logo.png" alt="GLANCE — Gigabit Low-latency Acquisition for Neuroscience & Closed-loop Experiments" width="680">
</picture>

# GLANCE — Open Ephys plugin

by the [Kemere Lab](https://kemerelab.com) at [Rice University](https://neuroengineering.rice.edu)

[![Linux](https://github.com/glanceneuro/glance-neuro-plugin/actions/workflows/linux.yml/badge.svg?branch=main)](https://github.com/glanceneuro/glance-neuro-plugin/actions/workflows/linux.yml) [![macOS](https://github.com/glanceneuro/glance-neuro-plugin/actions/workflows/mac.yml/badge.svg?branch=main)](https://github.com/glanceneuro/glance-neuro-plugin/actions/workflows/mac.yml) [![Windows](https://github.com/glanceneuro/glance-neuro-plugin/actions/workflows/windows.yml/badge.svg?branch=main)](https://github.com/glanceneuro/glance-neuro-plugin/actions/workflows/windows.yml)

</div>

An Open Ephys GUI `DataThread` source plugin for the **GLANCE** MicroZed/Zynq-7020 Intan
acquisition system ([glance-neuro](https://github.com/glanceneuro/glance-neuro)).
Forked from [Ephys Socket](https://github.com/open-ephys-plugins/ephys-socket) by Jonathan
Newman ([@jonnew](https://github.com/jonnew)); this fork replaces the generic
matrix-over-TCP source with the GLANCE device protocol:

- **TCP control** (port 0x6900 / 26880): start/stop, configuration, chip auto-detection,
  and the aux-sequencer commands. `IntanInterface.{h,cpp}` is a standalone C++ client for
  this protocol (no JUCE dependency) — the third implementation of the packet and register layout,
  after the firmware and `remote/net.py`.
- **UDP data** (port 0x6800 / 26624): one packet per ~30 kHz sample; a per-stream header
  plus up to 140 data words depending on the channel-enable mask.

<p align="center">
  <img src="ephys-socket.png" width="80%" />
</p>

The board firmware, the `remote/net.py` reference client, and this plugin are the
**three implementations of the same packet and register layout** — see the
[glance-neuro](https://github.com/glanceneuro/glance-neuro) repo when changing the protocol.

## Usage

1. **Flash the board** with a current GLANCE image (`blobs/BOOT.bin` from the
   [glance-neuro](https://github.com/glanceneuro/glance-neuro) repo) and put it on the
   network. Default device IP is `192.168.18.10`; put your host on the same subnet
   (port 0x6900 TCP control, 0x6800 UDP data).
2. **Install the plugin** (see [docs/building.md](docs/building.md)) so
   OpenEphys finds it in its `plugins` directory.
3. In OpenEphys, open the **Processor List → Sources** and drag **Intan Socket**
   in as the signal-chain source.
4. In the editor, set **Device IP** (and TCP/UDP ports if non-default), then
   click **CONNECT**. The editor mirrors the device's current state —
   chip indicators, DBG button, aux flags — so reconnecting after a successful
   RESCAN restores everything for free.
5. Click **RESCAN** to auto-detect connected chips on both port A and port B
   in parallel (one sweep over 16 cable-phase values, ce=0xFF) and pick the
   optimal phase per port. The channel count updates to match.
6. Press **play** to stream. Neural channels appear as `A_CH1…` / `B_CH1…`,
   aux inputs as `A_AUX0_1…` / `B_AUX0_1…` (per port × CIPO line). Click
   **STATUS** at any time to dump full device state to the console (View →
   **Console**, or Shift+C in a Release build).
7. To exercise the run-time features: **SETTLE** toggles amplifier fast
   settle and DSP reset together; the **TTL Settle** dropdown makes both
   follow a digital-input pin; **AUX SEQ** switches the aux slots to the
   banked accelerometer/housekeeping programs (and, toggled while streaming,
   performs a live bank swap).

A connected **headstage accelerometer** (auxin1/2/3) shows up on the AUX
channels — select the **AUX** channel type in the LFP viewer's range selector
to see it at the right scale.

> ⚠️ **Don't run `net.py` at the same time as this plugin.** Both bind the same UDP
> data port, so whichever is up consumes the datagrams. If `net.py` is running, the
> plugin will still **CONNECT and look fine** (control is over TCP) but its data
> packets get **swallowed by `net.py`** — the channels appear but show no (or partial)
> data. Quit `net.py` before pressing **play**.

## Further reading

- [docs/building.md](docs/building.md) — Windows / Linux / macOS build instructions.
- [docs/latency.md](docs/latency.md) — tuning for closed-loop work. **Read this before
  building a ripple detector**: Open Ephys paces the whole signal chain off its audio
  callback, 23 ms by default, and that is the dominant term — not this plugin.

## Attribution

Original Ephys Socket plugin by Jonathan Newman ([@jonnew](https://github.com/jonnew)) —
[open-ephys-plugins/ephys-socket](https://github.com/open-ephys-plugins/ephys-socket).
GLANCE fork by the Kemere Lab, Rice University. Licensed under [GPL-3.0](LICENSE).
