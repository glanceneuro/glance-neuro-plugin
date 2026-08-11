# Latency, and tuning it for closed loop

For visualization and recording the defaults are fine. For **closed-loop work** —
a ripple detector that has to act within a millisecond or two — the default Open
Ephys settings are the limiting factor, not this plugin, and the fix is one
setting in the GUI.

## Set the audio buffer size first

Open Ephys paces its **entire processing graph** off the audio device callback.
Every processor in the signal chain, including whatever you write to close the
loop, runs once per callback. The interval is:

```
callback interval = audio buffer size / audio sample rate
```

The GUI defaults to **1024 samples at 44.1 kHz = 23 ms**. Nothing you do in this
plugin can beat that number, because your detector does not run until the callback
fires.

**Click the Latency button (the clock icon) in the Control Panel** and lower the
buffer size. What that buys, at 30 kHz acquisition:

| audio buffer | callback interval | broadband samples per callback |
|---|---|---|
| 1024 (default) | 23.2 ms | ~697 |
| 512 | 11.6 ms | ~348 |
| 256 | 5.8 ms | ~174 |
| 128 | 2.9 ms | ~87 |
| **64** | **1.45 ms** | **~44** |
| 32 | 0.73 ms | ~22 |

The same window sets the audio **sample rate**; raising it shortens the interval
for a given buffer size too (128 @ 96 kHz = 1.3 ms).

Smaller is not free. Each callback carries fixed per-block overhead across every
processor in the chain, so at 256 channels a very small buffer can starve the
audio thread and cause dropouts — which show up as gaps, not as slowness. Step
down one setting at a time and watch the latency line described below. Pick the
largest buffer that meets your deadline.

## Reading `[GLANCE][LATENCY]`

Every 5 seconds the plugin prints, e.g.:

```
[GLANCE][LATENCY] plugin=38 us mean / 210 us max, sourceBuffer=44 samp (1.5 ms), dataQueue=0/120000
```

Three separate things, deliberately together:

- **`plugin=`** — UDP socket to demux decode. **This is ours.** Tens of
  microseconds is normal. If this climbs, the problem is in this plugin or in host
  CPU contention starving its threads.
- **`sourceBuffer=`** — how much data is waiting for Open Ephys to collect it.
  This is *paced by the audio callback above*, not by us, and it is the term that
  dominates closed-loop latency. Expect it to sawtooth: near zero right after a
  callback, up to roughly one callback's worth just before the next. Occasional
  peaks of two callbacks' worth mean a late callback, which is normal under load.
- **`dataQueue=`** — the queue into the record path. Should sit at zero. If it
  climbs, recording is not keeping up (disk), which is a different problem from
  latency.

**A worked example of what "healthy" looks like at the default setting:**
`sourceBuffer` oscillating between ~20 and ~1215 samples is exactly the 23 ms
callback at work — 1215 samples is 40 ms, i.e. not quite two callback intervals.
Nothing is wrong with the data path, but no closed loop can run faster than that
until the buffer size comes down.

## The rest of the chain

Worth knowing so you can rule it out. From electrode to your detector:

| stage | cost | notes |
|---|---|---|
| board: one 30 kHz sample per UDP datagram | ~0 | never batched — see the acquisition repo's hard rule 2 |
| network | ~0.1 ms | gigabit, one hop |
| plugin: socket → demux → source buffer | tens of µs | the `plugin=` figure |
| **Open Ephys: waiting for the process callback** | **buffer / sample rate** | **the dominant term** |
| your processor in the signal chain | yours | |

The board deliberately sends one datagram per sample rather than batching, so
there is no acquisition-side buffering to remove: the latency budget is spent
almost entirely in the last two rows.

## If it still is not fast enough

- Check `plugin=` first. If it is large, this plugin (or the host) is at fault and
  the audio buffer is a red herring.
- Put the detector as **early** in the signal chain as possible; every processor
  ahead of it adds its own work to the same callback.
- A wired connection and a host that is not also doing heavy disk work will both
  reduce late callbacks, which are what turn a 1.5 ms budget into a 3 ms one.
