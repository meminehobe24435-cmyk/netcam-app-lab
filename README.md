# netcam-app-lab

**An IPC (network camera) application/protocol layer, simulated on a PC in
C99, with every number in this file produced by the code in this repository.**

`netcam-app-lab` implements the part of a network camera that is easy to get
wrong and worth testing: the RTSP session state machine, RTP packetisation and
receiver reordering, FU-A style media fragmentation and reassembly, a motion
detection / alarm state machine, and a recording index with ring-buffer
eviction. The end-to-end simulation runs the whole chain:

```
RTSP session -> media fragmentation -> RTP packetisation
     -> network (loss / reorder / duplicate) -> receiver reordering
     -> NAL reassembly -> motion detection -> recording index
```

* **C99, standard library only** — no dependencies, no build system beyond
  `make`.
* **`make test`** runs 1387 self-checking assertions.
* **`make sim`** runs the end-to-end simulation and writes `results/`.
* Deterministic: the same seed gives the same numbers on Linux, macOS and
  Windows, which is why the tables below are reproducible.

---

## Why this exists

I wanted a project that demonstrates IPC *application and protocol layer*
reasoning rather than hardware or DSP work, because that is where the
interesting failure modes are on a real device:

* an RTSP session that answers `200 OK` for a `PLAY` it should have refused
  with `455` is a bug every NVR client will find;
* a CSeq that is not checked lets a replayed or reordered request mutate state
  twice;
* a reorder window that is one packet too small silently reports *zero* loss
  while dropping data — the worst possible outcome for a monitoring product;
* a reassembler that emits a NAL unit stitched together from non-adjacent
  fragments produces a picture that looks almost right;
* a motion detector without debounce fires on sensor noise, and one with a
  badly tuned cooldown misses the second half of an event.

Each of those is a test in `test/test_netcam.c`, and each of them is a bug I
actually hit while writing this (see [Bugs the tests
caught](#bugs-the-tests-caught)).

---

## Quick start

```sh
make            # build build/sim and build/test_netcam
make test       # build and run the self-checking suite (1387 assertions)
make sim        # build and run the end-to-end simulation, writes results/
make clean
```

`make test` and `make sim` each build the target they need; run `make` first if
you want both binaries.

Requires a C99 compiler. On Windows with MSYS2 UCRT64 and GNU make 3.80:

```sh
make CC=g++.exe
```

(If `CC` names a C++ driver such as `g++`, the Makefile adds `-x c`
automatically; that driver rejects `-std=c99` otherwise.)

`results/` is created by the C code itself (`ntc_mkdir`), not by `make` —
`make.exe` on Windows has no portable `mkdir -p`, and the build deliberately
does not need one.

---

## Measured results

Everything below was printed by `make test` and `make sim` on Windows/UCRT64,
gcc 15.2.0, `-O2`, `-ffp-contract=off`. Nothing here is estimated.

### Test suite

| Metric | Value |
|---|---|
| Assertions in `test/test_netcam.c` | **1387** |
| Failures | **0** |
| Pipeline determinism digest (two runs, whole chain) | `17392601112560187409` (identical) |

### Module 1 — RTSP session layer

| Measurement | Value |
|---|---|
| Methods accepted in their legal state | 200 `OK` |
| `PLAY` without a `Session` header | **454** `Session Not Found` |
| `PLAY` with an unknown session id | **454** |
| Second `SETUP` while `READY` | **455** `Method Not Valid In This State` |
| `PAUSE` while `READY` | **455** |
| `PLAY` while already `PLAYING` | **455** |
| `SETUP` while `PLAYING` | **455** |
| `SETUP` without a usable `Transport` header | **400** `Bad Request` |
| Replayed / out-of-order `CSeq` | **400** — 4 rejections in the focused test, 1 in the full request sequence |
| Unknown method (`RECORD`) | **405** `Method Not Allowed` |
| Session table exhausted | **503** `Service Unavailable` |
| Two sessions, independent `CSeq` counters | 1 replay rejected in session A, session B unaffected; 1 session survives after the other is torn down |
| State machine after rejected requests | unchanged (rejections never advance the state) |

The state machine is: `INIT --SETUP--> READY --PLAY--> PLAYING --PAUSE--> READY`,
`TEARDOWN` from `READY`/`PLAYING`. `OPTIONS` and `DESCRIBE` are legal in all
three states. The full 6 × 4 method/state legality matrix is asserted
entry by entry.

### Module 2 — RTP reordering, loss and duplicate detection

400 datagrams, 5 % loss / 3 % duplication / 25 % reordering, deterministic seed
`20240301`. The same run at five reorder window sizes:

| Window | Delivered | Declared lost | Late | Duplicates detected | Out-of-order arrivals |
|---:|---:|---:|---:|---:|---:|
| 1 | 314 | 86 | 65 | 14 | 0 |
| 4 | 367 | 33 | 12 | 14 | 129 |
| 16 | 376 | 24 | 3 | 14 | 222 |
| 64 | 377 | 23 | 2 | 14 | 304 |
| 512 | 379 | 21 | 0 | 14 | 374 |

Reading this table is the point of the module:

* the sender lost **21** datagrams (the 5 % loss) — that is the floor;
* with a window of 512 the receiver declares **21** losses and **0** late
  arrivals: exactly right;
* with a window of 1 it declares **86** losses and **65** late arrivals while
  delivering 314: a window that cannot span the reordering turns ordinary
  out-of-order arrivals into both claimed losses and dropped packets;
* duplicate detection is **14** at every window size — correct, since the
  network duplicated 14 datagrams.

At the measured operating point (window 16): **376 of 400 delivered (94.0 %)**,
24 declared lost, 3 late, 14 duplicates detected, 222 out-of-order arrivals
all repaired into ascending order.

### Module 3 — NAL fragmentation and reassembly

A 3000-byte NAL unit at a 1200-byte packet budget:

| Measurement | Value |
|---|---|
| Fragments produced | 3 |
| Bytes of fragment data | 2999 (the NAL header byte is rebuilt from the FU bytes, so it is not carried as data) |
| Clean in-order reassembly | **byte-exact**, 3000/3000 bytes match |
| Reverse arrival order | 2 fragments refused, unit never completed, nothing emitted |
| One middle fragment lost | **1 unit dropped**, 0 units completed, 0 bytes emitted |
| One middle fragment duplicated | unit still reassembles, byte-exact; 1 duplicate counted |
| Unfragmented (single-packet) NAL | passes straight through, 1 unit |
| Unit larger than the reassembly buffer | overflow reported, 1 drop, nothing emitted |
| Hand-written 3-fragment FU-A vector | header rebuilt as `0x65`, four data bytes recovered exactly |

**End-to-end (30 frames, 10 % loss / 4 % duplication / 35 % reordering, window 8):**

| Measurement | Value |
|---|---|
| Datagrams sent | 90 |
| Reached the receiver | 80 |
| Delivered to the application | 77 |
| Declared lost / late | 13 / 0 |
| Duplicates detected | 2 |
| NAL units reassembled | **18** |
| NAL units dropped (a fragment missing) | **7** |
| Bytes reassembled | 54 000 |
| **Frames that were byte-exact** | **18 of 18 completed** |
| **Frames containing corrupted content** | **0** |

That last row is the invariant that matters: a reassembled frame is either
bit-identical to what was sent, or it is dropped. There is no in-between.

### Module 4 — Motion detection and alarm state machine

A scripted 60-frame sequence with known ground truth: **2 real motion events**
and **3 single-frame noise blips**.

| Detector | Alarms raised | False positives | Missed events |
|---|---:|---:|---:|
| Threshold only, no debounce | 5 | **3** | 0 |
| Debounced (3 frames to raise, 5 to clear) | **2** | **0** | **0** |

Debouncing removed all 3 noise alarms without losing either real event
(23 mover frames were classified identically in both configurations).

| Other measurements | Value |
|---|---|
| Alarms with the default 25-frame cooldown (two bursts 10 frames apart) | 1 raised, 1 suppressed as an alarm storm |
| Alarm-to-recording linkage | 2 record triggers, 2 stream triggers |
| A blip shorter than `min_event_frames` | 1 event, 1 recorded as suppressed |
| Background adaptation, ratio after 40 frames on a static scene | 0.0833 |

### Module 5 — Recording index and ring-buffer eviction

30 clips of 100 bytes into a 1000-byte budget:

| Measurement | Value |
|---|---|
| Inserts | 30 |
| Evictions | **20** |
| Bytes reclaimed | 2000 |
| Segments surviving | 10 |
| Bytes used after the run | 1000 (exactly the budget) |
| Oldest surviving id / newest id | 21 / 30 |
| Query `[0, 1e9)` hits | 10 (all survivors) |
| Query `[0, 50000)` hits | 0 (those clips were evicted) |
| Segment larger than the whole budget | refused, not silently dropped |

Retrieval uses the documented half-open rule `seg.end > from && seg.start < to`;
a zero-width window matches nothing, and the boundary cases
(`[2000,3000)` vs a segment ending at 2000, `[500,1000)` vs one starting at
1000) are asserted explicitly.

---

## What this is NOT

Read this before judging the numbers.

* **Pure logic and numeric simulation.** There is no camera sensor, no ISP, no
  image pipeline, no real H.264/HEVC encoder or decoder, no sockets, no real
  network, no hardware, no timing guarantees, no threads and no memory
  bandwidth measurements. Everything runs on a PC as a single-threaded process.
* **The protocols are a common subset of the public specifications.** RTSP
  implements the six methods `OPTIONS` / `DESCRIBE` / `SETUP` / `PLAY` /
  `PAUSE` / `TEARDOWN` with CSeq, Session and Transport handling. There is
  **no** full SDP negotiation (the `DESCRIBE` body is a fixed stub), no
  authentication or digest, no SRTP, no TLS, no `REDIRECT`, no keep-alive
  timers, no `RTP-Info` handling, no interleaved-vs-UDP transport switching at
  runtime.
* **RTP implements the fixed 12-byte header** (V/P/X/CC, M, PT, sequence
  number, timestamp, SSRC), CSRC lists, header extension skipping and padding.
  There is **no RTCP**: no SR/RR reports, no receiver reports, no NACK/PLI/FIR,
  no jitter calculation, no RTCP-driven bitrate adaptation.
* **The NAL layer is FU-A style, not H.264.** Fragmentation and reassembly use
  the FU indicator / FU header shape from RFC 6184. SPS/PPS are not parsed,
  slice syntax is not interpreted, there is no decoder, and the payloads are
  synthetic bytes rather than a real bitstream. STAP-A and MTAP are not
  implemented.
* **Not product code.** It has not run on IPC hardware, has not been through
  interoperability testing, and is not a drop-in component for anything.
* **Not done, deliberately:** real IPC interoperability (NVR / client / cloud
  platforms), P2P traversal, ONVIF, GB28181, alarm I/O hardware, night vision,
  WDR and other image-quality work, lens/ISP tuning, OTA update, and anything
  involving a real sensor.

One honest limitation is recorded in the test suite rather than hidden: with a
**very small reorder window** the receiver can report a sequence position both
as a declared loss and as a later late arrival, so the reported total can
exceed an externally counted gap count. The invariant that is asserted is
`external_gaps <= reported_lost + reported_late`; the over-reporting is a
property of the window-pressure path and is visible in the window table above
(window 1: 86 losses + 65 late versus 86 external gaps).

---

## Repository layout

```
src/netcam.h      the public interface: every struct, status code and function
                  contract, with the wire formats documented at the top of each
                  module section
src/rtsp.c        RTSP parsing, CSeq/Session validation, session state machine
src/rtp.c         RTP header pack/parse, sender packetiser, receiver reorder
                  buffer, loss and duplicate detection
src/nal.c         FU-A fragmentation and reassembly
src/motion.c      frame/block difference, debounce, alarm state machine
src/record.c      recording index, time-range retrieval, ring-buffer eviction
src/util.c        status strings, directory creation, report writers, PRNG,
                  simulated network impairment
src/main_sim.c    end-to-end simulation, writes results/
test/test_netcam.c  1387 assertions, including hand-assembled byte vectors
```

The header is the delivery contract: it was written first and in full, so any
of the `.c` files can be reimplemented against it without reading the others.

---

## Bugs the tests caught

These are real defects found by this suite during development, in the order
they were found. They are the reason the suite is worth more than its
assertion count.

### 1. The first packet of a session was classified as an ancient duplicate

**Symptom** `PLAY` worked, packets arrived, and the receiver delivered nothing
while reporting `duplicates` instead of `lost`.

**Root cause** two mistakes in one function. The reorder window was
*established* from the first packet only after the "is this packet older than
what we expect?" test had already run — so the first packet was compared against
a meaningless `expected_seq == 0`, and in modular arithmetic `(uint16_t)(0 -
1001)` is 65289, i.e. "older". Separately the window-initialisation path marked
that same packet as *seen*, so the duplicate detector then rejected the very
packet it had just accepted.

**Fix** establish the window before any comparison, and never mark a packet as
seen before it has been stored. Both rules are now comments at the site, and
the case is pinned by a test that pushes a single packet into a fresh receiver
and asserts `delivered == 1`.

### 2. The reorder buffer never drained

**Symptom** `held` grew with every packet, `delivered` stayed at zero, and the
400-packet run reported 0 % delivery with no errors.

**Root cause** the drain condition was `... && !ntc_rx_ready(seq)`, where
`ntc_rx_ready()` returned true for a packet that was *held and not yet
delivered* — precisely the packet that had just been stored. The negation made
the loop unreachable, and because the helper was named after a state it did not
actually test (`ready` meant "ready for the queue", not "queued"), the double
negative hid the mistake.

**Fix** the predicate is now `ntc_rx_queued()` — it tests exactly "this slot
holds a packet already moved to the delivery queue" — so the condition reads as
"drain while the next packet is held and not queued". Renaming it removed the
double negative that concealed the bug. A single-push test that pops and checks
the delivered sequence number now guards the whole path.

### 3. The network model duplicated pointers and aborted the process

**Symptom** the test binary died with a stack-cookie abort
(`0xC0000409` / exit `-1073740940`) the moment the impaired datagram array was
freed. Printing the pointer array showed slots 0, 1 and 2 all holding the *same*
address.

**Root cause** the reordering pass used a hand-written "shift right" helper
whose loop copied each element onto itself instead of shifting it down from its
predecessor, so one pointer propagated into every slot in between. The
duplication pass then stored a *copy of a copy* and the loss pass nulled a slot
before reading it, losing the original. Three aliasing errors in one function,
all invisible until something freed the result.

**Fix** the impairment model was rewritten with an explicit, separated structure
(keep-or-free, then insert copies, then move one element later by walking from
the front) and a direct unit assertion that the array is a permutation with no
NULL holes and no repeated pointer. The final test now checks the pointer set
before freeing, so a regression fails as an assertion rather than as a crash.

### 4. The fragmenter re-sent the NAL header byte as payload

**Symptom** a 3000-byte NAL unit reassembled into 3001 bytes, and every
fragment count was one too high.

**Root cause** the sender's fragment loop started at `src[0]`. Byte 0 of a NAL
unit *is* the NAL header, and FU-A deliberately does not carry it as data — the
receiver rebuilds it from the FU indicator and FU header. Emitting it as data
added one byte per fragmented unit.

**Fix** fragment data now starts at `src[1]` in both the sender and the
standalone fragmenter. The test asserts a byte-exact round trip on a 3000-byte
unit, so an off-by-one in either direction fails immediately.

### 5. The loss counter reported more losses than packets sent

**Symptom** a 400-packet stream reported `lost = 510` and `lost = 763` in
successive revisions, i.e. more losses than the sender ever transmitted.

**Root cause** two compounding errors. A "rebuild the seen map from the ring"
step after every flush discarded the marks for sequence numbers that had already
been declared lost, so the next flush counted them again; and the flush walk
declared losses all the way up to a buffered sequence number that belonged to an
older revolution of the ring.

**Fix** losses are recorded in their own map, a sequence number enters the
accounting exactly once (either as a delivery or as a loss), the walk is bounded
by the highest sequence number actually observed, and the seen map is never
reconstructed from the ring. An external, independent gap count is computed in
the test and compared against the counter, which is what makes the fix
verifiable rather than self-referential.

### 6. Two smaller ones worth recording

* **RTSP method tokens were matched case-insensitively.** `play` was accepted as
  `PLAY`. Method tokens are case sensitive and header field names are not; the
  parser now uses `strcmp` for the method and a case-insensitive comparison for
  header names, and the test contains a lowercase-method vector.
* **`expect()` always reported its own line number.** The macro expanded
  `__LINE__` inside the helper, so every failure in the suite printed the same
  line and a failing check could not be located. The macros now take the call
  site's `__LINE__` as a parameter.

---

## Pitfalls hit while building this

Collected from the build, the tests and the release process. Several of these
cost more time than the protocol code did.

1. **GNU make 3.80 mishandles an order-only prerequisite on a pattern rule.**
   `build/%.o: src/%.c | build` produced `Circular ... dependency dropped` and
   then ran the compile with no output directory present. The object directory
   is now created at parse time with a guarded `$(shell mkdir ...)`. Related:
   redirections such as `2>/dev/null` are not portable across the shells make
   may use on Windows — a bare `mkdir` is.

2. **A C++ driver needs to be told it is compiling C.** With `CC=g++.exe`,
   `-std=c99` is rejected (`valid for C/ObjC but not for C++`). The Makefile
   detects `++` in the compiler basename and adds `-x c`.

3. **`-ffp-contract=off` is not optional.** Without it macOS clang forms FMAs at
   `-O2` where Linux gcc does not, so a floating-point assertion that passes on
   one host fails on the other. Every compile line carries the flag.

4. **Large structs on the stack abort rather than fail.** Two motion detectors
   (about 47 KB each) plus several datagram lists (about 25 KB each) as locals
   overflowed the default 1 MB Windows stack; the process died with a
   stack-cookie abort and *no test output at all*, because buffered `stdout` was
   lost. They are now file-scope objects, and the suite calls
   `setvbuf(stdout, NULL, _IONBF, 0)` so a hard crash still shows how far it got.

5. **`-Werror` turns a silent test-harness bug into a build failure — and that
   is a feature, but only if you look.** A debug harness that was missing
   `#include <stdlib.h>` failed to link, and the command re-ran the *previous*
   binary, which "reproduced" a bug that had already been fixed. Filtering
   compiler output for the word `error` hid the `undefined reference`. Always
   confirm the artefact's timestamp, not just the presence of a file.

6. **A test that only agrees with itself proves nothing.** Building a packet
   with the project's own packer and parsing it with the project's own parser
   passes even when both are wrong in the same direction. The RTSP, RTP and NAL
   sections each contain at least one vector written out byte by byte from the
   specification, and the modular sequence comparison is checked against a
   brute-force oracle over 3002 pairs before being accepted.

7. **Do not let a macro hide the call site.** See bug 6 above: `__LINE__`
   expanded inside a helper reports the helper's line for every failure.

8. **Reseeding a generator per iteration makes a channel look perfect.**
   `ntc_net_apply()` seeds from the profile, so calling it once per frame with a
   constant seed gave every frame *exactly the same* losses — which, with
   three-fragment frames, meant the same fragment every time and a 0 % drop rate
   over 30 frames. The simulation now advances the seed per frame.

9. **Windows text mode corrupts binary data.** `0x1A` in a stream opened
   without `"b"` is treated as end-of-file. Everything this project writes uses
   explicit `"wb"` mode and its own `\n`.

10. **Never pipe a program's output into `head` in CI.** The reader exits, the
    writer takes `SIGPIPE`, and the job goes red for a reason that has nothing
    to do with the code. The CI writes to a file first and then reads it.

---

## Licence

MIT. See `LICENSE`.
