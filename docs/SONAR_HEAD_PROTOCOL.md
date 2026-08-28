# PingDSP sonar-head / Control protocol (Örebro RE)

Distilled for implementers from fleet `orebro-sonar-head-re` (chair FINAL 2026-07-30).  
Evidence and skeptic trail: [docs/re/orebro-sonar-head/README.md](re/orebro-sonar-head/README.md).

**Scope:** offline RE of network dumps + Control CLI. **Out of scope:** CAATI / sample-body codec / processing algorithms. **Head TX** (commands *to* the head) was **not** captured on Örebro mirrors.

---

## Three planes (do not mix)

| Plane | Endpoints (Örebro LAN) | Transport | Purpose |
|-------|------------------------|-----------|---------|
| Control CLI | Client ↔ `192.168.228.50:23840` | TCP ASCII CRLF | Drive Windows 3DSS-DX Control |
| Processed API | `192.168.228.50:23848` → client | TCP binary | `DxHeader` + `DxData` (existing driver) |
| Raw sample RX | `192.168.228.1` → `239.81.2.{136,137}:24330` | UDP multicast | Fixed 1082 B `a101` sample blocks |

- TCP **`:23848` ≠** UDP **`:24330`**. Different magic, endpoints, and roles.
- Publisher `.1` is the raw-plane source (head or head-gateway — identity not proven).
- On Örebro mirrors: **no IP packets to `.1`**; Control never unicasts/multicasts commands on the captured NIC.

---

## Control CLI (`:23840`)

### Framing

1. Connect TCP to Control `:23840`.
2. Read/discard initial UTF-8 BOM (`EF BB BF`, 3 bytes, no CRLF).
3. Request: `UTF-8("<verb> [args...]\\r\\n")`.
4. Response: one CRLF line; leading token `okay` or `error`.

No length prefix, TLV, or TLS. Matches `pingdsp_driver` `sonar_control_node.send_command`.

### Verbs seen on wire

Control is sparse vs data. Prefer **`0807_bag6.pcap`** for CLI vocabulary (69 `:23840` frames); bag5 is the short setup dump (18 frames).

| Dump | Requests seen | Typical reply |
|------|---------------|----------------|
| bag5 | `app --init`, `sonar --connect`, `sidescan`, `sv` | `okay` / `error (already connected)` |
| **bag6** | `acquisition`, `sidescan3d`, `sv` (repeated GETs) | `okay (dutycycle=…)` / `okay port(…)` / `okay (bulk=…)` |

These bag6 lines match ROS 2 services in `sonar_control_node` (`sonar/acquisition`, `sonar/sidescan3d`, sound-velocity). SET-form Guide verbs (`gain`, `transmit`, `commit`, …) were **not** on either dump. **CLI bytes are not head wire bytes.**

### Observable `app --init` side-effect (mirror)

Not “join mcast and start RX.” On bag5: ~0.15 s later raw + processed **pause ~45 s**; Control then sends IGMP **Leave** (not Join) for the head groups; streams resume later with **no** matching CLI. Treat as correlated pause; head-TX causation unknown.

---

## Processed stream (`:23848`)

Authoritative layout is already in-tree — reuse, do not re-derive:

- `vendor/docs/3dss-dx-struct-api-0.6/pingdsp-3dss.hpp` (`kTcpPort=23848`, `kPreamble`, `DxHeader`, `DxData`)
- `pingdsp_driver/pingdsp_driver/dx_structures.py`
- `pingdsp_driver/pingdsp_driver/tcp_client.py` (`read_ping()`, `resync_to_preamble()`)
- Offline: `scripts/replay_pingdsp_pcap.py`

Wire: `[16 B preamble][4 B LE data_count][data_count B DxData]`.  
Join keys for raw: `DxData.ping_id`, `DxData.time.seconds`, `DxData.time.nanoseconds` (little-endian in TCP).

---

## Raw UDP echo (`:24330` twin mcast)

### Keep filter

```
ip.src==192.168.228.1 && udp.dstport==24330
  && (ip.dst==239.81.2.136 || ip.dst==239.81.2.137)
  && data.len==1082
```

Same UDP port to `239.81.255.255` is **status/NMEA** (`f1xx`), not samples.  
Subscribe **one** of `.136` / `.137` and dedupe (headers match aside stream id + CRC; **bodies differ** — dual channel).

### Frame = one UDP datagram

| Off | Type | Field |
|----:|------|-------|
| 0 | u16 BE | Magic `0x1234` |
| 2 | u16 BE | Port `0x5f0a` (24330) |
| 4 | u16 BE | CRC16-CCITT over bytes `[6..end)`; init `0xFFFF`, poly `0x1021` |
| 6 | u16 BE | Leg: `0x0288`→`.136`, `0x0289`→`.137` |
| 12 | u16 BE | Length = `data.len − 2` (1082 → `0x0438`) |
| 18 | u16 BE | Type **`0xa101`** (echo) |
| 24 | u16 BE | Sequence |
| 32 | u16 BE | Block index (`0,16,…,3824` → **240** blocks / ping / leg) |
| 34–37 | BE | **Ping number** (= `DxData.ping_id` value). Prefer reading as u32@34; bag5 also consistent with **u16BE@36** (ids &lt; 2¹⁶) |
| 38–41 | — | Unknown (`0x30070000`-class); not ping_id |
| 42–45 | u32 BE | `DxData.time.seconds` |
| 46–49 | u32 BE | `DxData.time.nanoseconds` |
| 50–57 | u16×4 BE | Observed `16,16,24,8` |
| 58… | — | Body — **codec unknown** (block 0: sparse table; ≥16: high entropy) |

**480** `a101` datagrams per processed ping (240 × 2 legs).

**Not a field:** ASCII “`NC=`” near `@43` — those bytes are the BE Unix-seconds word of the timestamp.

Gaps `@26–31` and `@38–41` are present; do not assume a fully closed header map.

### Join to processed

1. Reassemble TCP `:23848` → `(ping_id, time.s, time.ns)`.
2. For each filtered UDP frame, read BE time `@42`/`@46` and ping number `@34`–`@37`.
3. Join on ping number; verify time. Deduplicate legs.
4. Do **not** join on capture timestamps alone.

Acquisition rate ≈ **9.09 Hz** from embedded time deltas.

---

## Ignore for raw-echo parsers

| Flow | Why |
|------|-----|
| UDP `:24333` from `.1` | sbgECom Ellipse (`FF 5A`) — see `pingdsp_sbg` |
| UDP `:24334` | Wrapped SBG |
| UDP `:24335` | ASCII `/TRIG:…` |
| `:24330` → `239.81.255.255` | Head meta / NMEA |
| `.91` `:24338`/`24339` | Septentrio SBF |
| TCP `:23840` / `:23848` | Control / processed — separate tracks |
| DDS `:7400` | ROS 2 |

---

## Implementation checklist

1. **RX:** IGMP join `239.81.2.136` or `.137`; parse as above; optional CRC check.
2. **Correlate:** reuse existing Dx parsers; field-join to UDP.
3. **Control:** ASCII client on `:23840` for existing ops only.
4. **Do not** claim head TX / direct “talk to head” until a capture shows the command path.
5. **Defer** body codec and CAATI.

---

## Known unknowns

- Sample array layout inside `a101` body  
- Control→head command transport (off-mirror)  
- Ping packing after large `ping_id`  
- Dual-leg body semantics  
