# Örebro sonar-head reverse engineering — evidence index

Fleet run: `.fleet/runs/orebro-sonar-head-re/`  
Distilled protocol: [`docs/SONAR_HEAD_PROTOCOL.md`](../../SONAR_HEAD_PROTOCOL.md)  
Chair verdict: [`.fleet/runs/orebro-sonar-head-re/out/FINAL.md`](../../../.fleet/runs/orebro-sonar-head-re/out/FINAL.md)

Dataset: `/media/shekharu/DATA/OneDrive/SMaRC PhD/Datasets/orebro`  
- Raw/processed deep-work: `0807_bag5.pcap`  
- **Control-dense (`:23840` / ROS service GETs): `0807_bag6.pcap`** (see `out/control-dense-bag6-note.md`)
- **Sidescan share pack** (viewer + `*_sidescan.npz`): `orebro-sidescan-viewer.zip` — see [`docs/SIDESCAN_EXPORT_VIEWER.md`](../../SIDESCAN_EXPORT_VIEWER.md)

## Status: **crystallized** (chair done)

## Storage

| Kind | Path |
|------|------|
| Fleet evidence | `.fleet/runs/orebro-sonar-head-re/out/` |
| Distilled protocol | `docs/SONAR_HEAD_PROTOCOL.md` |
| This index | `docs/re/orebro-sonar-head/README.md` |

## Artifact map

Paths relative to `.fleet/runs/orebro-sonar-head-re/out/` unless noted.

### Chair / durable

| Artifact | Description |
|----------|-------------|
| [FINAL.md](../../../.fleet/runs/orebro-sonar-head-re/out/FINAL.md) | Majority-surviving claims, cmd table, raw layout, next steps |
| [`docs/SONAR_HEAD_PROTOCOL.md`](../../SONAR_HEAD_PROTOCOL.md) | Implementer-facing protocol distillate |

### Scout / tops

| Artifact | Description |
|----------|-------------|
| [inventory.md](../../../.fleet/runs/orebro-sonar-head-re/out/inventory.md) | 11-pcap catalog, slice counts, dump ranking |
| [raw-udp-report.md](../../../.fleet/runs/orebro-sonar-head-re/out/raw-udp-report.md) | UDP `:24330` / `:2433x` framing, cadence, sample vs status |
| [control-tcp-report.md](../../../.fleet/runs/orebro-sonar-head-re/out/control-tcp-report.md) | TCP `:23840` ASCII CLI; no Control→`.1` IP on mirror |
| [processed-3dss-report.md](../../../.fleet/runs/orebro-sonar-head-re/out/processed-3dss-report.md) | TCP `:23848` Dx stream; join keys to raw |
| [sbg-other-report.md](../../../.fleet/runs/orebro-sonar-head-re/out/sbg-other-report.md) | `:24333` sbgECom; Septentrio `.91`; ignore-list |

### Correlate

| Artifact | Description |
|----------|-------------|
| [cmd-translation.md](../../../.fleet/runs/orebro-sonar-head-re/out/cmd-translation.md) | Client→Control verbs + mirror side-effects (**IGMP Join row superseded by FINAL / skeptic-3**) |
| [raw-frame-notes.md](../../../.fleet/runs/orebro-sonar-head-re/out/raw-frame-notes.md) | Best-effort `a101` layout + raw↔processed join cites |

### Verify (skeptics)

| Artifact | Focus | Headline |
|----------|-------|----------|
| [skeptic-1.md](../../../.fleet/runs/orebro-sonar-head-re/out/skeptic-1.md) | Port attribution | `:23848` ≠ `:24330`; soften “`.1`=head” / bare `:24330`=echo |
| [skeptic-2.md](../../../.fleet/runs/orebro-sonar-head-re/out/skeptic-2.md) | Magic / offsets | Framing+time join stand; `ping_id` u32@34 weakened; **`NC=` device tag refuted** |
| [skeptic-3.md](../../../.fleet/runs/orebro-sonar-head-re/out/skeptic-3.md) | Cmd / causation | **`app --init`→IGMP Join/start-RX refuted** (Leave + ~45 s pause); head TX uncaptured |

### Minion fragments

| Directory | Top |
|-----------|-----|
| [fragments/raw-udp/](../../../.fleet/runs/orebro-sonar-head-re/out/fragments/raw-udp/) | raw-udp |
| [fragments/control-tcp/](../../../.fleet/runs/orebro-sonar-head-re/out/fragments/control-tcp/) | control-tcp |
| [fragments/processed-3dss/](../../../.fleet/runs/orebro-sonar-head-re/out/fragments/processed-3dss/) | processed-3dss |
| [fragments/sbg-other/](../../../.fleet/runs/orebro-sonar-head-re/out/fragments/sbg-other/) | sbg-other |

Scratch pcaps/TSVs: `out/scratch/` (not durable docs).

## Surviving takeaways (post-skeptic)

- **Control is rare:** full-file `:23840` peaks at **69 frames on bag6** (service-style `acquisition` / `sidescan3d` / `sv` GETs); bag5 only has the short init transcript.
- **Planes:** CLI `:23840` · processed `:23848` · raw twin-mcast `:24330` — never conflate.
- **Raw RX:** 1082 B `a101`, magic `12 34 5f 0a`; join via BE time `@42`/`@46` + ping number `@34`–`@37` (u16@36 may fit); 480 datagrams/ping.
- **Not a tag:** ASCII “`NC=`” is timestamp bytes.
- **No Control→`.1` IP** on these mirrors; head command path still uncaptured.
- **`app --init`:** correlated ~45 s dual-plane pause + IGMP **Leave**, not Join/start-RX.
