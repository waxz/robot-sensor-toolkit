#!/usr/bin/env python3
"""
bench_ops.py  —  Write / read operation micro-benchmark

Profiles the raw cost of ShmPublisher.write_state() and
ShmSubscriber.read_state_spin() under various conditions:

  Timestamp on/off   : heartbeat_every=1 (write_ns set) vs =0 (skipped)
  Robot count        : 1, 4, 16 robot slots
  Contention         : no-contention (single process) vs
                       reader-writer (two processes in parallel)

Metrics (n = 10 000 samples, 1 000 warmup)
  write_ns    : per-call duration of pub.write_state()
  read_ns     : per-call duration of sub.read_state_spin()

Goal: verify that write_ns timestamp adds minimal overhead and quantify
the seqlock read cost under concurrent writers.
"""

import multiprocessing as mp
import os
import sys
import time
from dataclasses import dataclass, field

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from shmbridge import _core

_N      = 10_000
_WARMUP = 1_000
_SHM    = "/bench_ops_shm"

# ── helpers ───────────────────────────────────────────────────────────────────

def _now() -> int:
    return time.monotonic_ns()


def _pct(lst: list[float], p: float) -> float:
    if not lst:
        return float("nan")
    s = sorted(lst)
    return s[max(0, int(len(s) * p / 100) - 1)]


@dataclass
class OpStats:
    label:   str = ""
    samples: list[float] = field(default_factory=list)

    def add(self, ns: float) -> None:
        self.samples.append(ns)

    @property
    def p50(self)  -> float: return _pct(self.samples, 50)
    @property
    def p99(self)  -> float: return _pct(self.samples, 99)
    @property
    def p999(self) -> float: return _pct(self.samples, 99.9)
    @property
    def mean(self) -> float:
        return sum(self.samples) / len(self.samples) if self.samples else float("nan")


# ═══════════════════════════════════════════════════════════════════════════════
# Single-process (no contention)
# ═══════════════════════════════════════════════════════════════════════════════

def bench_write(n_robots: int, heartbeat: int) -> OpStats:
    """Measure pure write_state() cost, no readers."""
    label = f"write  n_robots={n_robots}  ts={'on' if heartbeat else 'off'}"
    stat = OpStats(label)
    pub = _core.ShmPublisher(_SHM, n_robots, 1, heartbeat)
    pub.open()
    state = _core.RobotState()
    state.x = 1.0

    for i in range(_WARMUP + _N):
        t0 = _now()
        pub.write_state(i % n_robots, state)
        t1 = _now()
        if i >= _WARMUP:
            stat.add(t1 - t0)

    pub.close()
    return stat


def bench_read(n_robots: int, heartbeat: int) -> tuple[OpStats, OpStats]:
    """Measure write_state() and read_state_spin() with a single attached sub.
    The publisher writes once, the subscriber reads the same slot repeatedly
    (clean reads, no writer racing).
    """
    wstat = OpStats(f"write  n_robots={n_robots}  ts={'on' if heartbeat else 'off'}")
    rstat = OpStats(f"read   n_robots={n_robots}  ts={'on' if heartbeat else 'off'}")

    pub = _core.ShmPublisher(_SHM, n_robots, 1, heartbeat)
    pub.open()
    sub = _core.ShmSubscriber(_SHM, n_robots)
    sub.attach(5_000)

    state = _core.RobotState()

    for i in range(_WARMUP + _N):
        robot = i % n_robots

        t0 = _now()
        pub.write_state(robot, state)
        t1 = _now()

        s = sub.read_state_spin(robot, 128)
        t2 = _now()
        _ = s  # ensure not elided

        if i >= _WARMUP:
            wstat.add(t1 - t0)
            rstat.add(t2 - t1)

    sub.detach()
    pub.close()
    return wstat, rstat


# ═══════════════════════════════════════════════════════════════════════════════
# Contention: publisher + subscriber in separate processes
# ═══════════════════════════════════════════════════════════════════════════════

def _pub_worker(n_robots: int, heartbeat: int, ready: mp.Event, stop: mp.Event,
                q: mp.Queue) -> None:
    pub = _core.ShmPublisher(_SHM, n_robots, 1, heartbeat)
    pub.open()
    state = _core.RobotState()
    stat = OpStats()
    ready.set()
    i = 0
    while not stop.is_set():
        robot = i % n_robots
        t0 = _now()
        pub.write_state(robot, state)
        t1 = _now()
        if i >= _WARMUP:
            stat.add(t1 - t0)
        i += 1
    pub.close()
    q.put(("write", stat.samples))


def _sub_worker(n_robots: int, ready: mp.Event, stop: mp.Event,
                q: mp.Queue) -> None:
    ready.wait(10)
    sub = _core.ShmSubscriber(_SHM, n_robots)
    sub.attach(5_000)
    stat = OpStats()
    i = 0
    while not stop.is_set():
        robot = i % n_robots
        t0 = _now()
        s = sub.read_state_spin(robot, 128)
        t1 = _now()
        _ = s
        if i >= _WARMUP:
            stat.add(t1 - t0)
        i += 1
    sub.detach()
    q.put(("read", stat.samples))


def bench_contention(n_robots: int, heartbeat: int, duration_s: float = 2.0
                     ) -> tuple[OpStats, OpStats]:
    """Publisher and subscriber run simultaneously for `duration_s` seconds."""
    ready = mp.Event()
    stop  = mp.Event()
    q     = mp.Queue()

    pub_p = mp.Process(target=_pub_worker,
                       args=(n_robots, heartbeat, ready, stop, q), daemon=True)
    sub_p = mp.Process(target=_sub_worker,
                       args=(n_robots, ready, stop, q), daemon=True)

    pub_p.start()
    sub_p.start()
    ready.wait(5)
    time.sleep(duration_s)
    stop.set()
    pub_p.join(3)
    sub_p.join(3)

    wstat = OpStats(f"write  contention  n_robots={n_robots}  ts={'on' if heartbeat else 'off'}")
    rstat = OpStats(f"read   contention  n_robots={n_robots}  ts={'on' if heartbeat else 'off'}")

    while not q.empty():
        kind, samples = q.get_nowait()
        if kind == "write":
            wstat.samples = samples
        else:
            rstat.samples = samples

    return wstat, rstat


# ═══════════════════════════════════════════════════════════════════════════════
# Report
# ═══════════════════════════════════════════════════════════════════════════════

def _fmt_ns(v: float) -> str:
    if v != v:
        return "    N/A"
    return f"{v:7.1f}"


def _print_row(stat: OpStats) -> None:
    print(
        f"  {stat.label:<46}"
        f"  {_fmt_ns(stat.mean)} ns"
        f"  {_fmt_ns(stat.p50)} ns"
        f"  {_fmt_ns(stat.p99)} ns"
        f"  {_fmt_ns(stat.p999)} ns"
        f"  (n={len(stat.samples)})"
    )


ALL_RESULTS: list[tuple[str, list[OpStats]]] = []


def _section(title: str, stats: list[OpStats]) -> None:
    ALL_RESULTS.append((title, stats))
    print(f"\n── {title} {'─' * (60 - len(title))}")
    hdr = (
        f"  {'label':<46}"
        f"  {'mean':>8}"
        f"  {'p50':>8}"
        f"  {'p99':>8}"
        f"  {'p999':>8}"
    )
    print(hdr)
    print("  " + "-" * 84)
    for s in stats:
        _print_row(s)


def main() -> None:
    mp.set_start_method("fork", force=True)

    print(f"\n{'shmbridge operation micro-benchmark':^88}")
    print(f"{'n=' + str(_N) + '  warmup=' + str(_WARMUP):^88}\n")

    # ── 1. Write-only, no contention ──────────────────────────────────────────
    ws = []
    for nr in [1, 4, 16]:
        ws.append(bench_write(nr, 1))   # timestamp on
        ws.append(bench_write(nr, 0))   # timestamp off
    _section("Write only (no readers)", ws)

    # ── 2. Sequential write then read (no contention) ─────────────────────────
    wr_pairs: list[OpStats] = []
    for nr in [1, 4, 16]:
        w, r = bench_read(nr, 1)
        wr_pairs += [w, r]
    _section("Write + read, sequential (ts=on)", wr_pairs)

    wr_pairs2: list[OpStats] = []
    for nr in [1, 4, 16]:
        w, r = bench_read(nr, 0)
        wr_pairs2 += [w, r]
    _section("Write + read, sequential (ts=off)", wr_pairs2)

    # ── 3. Concurrent contention ──────────────────────────────────────────────
    cont: list[OpStats] = []
    for nr in [1, 4]:
        w, r = bench_contention(nr, 1, duration_s=2.0)
        cont += [w, r]
    _section("Concurrent write + read (2s, ts=on)", cont)

    # ── build HTML report ─────────────────────────────────────────────────────
    _build_report()


# ═══════════════════════════════════════════════════════════════════════════════
# HTML report
# ═══════════════════════════════════════════════════════════════════════════════

def _build_report() -> None:
    import json

    out_dir = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(out_dir, "ops_report.html")

    # Flatten all stats into a JSON-serialisable list
    rows = []
    for section_title, stats in ALL_RESULTS:
        for st in stats:
            rows.append({
                "section": section_title,
                "label":   st.label,
                "mean":    round(st.mean, 1),
                "p50":     round(st.p50, 1),
                "p99":     round(st.p99, 1),
                "p999":    round(st.p999, 1),
                "n":       len(st.samples),
            })

    rows_json = json.dumps(rows)

    # Build per-section chart data for write-only section (ts on vs off)
    def write_chart() -> str:
        """Bar chart: write p50 for ts=on vs ts=off across robot counts."""
        robot_counts = [1, 4, 16]
        on_p50  = []
        off_p50 = []
        for nr in robot_counts:
            for r in rows:
                if r["section"] == "Write only (no readers)":
                    if f"n_robots={nr}  ts=on" in r["label"]:
                        on_p50.append(r["p50"])
                    if f"n_robots={nr}  ts=off" in r["label"]:
                        off_p50.append(r["p50"])
        return json.dumps({
            "labels": ["1 robot", "4 robots", "16 robots"],
            "datasets": [
                {"label": "write (ts=on)",  "data": on_p50,
                 "backgroundColor": "#4a8fe8b0", "borderColor": "#4a8fe8", "borderWidth": 1.5, "borderRadius": 3},
                {"label": "write (ts=off)", "data": off_p50,
                 "backgroundColor": "#2fc87fb0", "borderColor": "#2fc87f", "borderWidth": 1.5, "borderRadius": 3},
            ]
        })

    def read_chart(ts_label: str, section: str) -> str:
        robot_counts = [1, 4, 16]
        w_p50 = []
        r_p50 = []
        for nr in robot_counts:
            for r in rows:
                if r["section"] == section:
                    if f"write  n_robots={nr}" in r["label"]:
                        w_p50.append(r["p50"])
                    if f"read   n_robots={nr}" in r["label"]:
                        r_p50.append(r["p50"])
        return json.dumps({
            "labels": ["1 robot", "4 robots", "16 robots"],
            "datasets": [
                {"label": "write p50", "data": w_p50,
                 "backgroundColor": "#4a8fe8b0", "borderColor": "#4a8fe8", "borderWidth": 1.5, "borderRadius": 3},
                {"label": "read p50",  "data": r_p50,
                 "backgroundColor": "#e09b30b0", "borderColor": "#e09b30", "borderWidth": 1.5, "borderRadius": 3},
            ]
        })

    def contention_chart() -> str:
        robot_counts = [1, 4]
        w_p50 = []
        r_p50 = []
        section = "Concurrent write + read (2s, ts=on)"
        for nr in robot_counts:
            for r in rows:
                if r["section"] == section:
                    if f"write  contention  n_robots={nr}" in r["label"]:
                        w_p50.append(r["p50"])
                    if f"read   contention  n_robots={nr}" in r["label"]:
                        r_p50.append(r["p50"])
        return json.dumps({
            "labels": ["1 robot", "4 robots"],
            "datasets": [
                {"label": "write p50 (contention)", "data": w_p50,
                 "backgroundColor": "#4a8fe8b0", "borderColor": "#4a8fe8", "borderWidth": 1.5, "borderRadius": 3},
                {"label": "read p50  (contention)", "data": r_p50,
                 "backgroundColor": "#d96050b0", "borderColor": "#d96050", "borderWidth": 1.5, "borderRadius": 3},
            ]
        })

    cw  = write_chart()
    cr1 = read_chart("ts=on",  "Write + read, sequential (ts=on)")
    cr2 = read_chart("ts=off", "Write + read, sequential (ts=off)")
    cc  = contention_chart()

    # Build table rows HTML
    def tbl_section(section: str) -> str:
        section_rows = [r for r in rows if r["section"] == section]
        if not section_rows:
            return ""
        min_p50 = min(r["p50"] for r in section_rows)
        html = f'<tr><td colspan="6" class="section-hdr">{section}</td></tr>\n'
        for r in section_rows:
            best = ' class="best"' if r["p50"] == min_p50 else ""
            html += (
                f'<tr><td class="lbl">{r["label"]}</td>'
                f'<td{best}>{r["mean"]:.1f}</td>'
                f'<td{best}>{r["p50"]:.1f}</td>'
                f'<td>{r["p99"]:.1f}</td>'
                f'<td>{r["p999"]:.1f}</td>'
                f'<td class="dim">{r["n"]}</td></tr>\n'
            )
        return html

    table_html = "".join(tbl_section(s) for s, _ in ALL_RESULTS)

    html = f"""<title>shmbridge Op Benchmark</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&family=JetBrains+Mono:wght@400;600&display=swap">
<script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.min.js"></script>

<style>
:root {{
  --bg:#f4f5f8; --surface:#fff; --surface2:#f0f1f6;
  --border:#e2e5ee; --text:#1a1e2c; --muted:#6b7280;
  --accent:#2b6fd4; --green:#2fc87f; --amber:#e09b30; --red:#d96050;
  --mono:'JetBrains Mono','Fira Code',monospace;
  --sans:'Inter',system-ui,sans-serif;
  --r:8px;
}}
@media (prefers-color-scheme:dark) {{
  :root:not([data-theme="light"]) {{
    --bg:#0f1117; --surface:#181c27; --surface2:#1f2438;
    --border:#2a3048; --text:#e4e8f4; --muted:#8892ab; --accent:#5aaaf8;
  }}
}}
:root[data-theme="dark"] {{
  --bg:#0f1117; --surface:#181c27; --surface2:#1f2438;
  --border:#2a3048; --text:#e4e8f4; --muted:#8892ab; --accent:#5aaaf8;
}}
*{{box-sizing:border-box;margin:0;padding:0}}
body{{font-family:var(--sans);background:var(--bg);color:var(--text);
     font-size:13.5px;line-height:1.55;
     padding:2rem max(1.5rem,calc(50vw - 44rem))}}
.eyebrow{{font-family:var(--mono);font-size:.68rem;font-weight:600;
          letter-spacing:.1em;text-transform:uppercase;color:var(--muted);
          margin-bottom:.35rem}}
h1{{font-size:1.5rem;font-weight:700;margin-bottom:.3rem;text-wrap:balance}}
.meta{{font-family:var(--mono);font-size:.78rem;color:var(--muted);margin-bottom:1.75rem}}
.stat-row{{display:flex;gap:1.5rem;flex-wrap:wrap;margin-bottom:2rem}}
.stat{{background:var(--surface);border:1px solid var(--border);
       border-radius:var(--r);padding:.9rem 1.1rem;min-width:9rem}}
.stat-val{{font-family:var(--mono);font-size:1.3rem;font-weight:600;
           line-height:1;color:var(--accent)}}
.stat-key{{font-size:.72rem;color:var(--muted);margin-top:.2rem}}
.chart-row{{display:grid;grid-template-columns:1fr 1fr;gap:1rem;margin-bottom:2rem}}
.card{{background:var(--surface);border:1px solid var(--border);
       border-radius:var(--r);padding:1rem 1.1rem}}
.card-title{{font-size:.7rem;font-weight:600;text-transform:uppercase;
             letter-spacing:.07em;color:var(--muted);margin-bottom:.75rem}}
canvas{{width:100%!important}}
h2{{font-size:.88rem;font-weight:700;text-transform:uppercase;
    letter-spacing:.06em;color:var(--muted);margin:1.75rem 0 .85rem}}
.tbl-wrap{{overflow-x:auto;border-radius:var(--r);margin-bottom:1.5rem}}
table{{width:100%;border-collapse:collapse;background:var(--surface);
       border:1px solid var(--border);font-size:.78rem;
       font-variant-numeric:tabular-nums}}
th{{background:var(--surface2);border-bottom:1px solid var(--border);
    padding:.5rem .8rem;text-align:left;font-family:var(--mono);
    font-size:.67rem;font-weight:600;color:var(--muted);white-space:nowrap}}
td{{padding:.45rem .8rem;border-bottom:1px solid var(--border);
    font-family:var(--mono);white-space:nowrap}}
tr:last-child td{{border-bottom:none}}
.section-hdr{{background:var(--surface2);font-family:var(--sans);
              font-size:.72rem;font-weight:600;color:var(--muted);
              text-transform:uppercase;letter-spacing:.06em;
              padding:.4rem .8rem!important;border-top:1px solid var(--border)}}
.lbl{{font-family:var(--sans)!important;color:var(--muted);font-size:.75rem!important}}
.best{{color:var(--green);font-weight:600}}
.dim{{color:var(--muted)}}
.insight{{background:var(--surface);border:1px solid var(--border);
          border-left:4px solid var(--accent);border-radius:var(--r);
          padding:1rem 1.25rem;font-size:.82rem;line-height:1.7;
          color:var(--muted);margin-top:1.5rem}}
.insight strong{{color:var(--text)}}
code{{font-family:var(--mono);font-size:.78rem;
      background:var(--surface2);padding:.1rem .3rem;border-radius:3px}}
</style>

<div class="eyebrow">shmbridge · seqlock micro-benchmark · n={_N} samples</div>
<h1>Write &amp; Read Operation Cost</h1>
<p class="meta">Single-process baseline · Sequential read-after-write · Concurrent contention · all times in nanoseconds</p>

<div id="stat-row" class="stat-row"></div>

<h2>Write cost — timestamp on vs off</h2>
<div class="chart-row">
  <div class="card">
    <div class="card-title">write_state() p50 (ns) — no readers</div>
    <canvas id="cw"></canvas>
  </div>
  <div class="card">
    <div class="card-title">write &amp; read p50 (ns) — sequential, ts=on</div>
    <canvas id="cr1"></canvas>
  </div>
</div>

<div class="chart-row">
  <div class="card">
    <div class="card-title">write &amp; read p50 (ns) — sequential, ts=off</div>
    <canvas id="cr2"></canvas>
  </div>
  <div class="card">
    <div class="card-title">write &amp; read p50 (ns) — concurrent contention</div>
    <canvas id="cc"></canvas>
  </div>
</div>

<h2>Full results (all times in ns)</h2>
<div class="tbl-wrap">
<table>
  <thead>
    <tr><th>Label</th><th>mean</th><th>p50</th><th>p99</th><th>p999</th><th>n</th></tr>
  </thead>
  <tbody>{table_html}</tbody>
</table>
</div>

<div class="insight">
  <strong>What to look for.</strong>
  The write p50 with <code>ts=on</code> vs <code>ts=off</code> shows the cost of
  <code>CLOCK_MONOTONIC_RAW</code> inside the seqlock window.
  A single <code>clock_gettime</code> call typically costs 20-60 ns on Linux (VDSO path);
  if the delta between ts=on and ts=off exceeds ~80 ns the kernel is falling back to a
  real syscall.
  <br><br>
  <strong>Under contention</strong> the seqlock reader spins when it catches an odd seq
  and retries.  High p99 on the read side indicates frequent torn reads, expected when
  the writer rate is much faster than one write per read.
  <br><br>
  <strong>Robot count</strong> does not affect write cost for a single robot index — each
  slot is independent.  Higher counts increase the memory footprint but not per-slot
  seqlock overhead.
</div>

<script>
const ROWS = {rows_json};
const OPTS = () => ({{
  responsive:true, animation:false,
  plugins:{{ legend:{{ position:'bottom', labels:{{ boxWidth:10,font:{{size:11}} }} }} }},
  scales:{{
    x:{{ grid:{{display:false}}, ticks:{{font:{{size:11}}}} }},
    y:{{ beginAtZero:true,
         ticks:{{font:{{family:"'JetBrains Mono',monospace",size:10}}}},
         title:{{display:true,text:'ns',font:{{size:10}}}} }}
  }}
}});

new Chart(document.getElementById('cw'),  {{type:'bar', data:{cw},  options:OPTS()}});
new Chart(document.getElementById('cr1'), {{type:'bar', data:{cr1}, options:OPTS()}});
new Chart(document.getElementById('cr2'), {{type:'bar', data:{cr2}, options:OPTS()}});
new Chart(document.getElementById('cc'),  {{type:'bar', data:{cc},  options:OPTS()}});

// Key stat tiles
const sr = document.getElementById('stat-row');
const tiles = [
  {{ label:'write p50 (ts=on, 1 robot)',  key: r => r.section==='Write only (no readers)' && r.label.includes('n_robots=1  ts=on'),  field:'p50', unit:'ns', color:'var(--accent)' }},
  {{ label:'write p50 (ts=off, 1 robot)', key: r => r.section==='Write only (no readers)' && r.label.includes('n_robots=1  ts=off'), field:'p50', unit:'ns', color:'var(--green)' }},
  {{ label:'read p50  (ts=on, 1 robot)',  key: r => r.section.includes('ts=on)') && r.label.includes('read') && r.label.includes('n_robots=1'), field:'p50', unit:'ns', color:'var(--amber)' }},
  {{ label:'read p50  (contention)',       key: r => r.section.includes('Concurrent') && r.label.includes('read') && r.label.includes('n_robots=1'), field:'p50', unit:'ns', color:'var(--red)' }},
];
tiles.forEach(t => {{
  const row = ROWS.find(t.key);
  if (!row) return;
  const div = document.createElement('div');
  div.className = 'stat';
  div.innerHTML = `<div class="stat-val" style="color:${{t.color}}">${{row[t.field].toFixed(0)}} ${{t.unit}}</div><div class="stat-key">${{t.label}}</div>`;
  sr.appendChild(div);
}});
</script>
"""

    with open(out, "w") as f:
        f.write(html)
    print(f"\nReport written to: {out}")


if __name__ == "__main__":
    main()
