#!/usr/bin/env python3
"""
Protocol Simulation Runner
--------------------------
Asks which protocol to simulate (I2C / CAN / UART), compiles and runs
the Verilog testbench, then saves the following into results/<PROTOCOL>/:
  - sim_output.txt   : full console log from vvp
  - <name>.vcd       : waveform dump
  - waveform.png     : signal plot generated from the VCD
"""

import os
import sys
import shutil
import subprocess
import textwrap
from pathlib import Path
from datetime import datetime

# ─── optional deps ────────────────────────────────────────────────────────────
try:
    import matplotlib
    matplotlib.use("Agg")           # headless / no display needed
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    print("[WARN] matplotlib not found – graphs will be skipped.")
    print("       Install with: pip install matplotlib\n")

# ─── project root ─────────────────────────────────────────────────────────────
ROOT = Path(__file__).parent.resolve()
RESULTS_ROOT = ROOT / "results"

# ─── protocol definitions ─────────────────────────────────────────────────────
PROTOCOLS = {
    "i2c": {
        "display": "I2C",
        "sources": ["i2c_decoder_tb.v", "i2c_decoder.v"],
        "top":     "i2c_decoder_tb",
        "vcd":     "i2c_decoder_tb.vcd",
        "timeout": 30,      # seconds
        "can_sim": False,
    },
    "uart": {
        "display": "UART",
        "sources": ["tb_uart_logic_analyzer.v", "uart_logic_analyzer.v"],
        "top":     "tb_logic_analyzer",
        "vcd":     "uart_tb.vcd",
        "timeout": 20,
        "can_sim": False,
    },
    "can": {
        "display": "CAN",
        "sources": ["tb_can_top.v", "can_top.v", "can_level_packet.v", "can_level_bit.v"],
        "top":     "tb_can_top",
        "vcd":     "dump.vcd",           # tb uses $dumpvars → dump.vcd by default
        "timeout": 15,                   # no $finish in CAN tb, cut at 15 s
        "can_sim": True,
    },
}

# ─── helpers ──────────────────────────────────────────────────────────────────

def banner(msg: str) -> None:
    width = 60
    print("\n" + "═" * width)
    print(f"  {msg}")
    print("═" * width)


def run(cmd: list, cwd: Path, timeout: int) -> tuple[int, str]:
    """Run *cmd* in *cwd*, return (returncode, combined_output)."""
    try:
        proc = subprocess.run(
            cmd,
            cwd=str(cwd),
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        output = proc.stdout + proc.stderr
        return proc.returncode, output
    except subprocess.TimeoutExpired as exc:
        partial = (exc.stdout or "") + (exc.stderr or "")
        return 0, f"[INFO] Simulation stopped after {timeout}s timeout\n" + (partial or "")


# ─── VCD parser ───────────────────────────────────────────────────────────────

def parse_vcd(vcd_path: Path) -> dict:
    """
    Minimal VCD parser. Returns:
      {signal_name: [(time_int, value_str), ...]}
    Handles only scalar ($var wire/reg 1 <id> <name> $end) signals.
    """
    signals: dict[str, list] = {}   # id → [(time, val)]
    id_to_name: dict[str, str] = {}
    current_time = 0

    with open(vcd_path, "r", errors="replace") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line:
                continue

            # variable declarations
            if line.startswith("$var"):
                parts = line.split()
                if len(parts) >= 5:
                    bit_width = parts[2]
                    var_id    = parts[3]
                    var_name  = parts[4]
                    if bit_width == "1":          # scalars only
                        id_to_name[var_id] = var_name
                        signals[var_id]     = []
                continue

            # timestamp
            if line.startswith("#"):
                try:
                    current_time = int(line[1:])
                except ValueError:
                    pass
                continue

            # value changes  b<val> <id>  or  <0/1/x/z><id>
            if line.startswith("b"):
                parts = line.split()
                if len(parts) == 2:
                    val, var_id = parts[0][1:], parts[1]
                    if var_id in signals:
                        signals[var_id].append((current_time, val))
            elif len(line) >= 2 and line[0] in "01xzXZ":
                val    = line[0]
                var_id = line[1:]
                if var_id in signals:
                    signals[var_id].append((current_time, val))

    # remap ids → human names
    named: dict[str, list] = {}
    for var_id, traces in signals.items():
        name = id_to_name.get(var_id, var_id)
        named[name] = traces
    return named


# ─── grapher ──────────────────────────────────────────────────────────────────

def plot_vcd(vcd_path: Path, out_png: Path, protocol: str, max_signals: int = 18) -> None:
    if not HAS_MPL:
        return

    print("  Parsing VCD …", end=" ", flush=True)
    try:
        data = parse_vcd(vcd_path)
    except Exception as exc:
        print(f"FAILED ({exc})")
        return
    print(f"OK  ({len(data)} signals found)")

    if not data:
        print("  [WARN] No scalar signals found in VCD – skipping graph.")
        return

    # pick signals: prefer interesting names (clk, rst, addr, data …)
    priority_kw = ["clk", "rst", "scl", "sda", "can", "tx", "rx",
                   "addr", "data", "valid", "ack", "error", "busy",
                   "trigger", "probe", "start", "state"]

    def priority(name: str) -> int:
        nl = name.lower()
        for i, kw in enumerate(priority_kw):
            if kw in nl:
                return i
        return len(priority_kw)

    chosen = sorted(data.keys(), key=priority)[:max_signals]

    # find global time range
    all_times = [t for sig in chosen for (t, _) in data[sig]]
    if not all_times:
        print("  [WARN] No time-stamped data – skipping graph.")
        return
    t_min, t_max = min(all_times), max(all_times)
    if t_max == t_min:
        t_max = t_min + 1

    n = len(chosen)
    fig_h = max(4, n * 0.55 + 2)
    fig, ax = plt.subplots(figsize=(16, fig_h))
    ax.set_facecolor("#0d1117")
    fig.patch.set_facecolor("#0d1117")

    colours = matplotlib.colormaps.get_cmap("tab20").resampled(n)

    for idx, sig_name in enumerate(reversed(chosen)):
        trace = data[sig_name]
        offset = idx * 1.4          # vertical spacing
        colour = colours(idx / max(n - 1, 1))

        if not trace:
            continue

        # build step-plot arrays
        times = [t_min] + [t for (t, _) in trace] + [t_max]
        vals  = []
        prev  = "0"
        for (_, v) in trace:
            vals.append(prev)
            prev = v
        vals = [vals[0]] + vals + [prev]

        def to_num(v: str) -> float:
            return 1.0 if v in ("1",) else 0.0

        ys = [to_num(v) + offset for v in vals]
        ax.step(times, ys, where="post", color=colour, linewidth=1.4)
        # fill under the high portions
        ax.fill_between(times, offset, ys, step="post",
                        color=colour, alpha=0.18)
        ax.text(t_min - (t_max - t_min) * 0.01, offset + 0.5,
                sig_name, ha="right", va="center",
                fontsize=7.5, color=colour, fontfamily="monospace")

    ax.set_xlim(t_min, t_max)
    ax.set_ylim(-0.5, n * 1.4 + 0.5)
    ax.set_xlabel("Time (simulation ticks)", color="#c9d1d9", fontsize=9)
    ax.set_title(f"{protocol} – Waveform  [{datetime.now().strftime('%Y-%m-%d %H:%M')}]",
                 color="#c9d1d9", fontsize=11, pad=12)
    ax.tick_params(colors="#c9d1d9", labelsize=8)
    ax.yaxis.set_visible(False)
    for spine in ax.spines.values():
        spine.set_edgecolor("#30363d")

    plt.tight_layout()
    fig.savefig(str(out_png), dpi=150, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"  Graph saved → {out_png.name}")


# ─── main ─────────────────────────────────────────────────────────────────────

def choose_protocol() -> dict:
    banner("Protocol Decoder Simulation Runner")
    print("Available protocols:")
    keys = list(PROTOCOLS.keys())
    for i, k in enumerate(keys, 1):
        print(f"  [{i}] {PROTOCOLS[k]['display']}")
    print()

    while True:
        raw = input("Enter protocol name or number (i2c / can / uart): ").strip().lower()
        if raw in PROTOCOLS:
            return PROTOCOLS[raw]
        if raw.isdigit() and 1 <= int(raw) <= len(keys):
            return PROTOCOLS[keys[int(raw) - 1]]
        print(f"  ✗ '{raw}' not recognised. Try again.")


def main() -> None:
    proto = choose_protocol()
    display  = proto["display"]
    sources  = proto["sources"]
    vcd_name = proto["vcd"]
    timeout  = proto["timeout"]

    banner(f"Running {display} Simulation")

    # ── output directory ──────────────────────────────────────────────────────
    out_dir = RESULTS_ROOT / display
    out_dir.mkdir(parents=True, exist_ok=True)

    sim_dir = out_dir / "sim"
    sim_dir.mkdir(exist_ok=True)

    # ── step 1 : compile ──────────────────────────────────────────────────────
    out_bin = ROOT / f"_sim_{display}.out"
    src_paths = [str(ROOT / s) for s in sources]
    compile_cmd = ["iverilog", "-g2005-sv", "-o", str(out_bin)] + src_paths

    print(f"\n[1/3] Compiling {display} …")
    print("  " + " ".join(compile_cmd))
    rc, compile_log = run(compile_cmd, ROOT, timeout=60)

    if rc != 0:
        print(f"  ✗ Compilation FAILED (exit {rc})\n{compile_log}")
        (out_dir / "compile_error.txt").write_text(compile_log)
        sys.exit(1)
    print(f"  ✓ Compilation OK")

    # ── step 2 : simulate ─────────────────────────────────────────────────────
    print(f"\n[2/3] Simulating {display}  (timeout {timeout}s) …")
    vvp_cmd = ["vvp", str(out_bin)]
    rc_sim, sim_log = run(vvp_cmd, ROOT, timeout=timeout)

    # save sim log
    log_path = sim_dir / "sim_output.txt"
    log_path.write_text(sim_log)
    print(f"  Console log → {log_path.relative_to(ROOT)}")
    # print first 40 lines to terminal
    lines = sim_log.splitlines()
    for ln in lines[:40]:
        print("  │ " + ln)
    if len(lines) > 40:
        print(f"  │ … ({len(lines) - 40} more lines in sim_output.txt)")

    # move VCD file into sim/ subfolder
    vcd_src = ROOT / vcd_name
    vcd_dst = sim_dir / vcd_name
    if vcd_src.exists():
        shutil.move(str(vcd_src), str(vcd_dst))
        print(f"  VCD file   → {vcd_dst.relative_to(ROOT)}")
    else:
        vcd_dst = None
        print(f"  [WARN] VCD file '{vcd_name}' not found – graph will be skipped.")

    # clean up binary
    out_bin.unlink(missing_ok=True)

    # ── step 3 : graph ────────────────────────────────────────────────────────
    print(f"\n[3/3] Generating waveform graph …")
    if vcd_dst and vcd_dst.exists() and HAS_MPL:
        graph_path = out_dir / "waveform.png"
        plot_vcd(vcd_dst, graph_path, display)
    elif not HAS_MPL:
        print("  Skipped (matplotlib not installed).")
    else:
        print("  Skipped (no VCD available).")

    # ── summary ───────────────────────────────────────────────────────────────
    banner(f"{display} Results saved to  results/{display}/")
    print(f"  📁 results/{display}/")
    print(f"     ├─ sim/")
    print(f"     │   ├─ sim_output.txt")
    if vcd_dst:
        print(f"     │   └─ {vcd_name}")
    if HAS_MPL and vcd_dst:
        print(f"     └─ waveform.png")
    print()


if __name__ == "__main__":
    main()
