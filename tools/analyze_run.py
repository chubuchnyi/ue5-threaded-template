#!/usr/bin/env python3
"""Analyze a controller_sim CSV: send/arrival interval jitter and the
phys-tick -> controller-receipt latency estimate.

Usage:
    python tools/analyze_run.py run.csv [--out docs/plots]

The controller writes one row per 2 kHz control tick with columns:
    tick_ns,sim_input_ns,seq,t_tx_ns,state,limiter,fill,in0..in5,out0..out5

We reduce to one record per received setpoint (first tick a seq appears) and
derive:
  * send interval   = Δ t_tx_ns   between consecutive seqs  (UE sender cadence)
  * arrival interval= Δ tick_ns   between consecutive seqs  (controller arrival)
  * transport jitter= (tick_ns - t_tx_ns) - min(...)        (loopback hop, skew
    removed; both clocks are QPC on the same host so the offset is constant)

Latency note: t_tx_ns (UE clock) and tick_ns (controller clock) share the QPC
rate but not the epoch, so only the *jitter* of the hop is recoverable here.
The absolute phys-tick -> worker-send leg is measured inside UE (motion.Stats
PipeLatencyUs); end-to-end latency estimate = that + this hop jitter floor.
"""
import argparse
import csv
import os
import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAVE_PLT = True
except Exception:
    HAVE_PLT = False


def load(path):
    seq_first = {}  # seq -> (tick_ns, t_tx_ns), first occurrence wins
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                seq = int(row["seq"])
                tick = int(row["tick_ns"])
                ttx = int(row["t_tx_ns"])
            except (KeyError, ValueError):
                continue
            # Skip pre-stream rows the controller writes before it has received
            # any setpoint (state INIT: seq/t_tx are still 0).
            if ttx <= 0 or seq <= 0:
                continue
            if seq not in seq_first:
                seq_first[seq] = (tick, ttx)
    seqs = np.array(sorted(seq_first))
    tick = np.array([seq_first[s][0] for s in seqs], dtype=np.int64)
    ttx = np.array([seq_first[s][1] for s in seqs], dtype=np.int64)
    return seqs, tick, ttx


def stats_us(x_ns):
    us = x_ns / 1000.0
    return dict(p50=np.percentile(us, 50), p99=np.percentile(us, 99),
                mx=us.max(), mn=us.min(), mean=us.mean(), n=len(us))


def hist_png(data_us, title, xlabel, path, clip=None):
    if not HAVE_PLT:
        return
    d = np.asarray(data_us)
    if clip is not None:
        d = d[d <= clip]
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.hist(d, bins=80, color="#3b82f6", edgecolor="none")
    for p, c in ((50, "#22c55e"), (99, "#f59e0b")):
        v = np.percentile(np.asarray(data_us), p)
        ax.axvline(v, color=c, lw=1.5, label=f"p{p} = {v:.0f} us")
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("count")
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--out", default="docs/plots")
    args = ap.parse_args()

    seqs, tick, ttx = load(args.csv)
    if len(seqs) < 3:
        print("not enough data")
        return

    # Gaps here are frames whose seq was coalesced into one 2 kHz control-tick
    # row (only the later seq is logged) — NOT network loss. controller_sim's
    # own per-frame drain loop is the authority on real loss.
    gaps = int((seqs[-1] - seqs[0] + 1) - len(seqs))
    send_iv = np.diff(ttx)
    arr_iv = np.diff(tick)
    delay = tick - ttx
    transport = delay - delay.min()  # skew-removed hop jitter

    s_send = stats_us(send_iv)
    s_arr = stats_us(arr_iv)
    s_tr = stats_us(transport)

    print(f"frames={len(seqs)}  seq_span={seqs[-1]-seqs[0]+1}  csv_coalesced_gaps={gaps} "
          f"(logging granularity, not loss)")
    print(f"send interval  us: p50={s_send['p50']:.0f} p99={s_send['p99']:.0f} "
          f"max={s_send['mx']:.0f} mean={s_send['mean']:.1f}")
    print(f"arrival interval us: p50={s_arr['p50']:.0f} p99={s_arr['p99']:.0f} "
          f"max={s_arr['mx']:.0f} mean={s_arr['mean']:.1f}")
    print(f"transport jitter us: p50={s_tr['p50']:.0f} p99={s_tr['p99']:.0f} "
          f"max={s_tr['mx']:.0f} (skew-removed hop floor)")

    if HAVE_PLT:
        os.makedirs(args.out, exist_ok=True)
        hist_png(send_iv / 1000.0, "Worker send interval", "interval (us)",
                 os.path.join(args.out, "send_interval.png"), clip=4000)
        hist_png(arr_iv / 1000.0, "Controller arrival interval", "interval (us)",
                 os.path.join(args.out, "arrival_interval.png"), clip=4000)
        hist_png(transport / 1000.0, "Transport jitter (skew-removed)", "delay (us)",
                 os.path.join(args.out, "transport_jitter.png"), clip=3000)
        print(f"plots -> {args.out}/{{send_interval,arrival_interval,transport_jitter}}.png")
    else:
        print("(matplotlib unavailable; numbers only)")


if __name__ == "__main__":
    main()
