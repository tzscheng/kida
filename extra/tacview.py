#!/usr/bin/env -S uv run python
import argparse
import sys
import time

import numpy as np
import zmq

RESET = "\033[0m"
CLEAR = "\033[2J\033[H"
HIDE_CURSOR = "\033[?25l"
SHOW_CURSOR = "\033[?25h"

PALETTE = [
    (0, 0, 0),
    (80, 0, 0),
    (140, 0, 0),
    (200, 20, 0),
    (255, 70, 0),
    (255, 130, 0),
    (255, 190, 40),
    (255, 230, 120),
    (255, 255, 255),
]

DEFAULT_NAMES = [
    "index_fingertip_taxel",
    "hand.index_fingertip_taxel",
    "hand1.index_fingertip_taxel",
]


def bg(rgb):
    r, g, b = rgb
    return f"\033[48;2;{r};{g};{b}m"


def color_for(v, vmax):
    if vmax <= 0:
        t = 0.0
    else:
        t = max(0.0, min(1.0, float(v) / vmax))
    idx = int(round(t * (len(PALETTE) - 1)))
    return PALETTE[idx]


def parse_shape(s):
    try:
        rows_s, cols_s = s.lower().split("x", 1)
        rows, cols = int(rows_s), int(cols_s)
    except Exception:
        raise SystemExit("--shape must look like 4x4")
    if rows <= 0 or cols <= 0:
        raise SystemExit("--shape dimensions must be positive")
    return rows, cols


def grid_lines(name, values, rows, cols, vmax):
    # YAML sample order is z-major rows with x along columns. For the fingertip
    # viewer, show x-major rows instead: first row is the most distal taxels.
    a = values.reshape(rows, cols).T
    lines = [
        f"{name}",
        f"  rows: distal -> proximal   cols: +z -> -z",
        f"  sum={a.sum():8.3f}  max={a.max():8.3f}  nonzero={(a > 1e-6).sum():2d}",
    ]
    for row in a:
        blocks = []
        for v in row:
            blocks.append(bg(color_for(v, vmax)) + "  " + RESET)
        nums = " ".join(f"{v:7.2f}" for v in row)
        lines.append("  " + " ".join(blocks) + "   " + nums)
    return lines


def main():
    ap = argparse.ArgumentParser(description="Live terminal heatmap for KIDA DG5 fingertip tactile sensor.")
    ap.add_argument(
        "--names",
        nargs="+",
        default=DEFAULT_NAMES,
        help="tactile IPC names to subscribe; defaults cover direct, single.run, and kida.run left hand",
    )
    ap.add_argument("--shape", default="4x4", help="array shape, e.g. 4x4")
    ap.add_argument("--hz", type=float, default=20.0, help="terminal refresh rate")
    ap.add_argument("--max", dest="vmax", type=float, default=0.0,
                    help="fixed color max; 0 means auto-scale from current frame")
    ap.add_argument("--timeout", type=float, default=2.0, help="seconds before showing stale data")
    args = ap.parse_args()

    rows, cols = parse_shape(args.shape)
    ntaxel = rows * cols

    ctx = zmq.Context.instance()
    poller = zmq.Poller()
    socks = {}
    for name in args.names:
        s = ctx.socket(zmq.SUB)
        s.setsockopt(zmq.CONFLATE, 1)
        s.setsockopt(zmq.SUBSCRIBE, b"")
        s.connect(f"ipc:///dev/shm/{name}")
        poller.register(s, zmq.POLLIN)
        socks[s] = name

    data = {name: np.zeros(ntaxel, dtype=np.float32) for name in args.names}
    stamp = {name: 0.0 for name in args.names}
    period = 1.0 / args.hz if args.hz > 0 else 0.05
    next_draw = 0.0

    sys.stdout.write(HIDE_CURSOR + CLEAR)
    sys.stdout.flush()
    try:
        while True:
            events = dict(poller.poll(max(1, int(period * 1000))))
            now = time.time()
            for s in events:
                name = socks[s]
                buf = s.recv()
                arr = np.frombuffer(buf, dtype="<f4")
                if arr.size != ntaxel:
                    sys.stdout.write(CLEAR + f"{name}: bad payload: {arr.size} floats, expected {ntaxel}\n")
                    sys.stdout.flush()
                    continue
                data[name] = arr.copy()
                stamp[name] = now

            if now < next_draw:
                continue
            next_draw = now + period

            active = [name for name in args.names if now - stamp[name] <= args.timeout]
            shown = active if active else args.names
            frame_max = max(float(data[name].max()) for name in shown)
            vmax = args.vmax if args.vmax > 0 else max(frame_max, 1e-6)

            lines = [
                "KIDA DG5 index fingertip tactile viewer  (Ctrl-C to exit)",
                f"sockets: {', '.join(args.names)}",
                f"shape: {rows}x{cols}   color max: {vmax:.3f}   payload: float32 ({ntaxel},1)",
                "",
            ]
            for name in shown:
                if now - stamp[name] > args.timeout:
                    age = "never" if stamp[name] == 0.0 else f"{now - stamp[name]:.1f}s"
                    lines.append(f"{name}  waiting/stale: no frame for {age}")
                    lines.append("")
                else:
                    lines.extend(grid_lines(name, data[name], rows, cols, vmax))
                    lines.append("")
            sys.stdout.write(CLEAR + "\n".join(lines))
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write(SHOW_CURSOR + RESET + "\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
