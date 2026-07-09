#!/usr/bin/env -S uv run python
import argparse
import os
import time
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib.pyplot as plt
import numpy as np
import yaml
import zmq
from matplotlib import colors

DEFAULT_NAMES = [
    "index_fingertip_taxel",
    "hand.index_fingertip_taxel",
    "hand1.index_fingertip_taxel",
]


def load_taxel_spec(yml_path, sensor_name):
    with Path(yml_path).open() as f:
        cfg = yaml.safe_load(f)

    specs = cfg.get("tactiles", [])
    spec = next((s for s in specs if s["name"] == sensor_name), None)
    if spec is None:
        raise SystemExit(f"sensor {sensor_name!r} not found in {yml_path}")

    bodies = {b["name"]: b for b in cfg["bodies"]}
    body = bodies[spec["body"]]
    shape = body["shapes"][0]
    if shape["type"] != "capsule":
        raise SystemExit(f"expected {spec['body']} shape to be capsule")

    samples = np.asarray(spec["samples"], dtype=float)
    radius, hh = map(float, shape["param"])
    center = np.asarray(shape.get("pos", [0.0, 0.0, 0.0]), dtype=float)
    return spec, samples, center, radius, hh


def connect(names):
    ctx = zmq.Context.instance()
    poller = zmq.Poller()
    socks = {}
    for name in names:
        s = ctx.socket(zmq.SUB)
        s.setsockopt(zmq.CONFLATE, 1)
        s.setsockopt(zmq.SUBSCRIBE, b"")
        s.connect(f"ipc:///dev/shm/{name}")
        poller.register(s, zmq.POLLIN)
        socks[s] = name
    return poller, socks


def capsule_wire(ax, center, radius, hh):
    cx = float(center[0])
    distal = cx - hh
    proximal = cx + hh
    theta = np.linspace(0.0, 2.0 * np.pi, 97)

    for x in np.linspace(distal, proximal, 4):
        ax.plot([x] * len(theta), radius * np.cos(theta), radius * np.sin(theta),
                color="0.75", linewidth=0.8)
    for a in [0.0, np.pi / 2.0, np.pi, 3.0 * np.pi / 2.0]:
        ax.plot([distal, proximal], [radius * np.cos(a)] * 2,
                [radius * np.sin(a)] * 2, color="0.75", linewidth=0.8)

    for side, center_x in [(-1.0, distal), (1.0, proximal)]:
        for phi in [np.pi / 6.0, np.pi / 3.0]:
            dx = side * radius * np.sin(phi)
            rr = radius * np.cos(phi)
            ax.plot([center_x + dx] * len(theta), rr * np.cos(theta),
                    rr * np.sin(theta), color="0.72", linewidth=0.8)
        for a in [0.0, np.pi / 2.0, np.pi, 3.0 * np.pi / 2.0]:
            phi = np.linspace(0.0, np.pi / 2.0, 40)
            ax.plot(center_x + side * radius * np.sin(phi),
                    radius * np.cos(phi) * np.cos(a),
                    radius * np.cos(phi) * np.sin(a),
                    color="0.72", linewidth=0.8)


def main():
    ap = argparse.ArgumentParser(description="Live matplotlib view for KIDA DG5F fingertip tactile sensor.")
    ap.add_argument("--names", nargs="+", default=DEFAULT_NAMES,
                    help="tactile IPC names to subscribe")
    ap.add_argument("--yml", default=str(Path(__file__).resolve().parent.parent / "yaml" / "dg5f-left.yaml"),
                    help="YAML file containing the unprefixed tactile geometry")
    ap.add_argument("--sensor", default="index_fingertip_taxel",
                    help="unprefixed tactile sensor name in --yml")
    ap.add_argument("--max", dest="vmax", type=float, default=0.0,
                    help="fixed color max; 0 means auto-scale")
    ap.add_argument("--hz", type=float, default=20.0, help="plot refresh rate")
    ap.add_argument("--timeout", type=float, default=2.0, help="seconds before showing stale title")
    args = ap.parse_args()

    spec, samples, center, radius, hh = load_taxel_spec(args.yml, args.sensor)
    pos = samples[:, 0:3]
    normal = samples[:, 3:6]
    ntaxel = pos.shape[0]
    if ntaxel != 16:
        raise SystemExit(f"expected 16 taxels, got {ntaxel}")

    poller, socks = connect(args.names)
    data = {name: np.zeros(ntaxel, dtype=np.float32) for name in args.names}
    stamp = {name: 0.0 for name in args.names}
    active_name = args.names[0]

    cmap = plt.get_cmap("inferno")
    norm = colors.Normalize(vmin=0.0, vmax=1.0)

    fig = plt.figure(figsize=(10, 5))
    ax = fig.add_subplot(121, projection="3d")
    ax2 = fig.add_subplot(122)
    fig.canvas.manager.set_window_title("KIDA DG5F tactile plot viewer")

    capsule_wire(ax, center, radius, hh)
    scat = ax.scatter(pos[:, 0], pos[:, 1], pos[:, 2], c=np.zeros(ntaxel),
                      cmap=cmap, norm=norm, s=75, edgecolor="black", linewidth=0.3)
    ax.quiver(pos[:, 0], pos[:, 1], pos[:, 2],
              normal[:, 0], normal[:, 1], normal[:, 2],
              length=0.004, color="navy", normalize=True)
    ax.set_title("index4 local frame")
    ax.set_xlabel("x: finger axis")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_box_aspect((4, 2, 2))
    ax.view_init(elev=22, azim=-55)

    # YAML sample order is z-major rows with x along columns. Viewer layout uses
    # rows as distal -> proximal, matching kida/tacview.py.
    layout = np.zeros((4, 4), dtype=float)
    img = ax2.imshow(layout, cmap=cmap, norm=norm, origin="upper", aspect="equal")
    ax2.set_title("viewer layout")
    ax2.set_xlabel("columns: +z -> -z")
    ax2.set_ylabel("rows: distal -> proximal")
    ax2.set_xticks(range(4))
    ax2.set_yticks(range(4))
    ax2.set_xticklabels(["+7.5", "+2.5", "-2.5", "-7.5"])
    ax2.set_yticklabels(["1 distal", "2", "3", "4 proximal"])
    for i in range(4):
        for j in range(4):
            ax2.text(j, i, "", ha="center", va="center", color="white", fontsize=9)
    texts = ax2.texts
    cb = fig.colorbar(img, ax=[ax, ax2], shrink=0.78, pad=0.04)
    cb.set_label("normal force")

    period = 1.0 / args.hz if args.hz > 0 else 0.05

    def update(_frame):
        nonlocal active_name
        events = dict(poller.poll(0))
        now = time.time()
        for s in events:
            name = socks[s]
            arr = np.frombuffer(s.recv(), dtype="<f4")
            if arr.size == ntaxel:
                data[name] = arr.copy()
                stamp[name] = now
                active_name = name

        value = data[active_name]
        frame_max = float(value.max())
        vmax = args.vmax if args.vmax > 0 else max(frame_max, 1e-6)
        norm.vmax = vmax

        scat.set_array(value)
        layout = value.reshape(4, 4).T
        img.set_data(layout)
        for text, v in zip(texts, layout.reshape(-1)):
            text.set_text(f"{v:.1f}")
            text.set_color("black" if v > 0.7 * vmax else "white")

        age = now - stamp[active_name] if stamp[active_name] else None
        state = "waiting" if age is None else ("stale" if age > args.timeout else "live")
        fig.suptitle(
            f"{active_name}  {state}  sum={value.sum():.3f}  max={frame_max:.3f}  color max={vmax:.3f}",
            fontsize=11,
        )
        return [scat, img, *texts]

    timer = fig.canvas.new_timer(interval=max(1, int(period * 1000)))
    timer.add_callback(lambda: (update(None), fig.canvas.draw_idle(), True)[-1])
    timer.start()
    update(None)
    plt.show()


if __name__ == "__main__":
    main()
