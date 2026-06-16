#!/bin/bash

OUT_DIR="../../Desktop/kida"

rm -rf "$OUT_DIR"
mkdir -p "$OUT_DIR"

#to binalry
# --include-data-files bundles libtact.so into the onefile binary at tact/bin/libtact.so,
# which is where tact/_clib.py's first lookup ('<package_dir>/bin/libtact.so') resolves to
# at runtime (nuitka rewrites __file__ to the extracted location).
uv run python -m nuitka --onefile --output-filename=kida   --output-dir="$OUT_DIR" --follow-imports --remove-output --include-data-files=../tact/native/lib/libtact.so=tact/bin/libtact.so kida.run
uv run python -m nuitka --onefile --output-filename=single --output-dir="$OUT_DIR" --follow-imports --remove-output --include-data-files=../tact/native/lib/libtact.so=tact/bin/libtact.so single.run

#cp kida.run kida.py dg5.py "$OUT_DIR"
#cp tact/sim.py tact/rbd.py tact/control.py tact/__init__.py tact/_clib.py "$OUT_DIR"/tact
#cp -r tact/bin "$OUT_DIR"/tact

cp _/uv.lock "$OUT_DIR"
cp _/pyproject.toml "$OUT_DIR"

cp ./README.md "$OUT_DIR"
cp ./usrsample.py "$OUT_DIR"

cp -rL yml "$OUT_DIR"
cp -rL eio "$OUT_DIR"

cp ../dev/rs2/msender "$OUT_DIR"
cp ../dev/rs2/mreceiver "$OUT_DIR"

#cp ../dev/vive/vmaster "$OUT_DIR"
#cp ../dev/vive/logger "$OUT_DIR"
#cp ../dev/vive/player.py "$OUT_DIR"
#cp ../dev/vive/steamvr.sh "$OUT_DIR"

#cp -r ../dev/manus/calib "$OUT_DIR"

cp ../dev/can/up "$OUT_DIR"
cp ../dev/can/down "$OUT_DIR"

cp ../zmqmsg "$OUT_DIR"
