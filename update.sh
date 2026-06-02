#!/bin/bash

OUT_DIR="../../../Desktop/kida"

#to binalry
# --include-data-files bundles libtact.so into the onefile binary at tact/bin/libtact.so,
# which is where tact/_clib.py's first lookup ('<package_dir>/bin/libtact.so') resolves to
# at runtime (nuitka rewrites __file__ to the extracted location).
uv run python -m nuitka --onefile --output-filename=kida   --output-dir="$OUT_DIR" --follow-imports --remove-output --include-data-files=../tact/bin/libtact.so=tact/bin/libtact.so kida.run
#uv run python -m nuitka --onefile --output-filename=single --output-dir="$OUT_DIR" --follow-imports --remove-output --include-data-files=../tact/bin/libtact.so=tact/bin/libtact.so single.run
#uv run python -m nuitka --onefile --output-filename=logger --output-dir="$OUT_DIR" --follow-imports --remove-output ../dev/vive/logger
#uv run python -m nuitka --onefile --output-filename=player --output-dir="$OUT_DIR" --follow-imports --remove-output ../dev/vive/player.py

cp ./usrsample.py "$OUT_DIR"
cp -rL yml "$OUT_DIR"
#cp -rL eio "$OUT_DIR"
#cp ../dev/rs2/msender "$OUT_DIR"
#cp ../dev/rs2/mreceiver "$OUT_DIR"
#cp ../dev/vive/logger "$OUT_DIR"
#cp ../dev/vive/player.py "$OUT_DIR"
#cp ../dev/vive/vmaster "$OUT_DIR"
#cp ../dev/vive/steamvr.sh "$OUT_DIR"
#cp -r ../dev/manus/calib "$OUT_DIR"
#cp ../dev/can/up "$OUT_DIR"
#cp ../dev/can/down "$OUT_DIR"
