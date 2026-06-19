#!/bin/bash

OUT_DIR="../../Desktop/kida"

bash build.sh

rm -rf "$OUT_DIR"
mkdir -p "$OUT_DIR"

#to binalry
uv run python -m nuitka --onefile --output-filename=kida   --output-dir="$OUT_DIR" --follow-imports --remove-output --include-data-files=../tact/native/lib/libtact.so=tact/bin/libtact.so kida.run
uv run python -m nuitka --onefile --output-filename=single --output-dir="$OUT_DIR" --follow-imports --remove-output --include-data-files=../tact/native/lib/libtact.so=tact/bin/libtact.so single.run

cp _/uv.lock "$OUT_DIR"
cp _/pyproject.toml "$OUT_DIR"

cp ./README.md "$OUT_DIR"
cp ./usrsample.py "$OUT_DIR"

cp -rL yml "$OUT_DIR"
cp -rL eio "$OUT_DIR"

cp ../dev/rs2/msender "$OUT_DIR"
cp ../dev/rs2/mreceiver "$OUT_DIR"

cp ../dev/vive/steamvr.sh "$OUT_DIR"
cp -r ../dev/manus/calib "$OUT_DIR"
cp ../dev/vive/vmaster "$OUT_DIR"
cp ../dev/vive/logger "$OUT_DIR"
cp ../dev/vive/player "$OUT_DIR"

cp ../dev/can/up "$OUT_DIR"
cp ../dev/can/down "$OUT_DIR"

cp ../zmqmsg "$OUT_DIR"
