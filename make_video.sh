#!/bin/bash
# Converts StarStaX_EW8A0483-EW8A0530_gap_filling_*.jpg from Downloads into a video at 24fps.

DIR="$HOME/Downloads"
OUT="$HOME/Downloads/StarStaX_output.mp4"

if ! command -v ffmpeg &>/dev/null; then
  echo "ffmpeg not found. Install it with: brew install ffmpeg"
  exit 1
fi

ffmpeg -framerate 24 \
  -start_number 1 \
  -i "$DIR/StarStaX_EW8A0483-EW8A0530_gap_filling_%08d.jpg" \
  -vframes 47 \
  -c:v libx264 \
  -pix_fmt yuv420p \
  -crf 18 \
  "$OUT"

echo "Done: $OUT"
