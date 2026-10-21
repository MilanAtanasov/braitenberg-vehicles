#!/usr/bin/env bash
# Convert clips recorded with the page's "Record clip" button (.webm) to PowerPoint-friendly MP4 (H.264, 30 fps, no audio).
# The size is kept: 1920x1584 for "Stage and panel", 1920x1080 for "Stage only" (padded to even dimensions if needed).
# Usage: ./make-mp4.sh braitenberg-v3c-run4.webm [more files...]
set -euo pipefail
for f in "$@"; do
  ffmpeg -y -i "$f" -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2" -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p -r 30 -movflags +faststart -an "${f%.*}.mp4"
done
