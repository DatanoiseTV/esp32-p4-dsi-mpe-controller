#!/usr/bin/env bash
#
# Fetches a pixel-perfect framebuffer screenshot from a running device
# and writes docs/screenshot.png. The firmware exposes a tiny HTTP
# server (mpe-screenshot component) that serves the live front buffer
# as a BMP; this script downloads it and re-encodes to PNG.
#
# Usage:
#   docs/grab_screenshot.sh <device-ip-or-hostname>
#
# Example:
#   docs/grab_screenshot.sh 192.168.178.67
#
set -euo pipefail

if [[ $# -lt 1 ]]; then
    echo "usage: $0 <device-ip>" >&2
    exit 2
fi
HOST="$1"

HERE="$(cd "$(dirname "$0")" && pwd)"
BMP="$HERE/.screenshot.bmp"
PNG="$HERE/screenshot.png"

echo "fetching from http://$HOST/screenshot.bmp ..."
curl --fail --max-time 30 -o "$BMP" "http://$HOST/screenshot.bmp" \
     -w "  HTTP %{http_code}  %{size_download} bytes  %{time_total}s\n"

# PIL is the most portable BMP→PNG conversion path; falls back to
# ImageMagick if PIL isn't available.
if python3 -c "from PIL import Image" 2>/dev/null; then
    python3 - <<PY
from PIL import Image
img = Image.open("$BMP")
img.convert("RGB").save("$PNG", "PNG", optimize=True)
PY
elif command -v magick >/dev/null 2>&1; then
    magick "$BMP" "$PNG"
elif command -v convert >/dev/null 2>&1; then
    convert "$BMP" "$PNG"
else
    echo "no PIL or ImageMagick — leaving the raw BMP at $BMP" >&2
    exit 0
fi

rm -f "$BMP"
echo "wrote $PNG"
