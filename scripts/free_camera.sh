#!/bin/bash
# Free the RealSense from unitree's onboard videohub.
#
# videohub_pc4 (launched + respawned by master_service) opens the RealSense
# /dev/video* node exclusively, so our transmitter gets EBUSY (errno 16,
# "Device or resource busy"). Killing it doesn't stick — master_service just
# respawns it. This replaces the videohub binary on disk with a no-op stub
# (sleep infinity): master_service's watchdog still sees a live process, but it
# no longer touches the camera. The original is backed up to <bin>.real.
#
#   RUN ON THE ROBOT HOST (not inside a container), as root:
#     sudo ./scripts/free_camera.sh
#   Restore the onboard camera later:
#     sudo ./scripts/free_camera.sh --restore
#
# Idempotent. If the rootfs restores the original on reboot, re-run after boot
# (or install the systemd unit printed by:  ./scripts/free_camera.sh --unit).

set -euo pipefail

MARKER="# kist-camera-stub"
PATTERN="${VIDEOHUB_PATTERN:-videohub}"   # override if the process name differs

if [ "$(id -u)" -ne 0 ]; then
    echo "must run as root (sudo)"; exit 1
fi

# ── locate the videohub binary (from a running PID, else a known stub) ────────
find_bins() {
    # unique resolved exe paths of matching processes
    for pid in $(pgrep -f "$PATTERN" || true); do
        readlink -f "/proc/$pid/exe" 2>/dev/null || true
    done | sort -u
}

# ── --restore: put the real binary back ──────────────────────────────────────
if [ "${1:-}" = "--restore" ]; then
    shopt -s nullglob
    restored=0
    for real in $(find / -maxdepth 6 -name '*.real' -path '*videohub*' 2>/dev/null); do
        bin="${real%.real}"
        echo "restoring $bin"
        mv -f "$real" "$bin"; chmod +x "$bin"; restored=1
    done
    [ "$restored" = 1 ] && { pkill -f "$PATTERN" || true; echo "done — onboard videohub restored"; } \
                        || echo "no <bin>.real backups found"
    exit 0
fi

# ── --unit: print a systemd unit for boot persistence ────────────────────────
if [ "${1:-}" = "--unit" ]; then
    self="$(readlink -f "$0")"
    cat <<EOF
# Install to make the stub survive reboots (if the rootfs restores the binary):
sudo tee /etc/systemd/system/kist-free-camera.service >/dev/null <<UNIT
[Unit]
Description=Free RealSense from unitree videohub
After=multi-user.target

[Service]
Type=oneshot
ExecStart=${self}
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
UNIT
sudo systemctl enable --now kist-free-camera.service
EOF
    exit 0
fi

# ── report current holders ───────────────────────────────────────────────────
echo "== holders of /dev/video* =="
fuser -v /dev/video* 2>&1 || true
echo

# ── stub every videohub binary found ─────────────────────────────────────────
mapfile -t BINS < <(find_bins)
if [ ${#BINS[@]} -eq 0 ]; then
    echo "no '$PATTERN' process found."
    if fuser /dev/video* >/dev/null 2>&1; then
        echo "but /dev/video* is still held — the holder isn't visible here."
        echo "are you inside a container? run this on the ROBOT HOST."
    else
        echo "/dev/video* is free — nothing to do."
    fi
    exit 0
fi

for bin in "${BINS[@]}"; do
    echo "== videohub binary: $bin =="
    if head -c 200 "$bin" 2>/dev/null | grep -q "$MARKER"; then
        echo "  already stubbed"
        continue
    fi
    # A running executable can't be overwritten (ETXTBSY) — rename/unlink it
    # first (both allowed while it runs; the live process keeps its inode), then
    # create a fresh stub file at the original path.
    if [ -e "${bin}.real" ]; then
        rm -f "$bin"                                   # backup already exists
    else
        mv "$bin" "${bin}.real"; echo "  backed up -> ${bin}.real"
    fi
    cat > "$bin" <<EOF
#!/bin/bash
$MARKER  original: ${bin}.real  (restore: free_camera.sh --restore)
exec sleep infinity
EOF
    chmod +x "$bin"
    echo "  replaced with no-op stub"
done

# ── kill running instances so master_service respawns the stub ───────────────
echo "== killing running videohub (master_service respawns the stub) =="
pkill -f "$PATTERN" || true
sleep 2

# ── verify ───────────────────────────────────────────────────────────────────
echo "== verify =="
if fuser /dev/video* >/dev/null 2>&1; then
    echo "  STILL HELD:"; fuser -v /dev/video* 2>&1 || true
    echo "  (a fresh videohub may have grabbed it before the stub took over — re-run once)"
    exit 1
fi
echo "  OK: /dev/video* is free — start the transmitter now."
