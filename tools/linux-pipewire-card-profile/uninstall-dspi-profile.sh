#!/usr/bin/env bash
set -euo pipefail

PROFILE_NAME="dspi-iec958.conf"
PATH_NAME="dspi-iec958-output.conf"
RULE_NAME="91-dspi-pipewire-alsa.rules"

NO_RESTART=0
DRY_RUN=0

usage() {
    cat <<EOF
Usage: $(basename "$0") [--no-restart] [--dry-run]

Removes the local DSPi ACP profile, mixer path, and udev rule.

Options:
  --no-restart   Do not restart user audio services after uninstall.
  --dry-run      Print what would be done without changing the system.
  -h, --help     Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-restart) NO_RESTART=1 ;;
        --dry-run) DRY_RUN=1 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

IN_TOOLBOX=0
[[ -f /run/.toolboxenv ]] && IN_TOOLBOX=1

need_cmd() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "missing required command: $1" >&2
        exit 1
    fi
}

need_host_cmd() {
    local check_cmd="$1"
    [[ "${IN_TOOLBOX}" -eq 1 && "$1" == "sudo" ]] && check_cmd="pkexec"
    if [[ "${IN_TOOLBOX}" -eq 1 ]]; then
        if ! flatpak-spawn --host -- command -v "${check_cmd}" >/dev/null 2>&1; then
            echo "missing required command on host: ${check_cmd}" >&2
            exit 1
        fi
    else
        need_cmd "${check_cmd}"
    fi
}

run() {
    printf '+'
    printf ' %q' "$@"
    printf '\n'
    if [[ "${DRY_RUN}" -eq 0 ]]; then
        "$@"
    fi
}

# Route an unprivileged command to the host when running inside a toolbox
# (pactl/systemctl talk to the per-user audio/session bus that toolbox shares
# with the host). Outside a toolbox it just runs locally.
run_user_host() {
    if [[ "${IN_TOOLBOX}" -eq 1 ]]; then
        flatpak-spawn --host -- "$@"
    else
        "$@"
    fi
}

# True if COMMAND exists where unprivileged host commands run (the host when in
# a toolbox, otherwise the local environment).
host_has_cmd() {
    if [[ "${IN_TOOLBOX}" -eq 1 ]]; then
        flatpak-spawn --host -- command -v "$1" >/dev/null 2>&1
    else
        command -v "$1" >/dev/null 2>&1
    fi
}

# Privileged host operations: udev lives on the real host, not the container's
# isolated view of /etc and /run/udev. Rather than elevate once per command
# (each pkexec/sudo call can trigger its own auth prompt), queue them with
# queue_host and run the whole batch in a single elevated shell via flush_host.
#
# In a toolbox, escalation uses pkexec (polkit GUI agent) instead of sudo:
# flatpak-spawn --host relays stdio as pipes, so sudo can't prompt for a
# password there ("a terminal is required to read the password"), whereas
# pkexec authenticates via the session's polkit agent.
HOST_CMDS=()

queue_host() {
    HOST_CMDS+=("$(printf '%q ' "$@")")
    printf '+'
    [[ "${IN_TOOLBOX}" -eq 1 ]] && printf ' [host]'
    printf ' %q' "$@"
    printf '\n'
}

flush_host() {
    [[ "${#HOST_CMDS[@]}" -eq 0 ]] && return 0
    local script
    script="set -e; $(printf '%s; ' "${HOST_CMDS[@]}")"
    if [[ "${DRY_RUN}" -eq 0 ]]; then
        if [[ "${IN_TOOLBOX}" -eq 1 ]]; then
            echo "Applying host udev changes (authenticate once via polkit)..."
            flatpak-spawn --host -- pkexec /bin/sh -c "${script}"
        else
            sudo /bin/sh -c "${script}"
        fi
    fi
    HOST_CMDS=()
}

audio_service_state() {
    if command -v systemctl >/dev/null 2>&1; then
        if systemctl --user --quiet is-active wireplumber 2>/dev/null ||
           systemctl --user --quiet is-active pipewire 2>/dev/null; then
            echo "pipewire"
            return
        fi
    fi
    if host_has_cmd pactl && run_user_host pactl info 2>/dev/null | grep -qi '^Server Name:.*PulseAudio'; then
        echo "pulseaudio"
        return
    fi
    echo "unknown"
}

retrigger_dspi_cards() {
    local found=0
    local card real

    [[ -d /sys/class/sound ]] || return 0
    for card in /sys/class/sound/card*; do
        [[ -e "${card}/id" ]] || continue
        [[ "$(cat "${card}/id")" == "DSPi" ]] || continue
        real="$(readlink -f "${card}")"
        found=1
        queue_host udevadm trigger --action=change "${real}"
    done

    if [[ "${found}" -eq 0 ]]; then
        echo "DSPi sound card is not currently present."
    fi
}

restart_audio() {
    local server
    server="$(audio_service_state)"

    if [[ "${NO_RESTART}" -eq 1 ]]; then
        echo "Skipping audio service restart (--no-restart). Replug DSPi or restart audio manually."
        return 0
    fi

    case "${server}" in
        pipewire)
            echo "Restarting user PipeWire services..."
            run systemctl --user restart wireplumber pipewire pipewire-pulse
            ;;
        pulseaudio)
            echo "Restarting PulseAudio..."
            if command -v systemctl >/dev/null 2>&1 &&
               systemctl --user list-unit-files pulseaudio.service >/dev/null 2>&1; then
                run systemctl --user restart pulseaudio.service
            else
                run pulseaudio -k
            fi
            ;;
        *)
            echo "Could not detect PipeWire or PulseAudio. Replug DSPi or restart your audio session manually."
            ;;
    esac
}

if [[ "${IN_TOOLBOX}" -eq 1 ]]; then
    need_cmd flatpak-spawn
    echo "Detected toolbox environment; udev steps will run on the host via flatpak-spawn."
fi

need_host_cmd sudo
need_host_cmd udevadm

CONFIG_HOME="${XDG_CONFIG_HOME:-${HOME}/.config}"

LOCAL_TARGETS=(
    "${CONFIG_HOME}/alsa-card-profile/mixer/profile-sets/${PROFILE_NAME}"
    "${CONFIG_HOME}/alsa-card-profile/mixer/paths/${PATH_NAME}"
)

# Legacy/system locations from older versions of this installer, or a
# system-wide install performed directly on a non-immutable host.
HOST_TARGETS=(
    "/usr/share/alsa-card-profile/mixer/profile-sets/${PROFILE_NAME}"
    "/usr/share/alsa-card-profile/mixer/paths/${PATH_NAME}"
    "/usr/share/pulseaudio/alsa-mixer/profile-sets/${PROFILE_NAME}"
    "/usr/share/pulseaudio/alsa-mixer/paths/${PATH_NAME}"
    "/etc/udev/rules.d/${RULE_NAME}"
)

echo "Removing DSPi ACP files..."
for target in "${LOCAL_TARGETS[@]}"; do
    if [[ -e "${target}" || "${DRY_RUN}" -eq 1 ]]; then
        run rm -f "${target}"
    fi
done
for target in "${HOST_TARGETS[@]}"; do
    queue_host rm -f "${target}"
done

echo "Reloading udev rules..."
queue_host udevadm control --reload-rules

echo "Retriggering DSPi sound card if present..."
retrigger_dspi_cards

flush_host

restart_audio

echo "Removed DSPi IEC958 profile."
